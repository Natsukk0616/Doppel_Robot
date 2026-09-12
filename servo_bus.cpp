#include "servo_bus.h"
#include <string.h>
#include <stdlib.h>

namespace {
constexpr int SERVO_RX_PIN = 1;
constexpr int SERVO_TX_PIN = 2;
constexpr uint32_t SERVO_BAUD = 1000000;
constexpr uint8_t INST_PING = 1;
constexpr uint8_t INST_READ = 2;
constexpr uint8_t INST_WRITE = 3;
constexpr uint8_t ADDR_MODEL_L = 3;
constexpr uint8_t ADDR_ACC = 41;
constexpr uint8_t ADDR_PRESENT_POSITION_L = 56;
constexpr uint8_t BROADCAST_ID = 0xFE;
HardwareSerial ServoBus(1);

uint8_t checksum(const uint8_t *packet, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 2; i < n; ++i) sum += packet[i];
  return static_cast<uint8_t>(~sum);
}

void flushRx() { while (ServoBus.available()) ServoBus.read(); }

bool readStatus(uint8_t id, uint8_t *payload, size_t capacity,
                size_t &payloadLength, uint8_t &error) {
  uint8_t packet[256];
  size_t count = 0;
  payloadLength = 0;
  error = 0;
  const uint32_t deadline = millis() + 80;
  while (millis() < deadline) {
    while (ServoBus.available() && count < sizeof(packet)) {
      packet[count++] = static_cast<uint8_t>(ServoBus.read());
      if (count >= 2 && packet[count - 2] == 0xFF && packet[count - 1] == 0xFF) {
        packet[0] = 0xFF; packet[1] = 0xFF; count = 2; break;
      }
    }
    if (count < 4) { delay(1); continue; }
    const uint8_t length = packet[3];
    const size_t total = static_cast<size_t>(length) + 4;
    if (length < 2 || total > sizeof(packet)) return false;
    while (count < total && millis() < deadline) {
      if (ServoBus.available()) packet[count++] = static_cast<uint8_t>(ServoBus.read());
    }
    if (count < total || packet[2] != id) return false;
    if (checksum(packet, total - 1) != packet[total - 1]) return false;
    error = packet[4];
    payloadLength = length - 2;
    if (payloadLength > capacity) return false;
    for (size_t i = 0; i < payloadLength; ++i) payload[i] = packet[5 + i];
    return true;
  }
  return false;
}

bool transact(uint8_t id, uint8_t instruction, const uint8_t *params,
              size_t paramLength, uint8_t *payload, size_t capacity,
              size_t &payloadLength, uint8_t &error) {
  if (paramLength > 240) return false;
  uint8_t packet[256] = {0};
  const size_t total = paramLength + 6;
  packet[0] = packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = static_cast<uint8_t>(paramLength + 2);
  packet[4] = instruction;
  for (size_t i = 0; i < paramLength; ++i) packet[5 + i] = params[i];
  packet[total - 1] = checksum(packet, total - 1);
  flushRx();
  ServoBus.write(packet, total);
  ServoBus.flush();
  if (id == BROADCAST_ID) { payloadLength = 0; error = 0; return true; }
  return readStatus(id, payload, capacity, payloadLength, error);
}

bool readReg(uint8_t id, uint8_t address, uint8_t length, uint8_t *data,
             uint8_t &error) {
  const uint8_t params[] = {address, length};
  size_t got = 0;
  return transact(id, INST_READ, params, sizeof(params), data, length, got, error) && got == length;
}

bool writeReg(uint8_t id, uint8_t address, const uint8_t *data, uint8_t length,
              uint8_t &error) {
  uint8_t params[32] = {address};
  if (length > sizeof(params) - 1) return false;
  for (uint8_t i = 0; i < length; ++i) params[i + 1] = data[i];
  uint8_t response[1]; size_t got = 0;
  return transact(id, INST_WRITE, params, length + 1, response, sizeof(response), got, error);
}

uint16_t u16(const uint8_t *p) { return p[0] | (static_cast<uint16_t>(p[1]) << 8); }
int16_t signed15(uint16_t x) { return (x & 0x8000) ? -static_cast<int16_t>(x & 0x7FFF) : static_cast<int16_t>(x); }

void printError(uint8_t e) {
  if (!e) return;
  Serial.printf(" servo_error=0x%02X", e);
  if (e & 1) Serial.print(" VOLTAGE");
  if (e & 2) Serial.print(" ANGLE");
  if (e & 4) Serial.print(" OVERHEAT");
  if (e & 8) Serial.print(" OVERCURRENT");
  if (e & 32) Serial.print(" OVERLOAD");
}

void servoPing(uint8_t id) {
  uint8_t response[1]; size_t responseLength = 0; uint8_t error = 0;
  if (!transact(id, INST_PING, nullptr, 0, response, sizeof(response), responseLength, error)) {
    Serial.printf("SERVO_PING ID=%u FAILED", id); printError(error); Serial.println(); return;
  }
  uint8_t data[2]; error = 0;
  if (!readReg(id, ADDR_MODEL_L, 2, data, error)) {
    Serial.printf("SERVO_PING ID=%u FAILED", id); printError(error); Serial.println(); return;
  }
  Serial.printf("SERVO_PING ID=%u OK model=%u", id, u16(data)); printError(error); Serial.println();
}

void servoRead(uint8_t id) {
  uint8_t data[11]; uint8_t error = 0;
  if (!readReg(id, ADDR_PRESENT_POSITION_L, sizeof(data), data, error)) {
    Serial.printf("SERVO_READ ID=%u FAILED", id); printError(error); Serial.println(); return;
  }
  Serial.printf("SERVO_READ ID=%u pos=%u speed=%d load=%d voltage=%.1fV temp=%uC moving=%u",
                id, u16(data) & 0x7FFF, signed15(u16(data + 2)), signed15(u16(data + 4)),
                data[6] / 10.0f, data[7], data[10]);
  printError(error); Serial.println();
}

void servoMove(uint8_t id, int position, int speed, int acc) {
  position = constrain(position, 0, 4095);
  speed = constrain(speed, 0, 32767);
  acc = constrain(acc, 0, 255);
  const uint8_t data[] = {static_cast<uint8_t>(acc), static_cast<uint8_t>(position),
                          static_cast<uint8_t>(position >> 8), 0, 0,
                          static_cast<uint8_t>(speed), static_cast<uint8_t>(speed >> 8)};
  uint8_t error = 0;
  if (!writeReg(id, ADDR_ACC, data, sizeof(data), error)) {
    Serial.printf("SERVO_MOVE ID=%u FAILED", id); printError(error); Serial.println(); return;
  }
  Serial.printf("SERVO_MOVE ID=%u pos=%d speed=%d acc=%d OK\n", id, position, speed, acc);
}

void servoStop(uint8_t id) { servoMove(id, 2048, 0, 0); }

void servoHelp() {
  Serial.println("SERVO_HELP: SERVO_PING <id> | SERVO_READ <id> | SERVO_MOVE <id> <pos> [speed] [acc] | SERVO_CENTER <id> | SERVO_STOP <id> | SERVO_SCAN [max_id]");
}
}

void initServoBus() {
  ServoBus.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  Serial.printf("Servo bus ready: RX=GPIO%d TX=GPIO%d baud=1000000\n", SERVO_RX_PIN, SERVO_TX_PIN);
  Serial.println("Servo bus has no automatic motion; use SERVO_HELP");
}

bool handleServoCommand(char *line) {
  char *cmd = strtok(line, " ,\t\r\n");
  if (!cmd) return false;
  if (strcasecmp(cmd, "SERVO_HELP") == 0) { servoHelp(); return true; }
  if (strcasecmp(cmd, "SERVO_PING") == 0) {
    char *a = strtok(nullptr, " ,\t\r\n"); if (a) servoPing(atoi(a)); else Serial.println("ERR usage: SERVO_PING <id>"); return true;
  }
  if (strcasecmp(cmd, "SERVO_READ") == 0) {
    char *a = strtok(nullptr, " ,\t\r\n"); if (a) servoRead(atoi(a)); else Serial.println("ERR usage: SERVO_READ <id>"); return true;
  }
  if (strcasecmp(cmd, "SERVO_MOVE") == 0) {
    char *a = strtok(nullptr, " ,\t\r\n"); char *p = strtok(nullptr, " ,\t\r\n");
    char *s = strtok(nullptr, " ,\t\r\n"); char *ac = strtok(nullptr, " ,\t\r\n");
    if (a && p) servoMove(atoi(a), atoi(p), s ? atoi(s) : 300, ac ? atoi(ac) : 20);
    else Serial.println("ERR usage: SERVO_MOVE <id> <pos> [speed] [acc]");
    return true;
  }
  if (strcasecmp(cmd, "SERVO_CENTER") == 0) {
    char *a = strtok(nullptr, " ,\t\r\n"); if (a) servoMove(atoi(a), 2048, 300, 20); else Serial.println("ERR usage: SERVO_CENTER <id>"); return true;
  }
  if (strcasecmp(cmd, "SERVO_STOP") == 0) {
    char *a = strtok(nullptr, " ,\t\r\n"); if (a) servoStop(atoi(a)); else Serial.println("ERR usage: SERVO_STOP <id>"); return true;
  }
  if (strcasecmp(cmd, "SERVO_SCAN") == 0) {
    char *a = strtok(nullptr, " ,\t\r\n"); int maxId = a ? constrain(atoi(a), 1, 20) : 20;
    for (int id = 1; id <= maxId; ++id) servoPing(id);
    return true;
  }
  return false;
}
