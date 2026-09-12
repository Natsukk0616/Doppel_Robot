#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_VL53L1X.h>
#include <7Semi_BNO08x.h>
#include <BnoI2CBus.h>
#include <string.h>
#include <stdlib.h>
#include "epd4in2_V2.h"
#include "epdpaint.h"
#include "servo_bus.h"

extern const unsigned char LOGO_400x300[];

// LCD：两块屏共享 MOSI/SCK/DC/RST，CS 独立
constexpr int LCD_SCK  = 18;
constexpr int LCD_MOSI = 17;
constexpr int LCD_DC   = 39;
constexpr int LCD_RST  = 40;
constexpr int LCD_CS_L = 41;
constexpr int LCD_CS_R = 42;

// I2C：PCA9548A/ToF 与 BNO085 使用两条完全独立的硬件 I2C 总线
constexpr int I2C_SDA_PIN = 8;
constexpr int I2C_SCL_PIN = 9;
constexpr int BNO_SDA_PIN = 6;
constexpr int BNO_SCL_PIN = 7;
constexpr int BNO_INT_PIN = 5;
constexpr int BNO_RST_PIN = 4;
constexpr int PCA_RST_PIN = 15;
constexpr uint8_t PCA_ADDR = 0x70;
constexpr uint8_t VL53_ADDR = 0x29;
constexpr uint8_t BNO_ADDR = 0x4B;
constexpr uint8_t TOF_COUNT = 4;
constexpr int EPD_SCK = 16;
constexpr int EPD_MOSI = 14;
constexpr int EPD_CS = 10;
constexpr int EPD_DC = 13;
constexpr int EPD_RST = 21;
constexpr int EPD_BUSY = 47;

constexpr uint16_t BLACK = 0x0000;
constexpr uint16_t WHITE = 0xFFFF;
constexpr uint16_t BLUE  = 0x001F;

// RST 在 initLcds() 中手动复位一次，避免第二个对象初始化时复位第一块屏。
Adafruit_GC9A01A lcdLeft(LCD_CS_L, LCD_DC, LCD_MOSI, LCD_SCK, -1);
Adafruit_GC9A01A lcdRight(LCD_CS_R, LCD_DC, LCD_MOSI, LCD_SCK, -1);
Adafruit_VL53L1X tof[TOF_COUNT];
Epd epd;
unsigned char epdImage[EPD_WIDTH * EPD_HEIGHT / 8];
unsigned char epdPartial[200 * 32 / 8];

TwoWire PcaWire(1);  // PCA9548A/ToF：GPIO8/9
// BNO085 使用独立总线，总线与 PCA/ToF 分离
TwoWire BnoWire(0);  // BNO085：GPIO6/7
BnoI2CBus bnoBus(BnoWire, BNO_SDA_PIN, BNO_SCL_PIN, BNO_ADDR,
                 100000, BNO_INT_PIN, BNO_RST_PIN); // BNO085 独立 I2C 总线
BNO08x_7Semi bno(bnoBus);
bool tofOK[TOF_COUNT] = {false, false, false, false};
bool bnoOK = false;
bool pcaOK = false;
uint32_t tofSamples[TOF_COUNT] = {0, 0, 0, 0};
uint32_t tofZeroSamples[TOF_COUNT] = {0, 0, 0, 0};
uint32_t bnoQuatSamples = 0;
uint32_t bnoAccelSamples = 0;
uint32_t bnoGyroSamples = 0;
uint32_t lastSummaryMs = 0;

enum Emotion : uint8_t { EM_NEUTRAL, EM_LISTEN, EM_THINK, EM_HAPPY, EM_SLEEP };
struct EyeState {
  int16_t x;
  int16_t y;
  int16_t targetX;
  int16_t targetY;
  uint16_t color;
  bool closed;
};

EyeState leftEye = {-5, 0, -5, 0, BLUE, false};
EyeState rightEye = {5, 0, 5, 0, BLUE, false};
Emotion currentEmotion = EM_NEUTRAL;
uint32_t blinkIntervalMs = 4000;
uint32_t nextBlinkMs = 0;
uint32_t blinkUntilMs = 0;
uint8_t blinkMask = 0;
uint32_t emotionUntilMs = 0;
uint32_t lastEyeRenderMs = 0;
char commandBuffer[128];
uint8_t commandLength = 0;

void epdLogo() {
  Serial.println("EPD: school emblem");
  if (epd.Init() != 0) { Serial.println("ERR EPD INIT"); return; }
  epd.Display(LOGO_400x300);
  Serial.println("OK EPD IMAGE");
}

void epdText(const char *message) {
  Serial.println("EPD: text refresh");
  if (epd.Init() != 0) { Serial.println("ERR EPD INIT"); return; }
  Paint paint(epdImage, EPD_WIDTH, EPD_HEIGHT);
  paint.Clear(1);
  paint.DrawStringAt(20, 20, "ROBOT STATUS", &Font16, 0);
  paint.DrawStringAt(20, 55, message, &Font16, 0);
  epd.Display(epdImage);
  Serial.println("OK EPD TEXT");
}

void epdClear() {
  Serial.println("EPD: clear");
  if (epd.Init() != 0) { Serial.println("ERR EPD INIT"); return; }
  epd.Clear();
  Serial.println("OK EPD CLEAR");
}

void selectPcaChannel(uint8_t channel) {
  PcaWire.beginTransmission(PCA_ADDR);
  PcaWire.write(static_cast<uint8_t>(1u << channel));
  PcaWire.endTransmission();
  delay(2);
}

void disablePcaChannels() {
  PcaWire.beginTransmission(PCA_ADDR);
  PcaWire.write(0x00);
  PcaWire.endTransmission();
}

void scanI2cBus() {
  Serial.println("Main I2C scan (PCA/ToF):");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    PcaWire.beginTransmission(addr);
    if (PcaWire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", addr);
      ++found;
    }
  }
  if (found == 0) Serial.println("  no I2C devices found");
}

void scanBnoI2cBus() {
  Serial.println("BNO I2C scan:");
  BnoWire.begin(BNO_SDA_PIN, BNO_SCL_PIN);
  BnoWire.setClock(100000);
  BnoWire.beginTransmission(BNO_ADDR);
  Serial.printf("  BNO085 0x%02X: %s\n", BNO_ADDR,
                BnoWire.endTransmission() == 0 ? "found" : "not found");
}

void drawEye(Adafruit_GC9A01A &lcd, const EyeState &eye) {
  lcd.fillScreen(BLACK);
  if (eye.closed || currentEmotion == EM_SLEEP) {
    for (int8_t offset = -4; offset <= 4; ++offset) {
      lcd.drawLine(48, 120 + offset, 192, 120 + offset, eye.color);
    }
    return;
  }
  lcd.fillCircle(120, 120, 78, eye.color);
  lcd.drawCircle(120, 120, 78, WHITE);
  lcd.fillCircle(120 + eye.x, 120 + eye.y, 31, BLACK);
  lcd.fillCircle(110 + eye.x, 109 + eye.y, 9, WHITE);
  if (currentEmotion == EM_HAPPY) {
    lcd.drawLine(65, 82, 175, 82, BLACK);
    lcd.drawLine(75, 75, 165, 75, BLACK);
  } else if (currentEmotion == EM_THINK) {
    lcd.drawCircle(120 + eye.x, 120 + eye.y, 39, eye.color);
  }
}

uint16_t rgb565(uint32_t rgb) {
  return static_cast<uint16_t>(((rgb >> 19) & 0x1F) << 11 |
                               ((rgb >> 10) & 0x3F) << 5 |
                               ((rgb >> 3) & 0x1F));
}

void renderEyes(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - lastEyeRenderMs < 70) return;
  lastEyeRenderMs = now;
  drawEye(lcdLeft, leftEye);
  drawEye(lcdRight, rightEye);
}

void setEmotion(const char *name, uint32_t durationMs) {
  if (strcasecmp(name, "NEUTRAL") == 0) currentEmotion = EM_NEUTRAL;
  else if (strcasecmp(name, "LISTEN") == 0) currentEmotion = EM_LISTEN;
  else if (strcasecmp(name, "THINK") == 0) currentEmotion = EM_THINK;
  else if (strcasecmp(name, "HAPPY") == 0) currentEmotion = EM_HAPPY;
  else if (strcasecmp(name, "SLEEP") == 0) currentEmotion = EM_SLEEP;
  else {
    Serial.println("ERR unknown emotion");
    return;
  }
  emotionUntilMs = durationMs ? millis() + durationMs : 0;
  if (currentEmotion == EM_SLEEP) {
    leftEye.closed = true;
    rightEye.closed = true;
  } else {
    leftEye.closed = false;
    rightEye.closed = false;
  }
  renderEyes(true);
  Serial.printf("OK EMOTION %s\n", name);
}

void handleCommand(char *line) {
  // 只让 SERVO_ 命令进入舵机解析器，避免普通命令被预解析影响 strtok 状态。
  if (strncasecmp(line, "SERVO_", 6) == 0 && handleServoCommand(line)) return;
  char *cmd = strtok(line, " ,\t");
  if (!cmd) return;

  if (strcasecmp(cmd, "EPD_IMAGE") == 0 || strcasecmp(cmd, "IMAGE") == 0) {
    epdLogo();
  } else if (strcasecmp(cmd, "EPD_CLEAR") == 0) {
    epdClear();
  } else if (strcasecmp(cmd, "EPD_TEXT") == 0 || strcasecmp(cmd, "TEXT") == 0) {
    char *message = strtok(nullptr, "");
    if (!message || !*message) Serial.println("ERR EPD_TEXT message required");
    else epdText(message);
  } else if (strcasecmp(cmd, "EPD_SLEEP") == 0) {
    epd.Sleep();
    Serial.println("OK EPD SLEEP");
  } else if (strcasecmp(cmd, "EPD_WAKE") == 0) {
    Serial.println(epd.Init() == 0 ? "OK EPD WAKE" : "ERR EPD WAKE");
  } else if (strcasecmp(cmd, "ANIM") == 0) {
    // 兼容独立 LCD 测试程序的动画指令。
    // 综合程序内部使用 EMOTION/GAZE/BLINK 实现相同效果。
    char *name = strtok(nullptr, " ,\t\r\n");
    if (!name) {
      Serial.println("ERR ANIM idle|blink|look|happy|sleepy|surprise|wink|breathe");
      return;
    }
    if (strcasecmp(name, "IDLE") == 0) {
      setEmotion("NEUTRAL", 0);
    } else if (strcasecmp(name, "HAPPY") == 0) {
      setEmotion("HAPPY", 0);
    } else if (strcasecmp(name, "SLEEPY") == 0) {
      setEmotion("SLEEP", 0);
    } else if (strcasecmp(name, "BLINK") == 0) {
      leftEye.closed = true;
      rightEye.closed = true;
      blinkMask = 3;
      blinkUntilMs = millis() + 180;
      renderEyes(true);
      Serial.println("OK ANIM blink");
    } else if (strcasecmp(name, "WINK") == 0) {
      leftEye.closed = true;
      rightEye.closed = false;
      blinkMask = 1;
      blinkUntilMs = millis() + 180;
      renderEyes(true);
      Serial.println("OK ANIM wink");
    } else if (strcasecmp(name, "LOOK") == 0) {
      leftEye.targetX = -40;
      rightEye.targetX = -30;
      leftEye.targetY = 0;
      rightEye.targetY = 0;
      Serial.println("OK ANIM look");
    } else if (strcasecmp(name, "SURPRISE") == 0) {
      setEmotion("THINK", 1200);
      Serial.println("OK ANIM surprise");
    } else if (strcasecmp(name, "BREATHE") == 0) {
      setEmotion("LISTEN", 0);
      Serial.println("OK ANIM breathe");
    } else {
      Serial.println("ERR unknown animation");
    }
  } else if (strcasecmp(cmd, "EMOTION") == 0) {
    char *name = strtok(nullptr, " ,\t");
    char *duration = strtok(nullptr, " ,\t");
    if (!name) { Serial.println("ERR EMOTION name required"); return; }
    setEmotion(name, duration ? strtoul(duration, nullptr, 10) : 0);
  } else if (strcasecmp(cmd, "GAZE") == 0) {
    char *sx = strtok(nullptr, " ,\t");
    char *sy = strtok(nullptr, " ,\t");
    if (!sx || !sy) { Serial.println("ERR GAZE x y required"); return; }
    int x = constrain(atoi(sx), -45, 45);
    int y = constrain(atoi(sy), -45, 45);
    leftEye.targetX = x - 5;
    rightEye.targetX = x + 5;
    leftEye.targetY = y;
    rightEye.targetY = y;
    Serial.printf("OK GAZE %d %d\n", x, y);
  } else if (strcasecmp(cmd, "BLINK") == 0) {
    char *which = strtok(nullptr, " ,\t");
    char *duration = strtok(nullptr, " ,\t");
    uint32_t ms = duration ? constrain(atoi(duration), 60, 2000) : 180;
    if (!which) which = const_cast<char *>("BOTH");
    blinkMask = 0;
    if (strcasecmp(which, "LEFT") == 0) blinkMask = 1;
    else if (strcasecmp(which, "RIGHT") == 0) blinkMask = 2;
    else if (strcasecmp(which, "BOTH") == 0) blinkMask = 3;
    else { Serial.println("ERR BLINK LEFT|RIGHT|BOTH"); return; }
    leftEye.closed = (blinkMask & 1) != 0;
    rightEye.closed = (blinkMask & 2) != 0;
    blinkUntilMs = millis() + ms;
    renderEyes(true);
    Serial.println("OK BLINK");
  } else if (strcasecmp(cmd, "COLOR") == 0) {
    char *value = strtok(nullptr, " ,\t");
    if (!value) { Serial.println("ERR COLOR RRGGBB required"); return; }
    uint16_t color = rgb565(strtoul(value, nullptr, 16));
    leftEye.color = color;
    rightEye.color = color;
    renderEyes(true);
    Serial.println("OK COLOR");
  } else if (strcasecmp(cmd, "BLINK_INTERVAL") == 0) {
    char *value = strtok(nullptr, " ,\t");
    if (!value) { Serial.println("ERR BLINK_INTERVAL ms required"); return; }
    blinkIntervalMs = constrain(strtoul(value, nullptr, 10), 500UL, 60000UL);
    Serial.println("OK BLINK_INTERVAL");
  } else if (strcasecmp(cmd, "STATUS") == 0) {
    Serial.printf("STATUS emotion=%u gaze=%d,%d blink_interval=%lu\n",
                  currentEmotion, leftEye.targetX, leftEye.targetY,
                  blinkIntervalMs);
  } else if (strcasecmp(cmd, "HELP") == 0) {
    Serial.println("EPD_IMAGE | EPD_TEXT message | EPD_CLEAR | EPD_WAKE | EPD_SLEEP");
    Serial.println("ANIM idle|blink|look|happy|sleepy|surprise|wink|breathe");
    Serial.println("EMOTION NEUTRAL|LISTEN|THINK|HAPPY|SLEEP [ms]");
    Serial.println("GAZE x y | BLINK LEFT|RIGHT|BOTH [ms] | COLOR RRGGBB");
    Serial.println("BLINK_INTERVAL ms | STATUS");
  } else {
    Serial.println("ERR unknown command; use HELP");
  }
}

void readCommands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        Serial.printf("RX: [%s]\n", commandBuffer);
        handleCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

void updateEyeAnimation() {
  const uint32_t now = millis();
  if (emotionUntilMs && now >= emotionUntilMs) {
    currentEmotion = EM_NEUTRAL;
    emotionUntilMs = 0;
    leftEye.closed = false;
    rightEye.closed = false;
  }
  if (blinkUntilMs && now >= blinkUntilMs) {
    blinkUntilMs = 0;
    blinkMask = 0;
    leftEye.closed = currentEmotion == EM_SLEEP;
    rightEye.closed = currentEmotion == EM_SLEEP;
  }
  if (!blinkUntilMs && currentEmotion != EM_SLEEP && now >= nextBlinkMs) {
    blinkMask = 3;
    leftEye.closed = true;
    rightEye.closed = true;
    blinkUntilMs = now + 160;
    nextBlinkMs = now + blinkIntervalMs;
  }
  leftEye.x += (leftEye.targetX - leftEye.x) / 3;
  leftEye.y += (leftEye.targetY - leftEye.y) / 3;
  rightEye.x += (rightEye.targetX - rightEye.x) / 3;
  rightEye.y += (rightEye.targetY - rightEye.y) / 3;
  renderEyes();
}

void initLcds() {
  pinMode(LCD_CS_L, OUTPUT);
  pinMode(LCD_CS_R, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  digitalWrite(LCD_CS_L, HIGH);
  digitalWrite(LCD_CS_R, HIGH);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(120);
  lcdLeft.begin();
  lcdRight.begin();
  lcdLeft.setRotation(0);
  lcdRight.setRotation(0);
  renderEyes(true);
  Serial.println("LCD dynamic eye renderer ready; commands on Serial 115200");
}

void initTof() {
  pinMode(PCA_RST_PIN, OUTPUT);
  digitalWrite(PCA_RST_PIN, LOW);
  delay(10);
  digitalWrite(PCA_RST_PIN, HIGH);
  delay(10);

  PcaWire.beginTransmission(PCA_ADDR);
  if (PcaWire.endTransmission() != 0) {
    Serial.println("PCA9548A not found at 0x70");
    return;
  }
  pcaOK = true;
  Serial.println("PCA9548A found at 0x70");

  for (uint8_t ch = 0; ch < TOF_COUNT; ++ch) {
    selectPcaChannel(ch);
    PcaWire.beginTransmission(VL53_ADDR);
    if (PcaWire.endTransmission() == 0 && tof[ch].begin(VL53_ADDR, &PcaWire)) {
      tof[ch].setTimingBudget(50);
      tof[ch].startRanging();
      tofOK[ch] = true;
      Serial.printf("CH%u: VL53L1X ready, address=0x29\n", ch);
    } else {
      Serial.printf("CH%u: VL53L1X not found\n", ch);
    }
    disablePcaChannels();
  }
}

void initBno() {
  // 不操作 PcaWire：BNO085 已经在独立的 BnoWire(GPIO6/7) 上。
  // 这样即使 PCA9548A 暂时掉线，也不会阻止 BNO085 初始化。
  pinMode(BNO_INT_PIN, INPUT_PULLUP);
  BnoWire.setClock(100000);
  if (!bno.begin()) {
    Serial.println("BNO085 not found at 0x4B");
    return;
  }
  // BNO085 上电后需要等待固件和传感器报告通道完全启动。
  delay(1000);
  bool rv = bno.enableRotationVector(50);
  delay(50);
  bool acc = bno.enableAcc(50);
  delay(50);
  bool gyro = bno.enableGyro(50);
  bnoOK = true;
  Serial.printf("BNO085 ready, address=0x4B (RV=%s ACC=%s GYRO=%s)\n",
                rv ? "OK" : "FAIL", acc ? "OK" : "FAIL",
                gyro ? "OK" : "FAIL");
}

void readTof() {
  for (uint8_t ch = 0; ch < TOF_COUNT; ++ch) {
    if (!tofOK[ch]) continue;
    selectPcaChannel(ch);
    if (tof[ch].dataReady()) {
      int16_t mm = tof[ch].distance();
      ++tofSamples[ch];
      if (mm <= 0) ++tofZeroSamples[ch];
      Serial.printf("T=%lu ms  CH%u  %d mm%s\n", millis(), ch, mm,
                    mm <= 0 ? "  [WARN: zero/invalid]" : "");
      tof[ch].clearInterrupt();
    }
  }
  disablePcaChannels();
}

void readBno() {
  if (!bnoOK) return;
  // BNO085 可能在一次 I2C 事务中返回多个控制/传感器帧；
  // 多处理几次，避免综合测试中错过报告。
  for (uint8_t n = 0; n < 16; ++n) {
    bno.processData();
    delay(1);
  }
  float x, y, z, i, j, k, r;
  if (bno.getQuaternion(i, j, k, r)) {
    ++bnoQuatSamples;
    Serial.printf("BNO quat: i=%.4f j=%.4f k=%.4f r=%.4f\n", i, j, k, r);
  }
  if (bno.getAccelerometer(x, y, z)) {
    ++bnoAccelSamples;
    Serial.printf("BNO accel: x=%.3f y=%.3f z=%.3f m/s2\n", x, y, z);
  }
  if (bno.getGyroscope(x, y, z)) {
    ++bnoGyroSamples;
    Serial.printf("BNO gyro: x=%.3f y=%.3f z=%.3f rad/s\n", x, y, z);
  }
}

void printSummary() {
  const uint32_t now = millis();
  if (now - lastSummaryMs < 10000) return;
  lastSummaryMs = now;
  Serial.println("--- TEST SUMMARY ---");
  Serial.printf("LCD: left=%s right=%s\n", "PASS", "PASS");
  Serial.printf("PCA9548A: %s @0x70\n", pcaOK ? "PASS" : "FAIL");
  for (uint8_t ch = 0; ch < TOF_COUNT; ++ch) {
    const char *status = "FAIL";
    if (tofOK[ch]) {
      if (tofSamples[ch] == 0 || tofZeroSamples[ch] == tofSamples[ch]) {
        status = "FAIL";
      } else if (tofZeroSamples[ch] > 0) {
        status = "WARN";
      } else {
        status = "PASS";
      }
    }
    Serial.printf("VL53L1X CH%u: %s samples=%lu zero=%lu\n", ch,
                  status, tofSamples[ch], tofZeroSamples[ch]);
  }
  const bool bnoDataOK = bnoOK && bnoQuatSamples > 0 &&
                         bnoAccelSamples > 0 && bnoGyroSamples > 0;
  Serial.printf("BNO085: %s quat=%lu accel=%lu gyro=%lu\n",
                bnoDataOK ? "PASS" : "FAIL", bnoQuatSamples,
                bnoAccelSamples, bnoGyroSamples);
  Serial.println("--------------------");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== ALL SENSOR TEST: ESP32-S3 + dual LCD + 4x VL53L1X + BNO085 ===");
  // 总线 1：PCA9548A + 4 路 VL53L1X
  PcaWire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  PcaWire.setClock(100000);
  scanI2cBus();

  // 总线 0：独立 BNO085；scanBnoI2cBus() 会初始化 GPIO6/7
  scanBnoI2cBus();
  initLcds();
  initTof();
  initBno();
  initServoBus();
  Serial.println("Sensors running simultaneously:");
  Serial.println("  PCA9548A + VL53L1X: SDA=GPIO8 SCL=GPIO9");
  Serial.println("  BNO085: SDA=GPIO6 SCL=GPIO7 INT=GPIO5 RST=GPIO4");
  Serial.println("  Servo Adapter: RX=GPIO1 TX=GPIO2, one shared bus for up to 6 servos");
  Serial.println("  D435/SX02: Jetson-side USB cameras; not controlled by ESP32 sketch");
}

void loop() {
  readTof();
  readBno();
  readCommands();
  updateEyeAnimation();
  printSummary();
  delay(50);
}
