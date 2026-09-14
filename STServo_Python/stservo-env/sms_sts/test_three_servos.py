#!/usr/bin/env python
"""Run a synchronized 5-second position test for servos ID 1, 2, and 3."""

import sys
import time

sys.path.append("..")
from scservo_sdk import *


SERVO_IDS = (1,2,3,4, 5, 6)
BAUDRATE = 1000000
DEVICENAME = "COM5"

# Keep the test within a small range around the center position.
CENTER_POSITION = 2048
MOVING_SPEED = 600
MOVING_ACC = 30
TEST_DURATION = 5.0

# Per-servo position offsets for angle calibration.
# Reduce the offset for servo 1 because it currently moves farther.
POSITION_OFFSETS = {1: 800, 2: 800, 3: 800}


def report_result(packet_handler, servo_id, comm_result, servo_error):
    """Print a communication result and return whether it succeeded."""
    if comm_result != COMM_SUCCESS:
        print(f"[ID:{servo_id:03d}] communication failed: {packet_handler.getTxRxResult(comm_result)}")
        return False
    if servo_error != 0:
        print(f"[ID:{servo_id:03d}] servo error: {packet_handler.getRxPacketError(servo_error)}")
        return False
    return True


def sync_move(packet_handler, target_positions):
    """Send one synchronized position command to all three servos."""
    for servo_id in SERVO_IDS:
        position = target_positions[servo_id]
        if not packet_handler.SyncWritePosEx(servo_id, position, MOVING_SPEED, MOVING_ACC):
            print(f"[ID:{servo_id:03d}] failed to add sync-write parameters")
            packet_handler.groupSyncWrite.clearParam()
            return False

    comm_result = packet_handler.groupSyncWrite.txPacket()
    packet_handler.groupSyncWrite.clearParam()
    if comm_result != COMM_SUCCESS:
        print(packet_handler.getTxRxResult(comm_result))
        return False
    return True



def stop_all_servos(packet_handler):
    """Disable torque on all tested servos so they stop immediately."""
    for servo_id in SERVO_IDS:
        comm_result, servo_error = packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_TORQUE_ENABLE, 0
        )
        if comm_result != COMM_SUCCESS:
            print(f"[ID:{servo_id:03d}] stop command failed: {packet_handler.getTxRxResult(comm_result)}")
        elif servo_error != 0:
            print(f"[ID:{servo_id:03d}] stop command error: {packet_handler.getRxPacketError(servo_error)}")
        else:
            print(f"[ID:{servo_id:03d}] stopped (torque disabled)")

port_handler = PortHandler(DEVICENAME)
packet_handler = sms_sts(port_handler)

try:
    if not port_handler.openPort():
        raise RuntimeError(f"Cannot open serial port {DEVICENAME}")
    print(f"Serial port opened: {DEVICENAME}")

    if not port_handler.setBaudRate(BAUDRATE):
        raise RuntimeError(f"Cannot set baudrate {BAUDRATE}")

    print(f"Baudrate set: {BAUDRATE}")
    print("Checking servo communication...")
    for servo_id in SERVO_IDS:
        model_number, comm_result, servo_error = packet_handler.ping(servo_id)
        if not report_result(packet_handler, servo_id, comm_result, servo_error):
            raise RuntimeError(f"Servo ID {servo_id} did not respond; motion test cancelled")
        print(f"[ID:{servo_id:03d}] ping succeeded, model={model_number}")

    print("Setting all servos to position mode...")
    for servo_id in SERVO_IDS:
        comm_result, servo_error = packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_TORQUE_ENABLE, 0
        )
        if not report_result(packet_handler, servo_id, comm_result, servo_error):
            raise RuntimeError(f"Could not disable torque for servo ID {servo_id}")

        comm_result, servo_error = packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_MODE, 0
        )
        if not report_result(packet_handler, servo_id, comm_result, servo_error):
            raise RuntimeError(f"Could not set position mode for servo ID {servo_id}")

        comm_result, servo_error = packet_handler.write1ByteTxRx(
            servo_id, SMS_STS_TORQUE_ENABLE, 1
        )
        if not report_result(packet_handler, servo_id, comm_result, servo_error):
            raise RuntimeError(f"Could not enable torque for servo ID {servo_id}")

    print(f"Starting synchronized test for {TEST_DURATION:.0f} seconds. Press Ctrl+C to stop.")
    # Read each current position and give every servo its own target.
    # This prevents servos already at the same target from appearing idle.
    target_positions = {}
    for servo_id in SERVO_IDS:
        current_position, comm_result, servo_error = packet_handler.ReadPos(servo_id)
        if not report_result(packet_handler, servo_id, comm_result, servo_error):
            raise RuntimeError(f"Could not read position of servo ID {servo_id}")
        position_offset = POSITION_OFFSETS.get(servo_id, 800)
        offset = position_offset if current_position < CENTER_POSITION else -position_offset
        target_positions[servo_id] = max(300, min(3800, current_position + offset))
        print(f"[ID:{servo_id:03d}] current={current_position}, target={target_positions[servo_id]}")

    # Send one target only; repeatedly changing targets can make a servo oscillate.
    if not sync_move(packet_handler, target_positions):
        raise RuntimeError("Synchronized movement failed")
    time.sleep(TEST_DURATION)

    print("Three-servo test completed.")

except KeyboardInterrupt:
    print("\\nTest interrupted by user.")
finally:
    try:
        stop_all_servos(packet_handler)
    except Exception as error:
        print(f"Failed to stop servos: {error}")
    port_handler.closePort()
    print("Serial port closed.")
