#!/usr/bin/env python
"""Low-speed, timed continuous-rotation test for one STS3215.

Run with the servo shaft unloaded. The script always sends a stop command
before closing the port, including after Ctrl+C.
"""

import sys
import time

sys.path.append("..")
from scservo_sdk import *


SCS_ID = 1
BAUDRATE = 1000000
DEVICENAME = "COM5"

ACC = 30
TEST_SPEED = 300
RUN_SECONDS = 2.0


def ok(packet_handler, comm_result, servo_error):
    if comm_result != COMM_SUCCESS:
        print(packet_handler.getTxRxResult(comm_result))
        return False
    if servo_error != 0:
        print(packet_handler.getRxPacketError(servo_error))
        return False
    return True


port_handler = PortHandler(DEVICENAME)
packet_handler = sms_sts(port_handler)
port_open = False

try:
    if not port_handler.openPort():
        raise RuntimeError(f"无法打开串口 {DEVICENAME}")
    port_open = True
    print(f"串口已打开: {DEVICENAME}")

    if not port_handler.setBaudRate(BAUDRATE):
        raise RuntimeError(f"无法设置波特率 {BAUDRATE}")

    model, comm_result, servo_error = packet_handler.ping(SCS_ID)
    if not ok(packet_handler, comm_result, servo_error):
        print("Ping 失败，停止测试，不切换连续旋转模式。")
        raise SystemExit(2)
    print(f"Ping 成功: ID={SCS_ID}, model={model}")

    input("确认舵机输出轴悬空、无轮子和无联轴器后，按 Enter 开始：")

    comm_result, servo_error = packet_handler.WheelMode(SCS_ID)
    if not ok(packet_handler, comm_result, servo_error):
        raise SystemExit(3)
    print("已切换到连续旋转模式。")

    for label, speed in (("正转", TEST_SPEED), ("停止", 0), ("反转", -TEST_SPEED), ("停止", 0)):
        print(f"{label}: speed={speed}")
        comm_result, servo_error = packet_handler.WriteSpec(SCS_ID, speed, ACC)
        if not ok(packet_handler, comm_result, servo_error):
            raise SystemExit(4)

        end_time = time.monotonic() + RUN_SECONDS
        while time.monotonic() < end_time:
            position, present_speed, comm_result, servo_error = packet_handler.ReadPosSpeed(SCS_ID)
            if not ok(packet_handler, comm_result, servo_error):
                raise SystemExit(5)
            print(
                f"  position={position}, "
                f"feedback_speed={present_speed}"
            )
            time.sleep(0.2)

finally:
    if port_open:
        try:
            # Stop before closing even if the user presses Ctrl+C.
            packet_handler.WriteSpec(SCS_ID, 0, ACC)
        except Exception as error:
            print(f"发送停止命令失败: {error}")
        port_handler.closePort()
        print("已发送停止命令，串口已关闭。")
