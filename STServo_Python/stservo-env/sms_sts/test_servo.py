#!/usr/bin/env python
"""Safe single STS3215 bench test.

Sequence: open port -> ping -> read position/speed -> small position moves.
No ID, baudrate, or wheel mode settings are changed.
"""

import sys
import time

sys.path.append("..")
from scservo_sdk import *


# Windows configuration
SCS_ID = 1
BAUDRATE = 1000000
DEVICENAME = "COM5"

# Small movement around the center; keep the servo unloaded.
TEST_POSITIONS = [1900, 2200, 2048]
MOVING_SPEED = 600
MOVING_ACC = 30


def show_result(packet_handler, comm_result, servo_error):
    if comm_result != COMM_SUCCESS:
        print(packet_handler.getTxRxResult(comm_result))
        return False
    if servo_error != 0:
        print(packet_handler.getRxPacketError(servo_error))
        return False
    return True


port_handler = PortHandler(DEVICENAME)
packet_handler = sms_sts(port_handler)

try:
    if not port_handler.openPort():
        raise RuntimeError(f"无法打开串口 {DEVICENAME}")
    print(f"串口已打开: {DEVICENAME}")

    if not port_handler.setBaudRate(BAUDRATE):
        raise RuntimeError(f"无法设置波特率 {BAUDRATE}")
    print(f"波特率: {BAUDRATE}")

    # 1. Communication test. Stop before motion if there is no reply.
    model_number, comm_result, servo_error = packet_handler.ping(SCS_ID)
    if not show_result(packet_handler, comm_result, servo_error):
        print(f"ID={SCS_ID} 没有返回状态包，停止测试，不执行运动。")
        raise SystemExit(2)
    print(f"Ping 成功: ID={SCS_ID}, model={model_number}")

    # 2. Read current position and speed.
    position, speed, comm_result, servo_error = packet_handler.ReadPosSpeed(SCS_ID)
    if not show_result(packet_handler, comm_result, servo_error):
        raise SystemExit(3)
    print(f"当前位置: {position}, 当前速度: {packet_handler.scs_tohost(speed, 15)}")

    input("确认舵机裸机、无轮子和联轴器后，按 Enter 开始小角度测试；Ctrl+C 退出：")

    # 3. Small position-mode movements.
    for target in TEST_POSITIONS:
        print(f"移动到目标位置: {target}")
        comm_result, servo_error = packet_handler.WritePosEx(
            SCS_ID, target, MOVING_SPEED, MOVING_ACC
        )
        if not show_result(packet_handler, comm_result, servo_error):
            raise SystemExit(4)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            position, speed, comm_result, servo_error = packet_handler.ReadPosSpeed(SCS_ID)
            if not show_result(packet_handler, comm_result, servo_error):
                raise SystemExit(5)
            print(
                f"  position={position}, "
                f"speed={packet_handler.scs_tohost(speed, 15)}"
            )
            moving, comm_result, servo_error = packet_handler.ReadMoving(SCS_ID)
            if not show_result(packet_handler, comm_result, servo_error):
                raise SystemExit(6)
            if moving == 0:
                break
            time.sleep(0.1)

    print("单舵机测试完成。")

except KeyboardInterrupt:
    print("\n用户中断，停止测试。")
finally:
    port_handler.closePort()
    print("串口已关闭。")

