# ESP32 全传感器综合测试

本目录把当前已验证的 ESP32 侧测试合并成一个 Arduino 工程：

- 双 GC9A01 SPI 圆屏：显示和动画验证
- PCA9548A + 4 路 VL53L1X：逐通道初始化、测距、零值统计
- BNO085：四元数、加速度、陀螺仪数据流
- 4.2 英寸电子纸：保留原有显示测试和串口命令
- Bus Servo Adapter (A) + 最多 6 只 STS3215：共用一条 UART 总线，按 ID 控制

## 引脚

```text
GC9A01：SCK=18 MOSI=17 DC=39 RST=40 CS_L=41 CS_R=42
PCA9548A/ToF：SDA=8 SCL=9 RST=15
BNO085：SDA=6 SCL=7 INT=5 RST=4 ADDR=0x4B
电子纸：SCK=16 MOSI=14 CS=10 DC=13 RST=21 BUSY=47
舵机适配器：RX=GPIO1 TX=GPIO2，UART=1 Mbps
```

## 运行与判定

串口监视器使用 `115200`。程序启动后会同时初始化各设备，并持续输出：

- `PCA9548A: PASS/FAIL`
- 每路 `VL53L1X CH0..CH3` 的距离、样本数和零值数
- `BNO085` 的四元数、加速度和陀螺仪数据
- LCD/电子纸初始化结果
- 舵机 Ping、反馈读取、小角度移动和停止结果

## 急停与硬切断

- GPIO11：舵机电源继电器或 MOSFET 使能，HIGH=供电，LOW=断电。
- GPIO12：外部急停输入，使用 `INPUT_PULLDOWN`，触发时接到 3.3V。
- 急停触发后会锁存，必须复位 ESP32 才能恢复。
- GPIO11 只能驱动继电器/MOSFET 的控制端，不能直接切换舵机 12V 电源。
- 建议在舵机电源回路增加硬件急停串联触点；软件 GPIO 急停不能替代硬件安全回路。

正常停止轮子：
```text
SERVO_WALL 0
```

急停触发后应立即断开舵机电源，并检查机械结构和接线后再复位。

## 舵机命令

完整命令说明见 [ESP32综合程序输入指令.md](ESP32综合程序输入指令.md)。

```text
SERVO_HELP
SERVO_PING <id>
SERVO_READ <id>
SERVO_MOVE <id> <position> [speed] [acc]
SERVO_CENTER <id>
SERVO_STOP <id>
SERVO_SCAN [max_id]
SERVO_PINGALL
SERVO_WHEEL <id> <speed>
SERVO_WSTOP <id>
SERVO_WALL <speed>
SERVO_HEAD <id> <pos> [speed] [acc]
SERVO_HCENTER
```

程序上电不会自动驱动舵机。首次仅接一只舵机，先执行 `SERVO_PING 1`、`SERVO_READ 1`，确认裸机后再执行 `SERVO_MOVE 1 1900 300 20`。

舵机分组：ID 1～3 为轮子（轮模式，速度范围 -1000～1000）；ID 4～6 为头部支撑（位置模式，位置 0～4095，速度 0～32767）。
示例：`SERVO_WHEEL 1 300`、`SERVO_WSTOP 1`、`SERVO_WALL 500`、`SERVO_WALL 0`。
头部示例：`SERVO_HEAD 4 2048 300 20`、`SERVO_HCENTER`。

`CH2/CH3` 仍需重点观察零值样本；出现非零数据不等于问题已经完全放行。

## 不包含的设备

D435 和 SX02 是 USB 相机，运行在 Jetson/Linux 侧，不能通过 ESP32 Arduino 草图直接采集。本地已有 `d435_capture`、`d435_detection` 和 Jetson 相机测试程序；本综合程序只负责 ESP32 侧传感器与显示设备。

## 注意

本工程是从原 `integration/full_system_epaper_test` 复制出的独立版本，原目录保持不变。上电时应分别确认 ESP32 的 3.3V 传感器支路和显示背光供电，不要将舵机12V接入 ESP32。
