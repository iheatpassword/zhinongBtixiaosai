# C30D 跨垄小车固件

基于轮趣 C30D V2.0 官方源码（WHEELTEC V5.0，STM32F407VET6）修改的**跨垄自主行走**固件。
裸机主循环（不启动 FreeRTOS 调度器），主要改动集中在 `USER/main.c`，并新增了
`laser`、`fsm2`、`safety` 等模块。

> 完整现状、接线、参数、已知问题与下一步 → 见 **[HANDOVER.md](HANDOVER.md)**。

## 功能概览

- 四轮速度闭环（50 Hz PID + 前馈 + 斜率限制），编码器计程；
- ICM-20948 航向闭环（差速 + 转向），yaw 50 Hz；
- 四路激光（前/左/右 + 垄侧 TOF，F103 经 UART4，v2 帧 + CRC8）；
- 贴边模式：左激光 100 mm + 差速 PID（900 速度已验证收敛）；
- 右/左蟹行（±90°）+ 垄侧 TOF 横向 PI；
- 倒车换垄（70 cm）/ 入库（100 cm）；
- FSM2 五垄蛇形状态机（已集成，尚未完整跑通）；
- 安全层：中值滤波、任意激光 <30 mm 停车、堵转检测、IWDG、HardFault dump。

## 按键（PE0 / USER）

| 操作 | 功能 |
|---|---|
| 单击 | 贴垄 / S1（edge-follow 测试；安全条件不满足时回 `SAFE: MOVE AWAY`） |
| 双击 | 启动 FSM2（完整流程；安全停车后条件消失可再次双击） |
| 三击 | 蟹行右测试（等同串口 `W`）；单轮标定模式下为退出 |
| 长按 | 单轮标定（WHEEL ID TEST，单击切换 MotorA→D，OLED 显示 E1–E4 峰值） |

> 三击停车已移除；需要停车请用串口 `Q`（全部中止）或 `S`（停止并清所有模式）。
> 蟹行左测试用串口 `W-`。

## 快速开始

```powershell
$uv='D:\mdk5\UV4\UV4.exe'
$proj='C:\Users\iheat\Desktop\smart wheels2\C30D_MotorTest\USER\WHEELTEC.uvprojx'
& $uv -b $proj -o build.log     # 期望 0 Error
& $uv -f $proj -o flash.log     # 期望 Flash Load finished
```

- 串口：**COM18**（CMSIS-DAP 虚拟串口），115200 8N1；
- 上电后保持静止约 0.6 s（IMU 零偏）；
- 日志 1 Hz：`F2=` 状态、`FA=` 故障、`LS=` 激光、`RG=` 垄侧、`YAW=`。

## 串口指令（摘要）

`A` 启动 FSM2、`E` 贴边、`Q` 中止、`K kp ki kd max [ff]` 在线调差速 PID、
`D+/D-` 差速标定、`P/M` 转向标定、`F/B/S` 前进/后退/停车、`T/T-/C` 舵机、
`H` 帮助。完整列表见 **[HANDOVER.md](HANDOVER.md)**。

> ⚠️ 会移动车的指令（`A/E/D/P/G/F/B/T`）必须经确认后再发；日常测试用按键触发。

## 硬件接线

| 物理轮 | 电机 | 编码器 | 前进时编码器符号 | 舵机 ID |
|---|---|---|---|---|
| FL 左前 | MotorA | E1 / TIM2 | − | 4 |
| FR 右前 | MotorB | E2 / TIM3 | + | 3 |
| RL 左后 | MotorC | E3 / TIM4 | + | 2 |
| RR 右后 | MotorD | E4 / TIM5 | − | 1 |

> 2026-11-23 单轮标定实测确认（`MOTOR A/B/C/D` + `E1..E4` 峰值）。表中编码器符号是
> **AB 极性翻转前**的测量（翻转后 A/B 前进符号预计变为 `+ / −`，需复测）；
> 速度环取绝对值，符号仅供带符号里程/航向扩展使用。

- 舵机总线：USART2（PD5/PD6），115200，广播 ID `0xFE`，位置 `500 ± 角度×4.1667`；
- 激光：F103 → UART4（PC10/PC11），v2 帧 `AA 55 | dL dR dF dB | flags | CRC8`；
- IMU：I2C PB10/PB11；按键 PE0；心跳 LED PE8。

## 已知问题

1. 前进约 3°/m 右偏（轮胎/阻力不对称）；900 速度差速 PID 可压住，**2600 仍偏**；
2. 左激光 >1.2 m 偶发 2000/丢帧；
3. FSM2 尚未全流程跑通（ENTRY→蟹行切换、蟹行横向保持需逐状态验证）；
4. 安全阈值 30 mm，廊道单侧间隙 40~60 mm 时可能触发停车。

详见 **[HANDOVER.md](HANDOVER.md)** 第 9 节。

## Git

- 远程：`ssh://git@10.206.159.40/srv/git/c30d-motor-test.git`（局域网 `192.168.1.1`）；
- 回滚：`git checkout <tag>` 后重新 `UV4 -b/-f`。

## 目录

```
C30D_MotorTest/
├─ USER/            工程、main.c、laser、fsm2、safety、中断
├─ BALANCE/         官方系统初始化/控制（未改动）
├─ HARDWARE/        电机、编码器、OLED、IMU 等驱动
├─ FWLIB/           STM32F4 标准外设库
├─ docs/            传感器协议（RIDGE_SENSOR.md）
├─ HANDOVER.md      交接文档
└─ README.md        本文件
```
