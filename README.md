# C30D 跨垄小车固件（基于 WHEELTEC 官方源码修改）

基于轮趣 C30D V2.0 官方源码（WHEELTEC V5.0，2025.12.26，STM32F407VET6）修改的自主行走固件。
只替换了 `USER/main.c`，并新增 `USER/laser.c/.h`；FreeRTOS 调度器不启动（裸机主循环）。

## 1. 硬件与通道映射（实车标定）

| 物理轮子 | 电机通道 | 编码器 | 转向舵机（HTS-35H） |
|---|---|---|---|
| FL 左前 | MotorA | E1 / TIM2 | ID 4 |
| FR 右前 | MotorB | E2 / TIM3 | ID 3 |
| RL 左后 | MotorC | E3 / TIM4 | ID 2 |
| RR 右后 | MotorD | E4 / TIM5 | ID 1 |

- 电机极性（`motor_dir[4] = {-1,-1,+1,+1}`）：见 `CALIBRATION.md`；
- 舵机总线：BusLinker V3.0，USART2，115200，帧 `55 55 ID LEN CMD ... CS`，广播 ID = 0xFE；
- 舵机位置：0~1000 ↔ 0~240°，中位 500，90° = ±375（位置 875/125）。

## 2. 串口分配（当前固件）

| 链路 | 外设 | 引脚 | 波特率 | 用途 |
|---|---|---|---|---|
| 无线调试/指令 | **UART5** | PC12=TX / PD2=RX | 115200 | 串口指令（当前主要调试口） |
| 板载调试 | USART1 | PA9/PA10（Type-C 串口1，CH9102） | 115200 | 日志 + 指令 |
| 备用 | USART3 | PD8/PD9（Type-C 串口3） | 115200 | 日志 + 指令 |
| 总线舵机 | USART2 | PD5/PD6（蓝牙座） | 115200 | 4×HTS-35H |
| 板载 IMU | I2C | PB10/PB11 | — | ICM-20948 |
| 预留 | UART4 | PC10=TX / PC11=RX | 115200 | **计划给 3 路激光 F103**（原方案 UART5 已改作无线调试） |

## 3. 固件功能

- **四轮速度闭环**：50 Hz，PID（KP=10、KI=1/8、前馈 PWM≈2.5×速度+250、斜率限制 400/20ms、2 点滤波）；
- **IMU 航向闭环**：板载 ICM-20948，yaw 50 Hz 积分，左右轮差速修正（`YAW_KP=6`、`YAW_KD=1.5`、限幅 150），上电静止 0.6 s 做零偏标定；
- **70 cm 定距直行**：按键（PE0/USER）或指令 `G` 触发；编码器平均计程；15 s 超时；
- **手动调试指令**：见下表；
- **失联保护**：前进/后退时 5 s 无新指令自动停车。

### 串口指令（115200 8N1，回车/换行结束，大小写不敏感）

| 指令 | 作用 |
|---|---|
| `F` / `B` / `S` | 前进 / 后退 / 停车（速度闭环） |
| `V<n>` | 设置目标速度，n = counts/20ms（0~2000，如 `V900`） |
| `G` | 直行 70 cm 测试（按键等效） |
| `O<n>` | 开环固定 PWM 诊断（0~12000，如 `O2500`） |
| `T` / `T-` | 四舵机广播 +90° / −90° |
| `C` | 四舵机回中（0°） |
| `H` / `?` | 帮助 |

日志示例（每秒一行）：

```
SPD FWD tgt=900 1007/3278 926/3784 428/4335 998/3418 YAW=-2 C=0 SVO=0 P=5 CMD=G
```

## 4. 关键标定参数（`USER/main.c` 顶部）

| 宏 | 当前值 | 说明 |
|---|---|---|
| `WHEEL_DIAMETER_MM` | 76.4 | 名义 69 mm 实测 77/78/78 cm，标定后 70 cm 准确 |
| `ENCODER_CPR` | 60000 | 4 倍频 × 500 线 × 30 减速比 |
| `RUN_CM` / `RUN_SPD` | 70 / 900 | 70 cm 测试距离 / 速度 |
| `SPD_DEFAULT` / `SPD_MAX` | 150 / 2000 | 默认/最大目标速度 |
| `YAW_KP` / `YAW_KD` / `YAW_CORR_MAX` | 6.0 / 1.5 / 150 | IMU 航向环参数 |
| `SERVO_CENTER` | 500 | 舵机中位 |

## 5. 编译与烧录

工具链：Keil MDK（`D:\mdk5\UV4\UV4.exe`），工程 `USER/WHEELTEC.uvprojx`。

```powershell
$uv='D:\mdk5\UV4\UV4.exe'
$proj='C:\Users\iheat\Desktop\smart wheels2\C30D_MotorTest\USER\WHEELTEC.uvprojx'
& $uv -b $proj -o build.log     # 期望 0 Error / 0 Warning
& $uv -f $proj -o flash.log     # 期望 Verify OK. Application running
```

- 调试器：工程当前配置为 **CMSIS-DAP（WCH 无线 DAPLink，带 COM18）**；
  换回 ST-Link 需把 `WHEELTEC.uvoptx` 改回 `STLink\ST-LINKIII-KEIL_SWO.dll`（探针序列号 E）；
- 常见问题：`Target DLL has been cancelled` 多为探针被 Keil 调试会话占用或 SWD 接触问题，关掉调试会话/重插探针再试；
- 串口工具打开 COM18 后如无输出，注意 DTR/RTS 状态（错误的 DTR/RTS 会经下载电路把 MCU 按住复位）。

## 6. 上电与测试流程

1. 上电后**保持车静止约 0.6 s**（IMU 零偏标定）；
2. 无线/串口 115200 发 `H` 确认通信；
3. 架空前轮/放平地面，发 `G` 测试 70 cm 直行，观察串口日志与 OLED；
4. 需要连续运动时用 `F` + `S`；`O2500` 可做开环诊断。

## 7. 已知问题

1. **RL（左后，MotorC/E3）PWM 异常偏高**：同转速下 PWM 约为其他轮 2 倍（5458 vs 2500），疑似编码器接触不良或机械阻力，导致车右偏；待检查 6pin 与 C/D 对调定位。
2. **激光链路未打通**：F103 与 C30D 接线/协议未验证；激光接口计划从 UART5 改到 UART4。
3. 横向对中、换垄横移、比赛状态机尚未实现（见 `STAGES.md`、`PROGRESS.md`）。
4. `laser.c/.h` 目前保留在工程中但主程序未调用（历史阶段代码）。

## 8. Git / 版本与回滚

- 本地仓库；远程备份：`ssh://git@10.206.159.40/srv/git/c30d-motor-test.git`（ZeroTier，局域网 192.168.1.1 时自动切换）；
- 主要标签：`v0.1-baseline`、`v0.2-calib`、`v0.3-cmd-serial`、`v0.3.1-dir-ok`、`v0.4-speed-loop`、`v0.5-straight70`、`v0.6-imu-heading`、`v0.6.1-imu-tune`；
- 回滚：`git checkout <tag>` → 重新 `UV4 -b/-f`。

## 9. 目录结构

```
C30D_MotorTest/
├─ USER/                工程文件、main.c、laser.c/h、中断与启动文件
├─ BALANCE/             官方控制/系统初始化（systemInit 等）
├─ HARDWARE/            电机、编码器、OLED、I2C、IMU 等驱动
├─ FWLIB/               STM32F4 标准外设库
├─ OBJ/                 编译产物（.axf/.hex，gitignore）
├─ 方案_跨垄小车控制.md   总体方案
├─ STAGES.md            分阶段实施计划与验证方法
├─ PROGRESS.md          进度检查报告
├─ CALIBRATION.md       方向标定记录
└─ README.md            本文件
```
