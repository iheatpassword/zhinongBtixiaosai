# 从 C30D_MotorTest 迁移到 C30D_CubeMX —— 评估、架构与计划

> 更新时间：2026-11-24（第二版：改为 FreeRTOS 架构）
> 源工程：`C30D_MotorTest`（Keil / ARMCC5 / 标准外设库 / 裸机 super-loop）
> 目标工程：`C30D_CubeMX`（CMake + Ninja + arm-none-eabi-gcc / STM32 HAL / FreeRTOS + CMSIS-OS2）
> 代码裁剪依据：`../../C30D_MotorTest/docs/CODE_AUDIT.md`

---

## 1. 旧 50 Hz 架构评估

### 1.1 `fsm2.c` 这个状态机本身：合格

先把它和"50 Hz"分开看——**状态机逻辑是这份代码里质量最高的部分**：

- 纯逻辑：一次 `Fsm2_Tick(输入快照 → 输出意图)`，不碰寄存器、不依赖 RTOS、无 `printf`、无动态内存；
- 入口动作 + settle 保持（`F2_SETTLE_MS`）分离，转态时先给舵机角度再等稳定；
- 时间比较全部用 `(u32)(now - t0)` 无符号差，天然抗 49 天回绕；
- 里程基线在保持期重同步，并能识别 `F2_DIST_JUMP_MAX_CM` 以上的跳变（编码器被清零/干扰）；
- 故障码锁存，只有 `Fsm2_Start()` / `Fsm2_Init()` 能清；
- 每状态独立超时表；
- 一次 tick 最多 3 遍 pass，允许"转态 → 立刻完成 → 再转态"。

这部分**建议原样迁移，不要重写**。

### 1.2 但这个"50 Hz 循环"的调度质量：不合格

问题不在频率，在于**这个循环不是周期循环，是"干完活睡 5 ms"**：

```c
while (1) {
    ... 20 ms 控制块 ...
    speed_tick();
    delay_ms(5);                 // ← 固定睡 5 ms，周期 = 工作耗时 + 5 ms
    Laser_Poll(); cmd_poll()×3; key_tick(); screen();
}
```

实测/静态统计出的阻塞量：

| 阻塞源 | 每次耗时 | 频率 | 依据 |
|---|---|---|---|
| 1 Hz 遥测行写 3 个串口 | **≈ 74 ms** | 1 次/秒 | `tools/serial_tuner/sample_log.txt` 实测平均 **282 字符/行** ÷ 11.52 字符/ms × **3 个口** |
| `App_SetSteer4()` 舵机帧 | ≈ 3.5 ms | 每 20 ms（转向时） | 4 × 10 字节 @115200 |
| 陀螺仪软 I2C | ≈ 0.6 ms | 每 20 ms（edge 模式） | `gyro_z_dps()` 被调用 **3 次**/tick，每次 2 个单字节事务，每事务 ≈100 µs |
| OLED 整屏刷新 | < 1 ms | 1 次/200 ms | 8 页 × 131 字节软 SPI |

**后果**：每秒有一次 ≈74 ms 的空档（3.7 个控制周期），随后 `next_ctrl += CTRL_MS` 让积压的
3 个 tick 在 ≈5 ms 内连续补跑：

```c
next_ctrl += CTRL_MS;
if ((s32)(now - next_ctrl) > 200) next_ctrl = now + CTRL_MS;   // 74 ms < 200 ms，不触发重同步
```

`yaw_deg += r * 0.02f` 和 PID 的 `integral[w] += e/KI_DIV` **都假设每个 tick 恰好 20 ms**。
补跑时 3 个 tick 间隔只有 ~5 ms，却各按 20 ms 积分——**平均速率没错，但采样时刻被压缩/缺失，
等于给航向积分和速度环积分注入了 ±60 ms 的周期性时间错位**。
按 0.52 m/s 算，60 ms ≈ 3 cm 的位置错配，这是航向环里一个持续的相位扰动源。

### 1.3 更关键：50 Hz 根本不是瓶颈

把延迟链算全（按 `EDGE_SPD=2600` counts/20ms ≈ **0.52 m/s**）：

| 环节 | 延迟 |
|---|---|
| 激光帧周期 | 0–20 ms |
| 3 帧中值滤波 | **60 ms**（组延迟） |
| 控制周期 | 0–20 ms |
| 舵机命令 `SERVO_FAST_MS` | **150 ms**（`servo_move(..., 150)`） |
| **合计** | **≈ 200–250 ms** → 车已走 **10–13 cm** |

而廊道单侧间隙只有 40–60 mm。**所以把控制周期从 20 ms 降到 10 ms 收益极小**——
真正的瓶颈是 150 ms 的舵机行程时间和 60 ms 的中值滤波。
这也解释了为什么 `EDGE_KP` 只能给到 0.10、差速 PID 只能给 0：

> 被控对象有 ≈150 ms 纯滞后，只能用极小增益，否则必振荡。
> 50 Hz 采样一个 150 ms 滞后的对象，是 3 倍过采样，不是欠采样。

**结论**：
- 50 Hz **对四轮速度环是合适的**（电机+轮子的机械时间常数远大于 20 ms）；
- 50 Hz **对转向/横向环是过采样**，提频率没用，要降滞后：
  1. 把 `SERVO_FAST_MS` 从 150 降到 40–60 ms（HTS-35H 实际能跟上），
  2. 中值滤波 3 帧改 2 帧（`edge_dl_read()` 已有 300 ms 丢帧保持，可以只留它），
  3. 横向环先做"角度前馈 + 低速积分"，别指望 PD 拉带宽。

### 1.4 其他结构性问题（迁移时顺手解决）

| 问题 | 现象 |
|---|---|
| 11 个并列模式标志 | `run_state` `rev_state` `shift_mode` `cent_mode` `fsm2_mode` `edge_mode` `cal_mode` `diff_cal_mode` `diff_test_mode` `wheel_test_mode` `open_loop`，全在一个 tick 里 `if` 互斥，顺序隐含（`edge_mode` 读到的是**上一拍**的 `g_sf`，`fsm2_mode` 读到的是**本拍**） |
| 安全层实际被编译掉了 | `SAFETY_ENABLE=0`、`F2_SAFETY_ENABLE=0` → `<30 mm 紧急停车` 与 `堵转` **不生效**。只有各模式自己的边界停车（`dF<=ENTRY_STOP_MM`、`dL/dR<=CRAB_STOP_MM`）还在。README/HANDOVER 写"✅ 安全层"与代码不符 |
| 两个航向控制器打架 | `speed_tick()` 里的 `YAW_KP/YAW_KD` 差速，和各模式自己算的差速；靠 `app_override` 切换，`centering.h` 专门写了注释警告 |
| 文档与代码参数不一致 | `EDGE_SPD` 代码 2600 / 文档 3200；`SHIFT_TEST_CM` 代码 280（HANDOVER 写 260） |
| 看门狗喂养过于宽松 | 循环顶部无条件喂狗，只证明"回到了循环顶部"，不证明控制块按时完成 |

---

## 2. 目标架构（FreeRTOS）

按**截止时间的紧迫程度**拆任务，让慢任务永远拉不长 20 ms 控制周期。

| 任务 | 周期 | 优先级 | 栈（字） | 职责 | 阻塞预算 |
|---|---|---|---|---|---|
| `ctrl` | **20 ms** | 30 | 384 | 采样（编码器/IMU/激光/电池）→ 模式机 → `Fsm2_Tick` → 速度环 PID → 写 PWM/舵机 | **必须 < 5 ms，禁止任何等待** |
| `laser` | 5 ms | 28 | 256 | UART4 DMA 排空 + CRC8 组帧 | 无 |
| `mon` | 100 ms | 26 | 192 | 任务存活检查 + **IWDG 刷新** | 无 |
| `cmd` | 10 ms | 20 | 320 | USART1/3/5 DMA 排空 + 指令解析 | 短 |
| `ui` | 200 ms | 12 | 384 | OLED 刷新 | < 1 ms |
| `log` | 1000 ms | 10 | 320 | 1 Hz 遥测（74 ms 阻塞就让它在这里慢慢写） | 允许 |

要点：

1. **`vTaskDelayUntil` 而不是 `vTaskDelay`** —— 绝对截止时间，晚一拍不会顺延，也不会积压补跑。
   这一条直接消灭 §1.2 的时间错位。
2. **阻塞 I/O 全部下沉**：74 ms 的遥测在 `log`（优先级 10），OLED 在 `ui`（12）。
   `ctrl` 和它们差 18–20 个优先级，慢任务根本抢不到 CPU。
3. **舵机帧留在 `ctrl` 里**（≈3.5 ms < 20 ms 预算），保证转向命令零额外延迟；
   若实测超预算，再抽独立 `servo` 任务 + 长度 1 的覆盖式队列。
4. **陀螺仪每 tick 只读一次**，`in.yaw_rate_dps` 复用；旧代码 edge 模式每 tick 读 3 次。
5. **静态分配**（`xTaskCreateStatic`）：`configSUPPORT_STATIC_ALLOCATION=1` 已开，
   不用堆、不碎片、栈大小编译期可见。
6. **看门狗归 `mon` 管**：`ctrl` 连续 3 个 100 ms 窗口没走 → 停止喂狗，等 1 s IWDG 复位。
   这比"循环顶部无条件喂狗"强得多。
7. **无锁发布输入快照**：`ctrl` 写另一块缓冲再原子翻索引（Cortex-M 单字节存储是原子的），
   读者拿到的永远是完整快照，控制路径里没有 mutex、没有临界区。

### 2.1 已修掉的一个真实缺陷

CubeMX 生成的 `main.c` 里，`HAL_IWDG_Refresh()` 写在 `osKernelStart()` **之后**的
`while(1)` 里——那段代码永远到不了。而 `MX_IWDG_Init()` 已经把 IWDG 启动了
（`PRESCALER_64`、`Reload=500`、LSI 32 kHz → **1 s 超时**）。

**所以原始 CubeMX 工程一上电就是 1 秒一复位的循环。**
现在由 `mon` 任务接管刷新，这个坑已经修掉。

---

## 3. 当前状态

| 项 | 状态 | 位置 |
|---|---|---|
| 目标目录骨架 | ✅ | `User/` |
| 接口契约（硬件无关） | ✅ | `User/app_if.h` |
| 五垄蛇形状态机 | ✅ 迁移并编译 | `User/fsm2.c/.h` |
| 安全层 | ✅ 迁移并编译 | `User/safety.c/.h` |
| 4WS 对中 | ✅ 迁移并编译 | `User/centering.c/.h` |
| **FreeRTOS 任务骨架** | ✅ 已实现、已链接、已从 `MX_FREERTOS_Init()` 启动 | `User/app/app_tasks.c/.h` |
| 共享输入快照 / 输出意图 | ✅ | `AppInputs` / `AppOutputs` |
| 板级驱动 | ⬜ 待写 | `User/board/` |
| 控制块主体 | ⬜ 待写 | `app_control_tick()` |
| 构建验证 | ✅ 通过 | FLASH 45,376 B / RAM 32,184 B |

`c30d_app`（三个纯逻辑模块）先以 OBJECT 库编译、**暂不链接**；
`c30d_rtos`（任务骨架）**已经链接并运行**——因为 `App_InitRtos()` 从
`Core/Src/freertos.c` 的 `USER CODE BEGIN RTOS_THREADS` 段被调用。

`app_tasks.c` 里那几个 `static` 桩函数就是迁移接口：

```c
board_sample_inputs()    // 编码器 + IMU + 激光 + 电池
board_apply_outputs()    // PWM + 舵机帧
app_control_tick()       // 模式机 + Fsm2_Tick + 速度环 PID
board_laser_poll() / board_cmd_poll() / board_ui_refresh() / board_log_dump()
```

---

## 4. 逐文件迁移映射

| 旧文件 | 去向 | 处理 |
|---|---|---|
| `USER/fsm2.c/.h` | `User/fsm2.c/.h` | ✅ 原样照搬 |
| `USER/safety.c/.h` | `User/safety.c/.h` | ✅ 原样照搬 |
| `USER/centering.c/.h` | `User/centering.c/.h` | ✅ 原样照搬 |
| `USER/app_if.h` | `User/app_if.h` | ✅ 去掉 `system.h`，改 `<stdint.h>` |
| `USER/main.c` 的 `main()` | `app_tasks.c` | ✅ 已被任务骨架取代 |
| `USER/main.c` 的 20 ms 块 | `app_control_tick()` | ⬜ **重写**（逻辑照抄，去掉模式标志汤） |
| `USER/main.c` 的 `speed_tick()` | `board/board_motor.c` + `app_control_tick()` | ⬜ PID 部分照抄 |
| `USER/laser.c` | `board/board_laser.c` | ⬜ DMA 改 HAL，解析照抄 |
| `HARDWARE/motor.c` | `board/board_motor.c` | ⬜ CCR 直写 → `__HAL_TIM_SET_COMPARE` |
| `HARDWARE/encoder.c` | `board/board_encoder.c` | ⬜ `HAL_TIM_Encoder_Start` / `__HAL_TIM_GET_COUNTER` |
| `HARDWARE/I2C.c` | `board/board_imu.c` | ✅ 软件位带 I2C 可原样保留（时序已验证） |
| `HARDWARE/oled.c` + `oledfont.h` | `board/board_oled.c` | ⚠️ **字体表含 GBK 字节，按二进制复制**，不要转码 |
| `HARDWARE/adc.c` | `board/board_adc.c` | ⬜ `HAL_ADC_Start/PollForConversion/GetValue` |
| `HARDWARE/usartx.c` | CubeMX `Core/Src/usart.c` | ✅ 已被覆盖，核对波特率/引脚 |
| `HARDWARE/timer.c` (TIM12) | — | ⚠️ 老 PWM 舵机通道；实车走 USART2，可只留空实现 |
| `HARDWARE/key.c` | `board/board_key.c` | ⬜ PE0 单击/双击/三击/长按 |
| `SYSTEM/delay/delay.c` | 不需要 | ✅ 用 `osDelay` / `HAL_GetTick`；软 I2C 若需 µs 延时用 DWT |
| FWLIB | 不需要 | ✅ 用 HAL |

---

## 5. StdPeriph → HAL 对照

### 5.1 电机 PWM

| 逻辑 | 旧 | 新 |
|---|---|---|
| MotorA 正/反 | `TIM10->CCR1` / `TIM11->CCR1` | `__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, v)` / `htim11` CH1 |
| MotorB 正/反 | `TIM9->CCR1` / `TIM9->CCR2` | `htim9` CH1 / CH2 |
| MotorC 正/反 | `TIM1->CCR2` / `TIM1->CCR1` | `htim1` CH2 / CH1 |
| MotorD 正/反 | `TIM1->CCR4` / `TIM1->CCR3` | `htim1` CH4 / CH3 |
| 满量程 | `16799` | `htimX.Init.Period` —— **必须核对** |

`Set_Pwm()` 语义保持不变：
```c
if (v < 0) { ccr_pos = 16799; ccr_neg = 16799 + v; }
else       { ccr_neg = 16799; ccr_pos = 16799 - v; }
```
> ⚠️ 若 CubeMX 生成的 `Period` 不是 16799，`PWM_LIM=12000`、`PWM_FF=250`、`PWM_SLEW=400`
> 全部要按比例重标。

### 5.2 编码器

`(short)TIMx->CNT; TIMx->CNT=0;` → `__HAL_TIM_GET_COUNTER(&htimx)` / `__HAL_TIM_SET_COUNTER(&htimx, 0)`。
`Read_Encoder(2 + w)` 的 2..5 约定保留。

### 5.3 串口 + DMA（必须与旧工程一致，否则收不到数）

| 口 | 引脚 | RX DMA |
|---|---|---|
| USART1 | PA9 / PA10 | DMA2 Stream2, Ch4 |
| USART3 | PD8 / PD9 | DMA1 Stream1, Ch4 |
| UART5 | PC12 / PD2 | DMA1 Stream0, Ch4 |
| UART4（激光） | PC10 / PC11 | DMA1 Stream2, Ch4 |
| USART2（舵机） | PD5 / PD6 | 发送不用 DMA |

DMA/UART 中断优先级 = 5，正好等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，
**可以**在其中调 `xQueueSendFromISR`。若用 IDLE 中断 + `HAL_UARTEx_ReceiveToIdle_DMA`，
回调里只做"投递"，解析放任务。

---

## 6. 必须照搬的 5 个坑

1. **ICM-20948 量程**：`GYRO_LSB_PER_DPS=65.5` 假设 **±500 dps**，芯片默认 ±250 dps。
   必须移植这 8 条写操作：
   ```
   0x7F=0x00 ; 0x06=0x01 ; delay 10ms ; 0x06=0x01
   0x7F=0x20 ; 0x00=0x07 ; 0x01=0x33 ; 0x11=0x07 ; 0x14=0x31
   0x7F=0x00 ; delay 100ms
   ```
   漏掉 → yaw 差 2 倍 → 所有航向控制失效。
2. **PE8 心跳灯**要显式配推挽输出（原 `V1_1_LED_Init()`）。
3. **舵机总线协议**（USART2 / 115200 / 广播 `0xFE`）：
   `55 55 id len cmd data… checksum`，`checksum = 255 - (id+len+cmd+Σdata)`，
   位置 `500 ± deg×4.1667`，**正角度 = 俯视顺时针（向右）**。
4. **OLED 字体表** `oledfont.h` 是 GBK 点阵索引，按二进制复制。
5. **里程换算常数**：`WHEEL_DIAMETER_MM=76.4`、`ENCODER_CPR=60000`（70 cm 实测标定）。
   这两个一改，所有距离阈值全失效。

---

## 7. 分阶段执行

### 阶段 0（已完成）
- [x] 源工程裁剪 + GCC 编译验证
- [x] 三个纯逻辑模块迁移
- [x] FreeRTOS 任务骨架 + 链接 + 从 `MX_FREERTOS_Init()` 启动
- [x] 修掉 IWDG 1 s 复位循环

**验收**：烧进去不再复位；调试器里能看到 6 个任务；`g_ctl_ticks` 每秒 +50。

### 阶段 1：板级驱动
- [ ] `board_motor.c` + `board_encoder.c` → 长按单轮标定复现
- [ ] `board_servo.c` → 串口 `T`/`C` 能让四舵机 ±90°/回中
- [ ] `board_imu.c`（含 §6.1 量程配置）→ 静止 yaw 平稳、手转方向对
- [ ] `board_adc.c` → `VB=` 正常
- [ ] `board_laser.c` → 1 Hz 日志 `LS=` `RG=` 正常
- [ ] `board_oled.c`、`board_key.c`

**验收**：各模块单独可测，`ctrl` 任务里还只调 `board_sample_inputs`。

### 阶段 2：控制块
- [ ] `app_control_tick()`：速度环 PID（`speed_tick` 原样搬）
- [ ] 模式管理器：11 个标志收敛成 `enum { MODE_IDLE, EDGE, FSM2, CRAB, CENT, CAL }` 单变量
- [ ] `Fsm2_Tick()` 接上（`User/fsm2.c` 已就绪）
- [ ] `board_cmd_poll()`：先做 `F/B/S/V/Q/H`

**验收**：`V900`+`F` 直行，四轮跟随；`S` 停。

### 阶段 3：重新打开安全层
- [ ] `SAFETY_ENABLE=1`、`F2_SAFETY_ENABLE=1`
- [ ] 安全判断放在 `board_apply_outputs()` **之前**，同一拍生效（旧代码差一拍）
- [ ] `mon` 任务里加"任一激光 <30 mm 持续"的独立兜底

**验收**：手挡激光 → 车停；堵转 → 车停。

### 阶段 4：降滞后（§1.3）
- [ ] `SERVO_FAST_MS` 150 → 40–60，重标 `EDGE_KP`
- [ ] 中值滤波改 2 帧 + 保持
- [ ] 重跑 900 / 2600 贴边，对比 `ET=` / `yaw`

---

## 8. 自检命令

```powershell
cd C30D_CubeMX_Route
cmake --preset Debug
cmake --build --preset Debug      # 期望 0 error
```
当前占用（2026-11-25，`cc91f5d`）：**FLASH 74,180 B / 512 KB（14.15%）、RAM 34,456 B / 128 KB（26.29%）**。

工具链来源、PATH 拼法、ST-Link 与 DAPLink(OpenOCD) 烧录命令、故障对照见
[BUILD_AND_FLASH.md](BUILD_AND_FLASH.md)。

建议在 `.ioc` 的 FreeRTOS 参数里打开 "Check for stack overflow"（`configCHECK_FOR_STACK_OVERFLOW 2`）；
`vApplicationStackOverflowHook` 已在 `app_tasks.c` 实现，打开即可生效。

---

## 9. 底层完整性核对（2026-11-24）

结论：**CubeMX 的"外设配置"是完整的，而且与旧工程逐脚、逐参数一致；
但"启动 + 数据搬运"的驱动代码一行都没有——那部分在 `User/board/`，尚未创建。**

### 9.1 已配置且核对一致

| 外设 | CubeMX 配置 | 旧工程 | 一致 |
|---|---|---|---|
| TIM1 CH1–4 | Period 16799, PSC 0, PE9/PE11/PE13/PE14 | `TIM1_PWM_Init(16799,0)` | ✅ |
| TIM9 CH1–2 | Period 16799, PE5/PE6 | `TIM9_PWM_Init(16799,0)` | ✅ |
| TIM10 CH1 | Period 16799, PB8 | `TIM10_PWM_Init(16799,0)` | ✅ |
| TIM11 CH1 | Period 16799, PB9 | `TIM11_PWM_Init(16799,0)` | ✅ |
| TIM12 CH1–2 | Period 9999, PSC 83, PB14/PB15 | `TIM12_SERVO_Init(9999,84-1)` | ✅ |
| TIM2 编码器 | TI12, Period 65535, PA15/PB3 | `Encoder_Init_TIM2` | ✅ |
| TIM3 编码器 | TI12, Period 65535, PB4/PB5 | `Encoder_Init_TIM3` | ✅ |
| TIM4 编码器 | TI12, Period 65535, PB6/PB7 | `Encoder_Init_TIM4` | ✅ |
| TIM5 编码器 | TI12, Period 65535, PA0/PA1 | `Encoder_Init_TIM5` | ✅ |
| TIM6 | 1 ms HAL 时基 | 旧工程自建 `ms_init()` | ✅ 等价 |
| USART1/2/3, UART4/5 | 全部 115200 8N1 | 同上 | ✅ |
| 4 路 RX DMA | 全部 CIRCULAR + `__HAL_LINKDMA` | UART4→DMA1_S2、UART5→DMA1_S0、USART1→DMA2_S2、USART3→DMA1_S1 | ✅ 完全一致 |
| ADC1 | 单次、CH9（PB1，电池） | `Adc_Init` + `Get_Adc(9)` | ✅ |
| IWDG | PSC 64, Reload 500 → 1 s | 同 | ✅ |
| GPIO | LED PE8 / IMU PB10·PB11 / OLED PD11–14 / BUZZER PA8 / ENABLE PD3 / KEY PE0 | 同 | ✅ |

OLED 四脚也逐一对上：`RST=PD12`、`DC=PD11`、`SCLK=PD14`、`SDIN=PD13`。

### 9.2 缺失（必须在 `User/board/` 补，CubeMX 不会生成）

| # | 缺什么 | 后果 |
|---|---|---|
| 1 | **`HAL_TIM_PWM_Start()` × 8 通道**（TIM1 4 + TIM9 2 + TIM10 1 + TIM11 1） | 不启动 → PWM 无输出，**车完全不动** |
| 2 | **`HAL_TIM_Encoder_Start()` × 4**（TIM2–5） | 计数器不跑 → 速度环读不到数 |
| 3 | **`HAL_ADC_Start()`** + 单次转换读取 | 电池电压读不到 |
| 4 | **`HAL_UART_Receive_DMA()` × 4** | CubeMX 只配置 DMA，不启动接收 → 激光/指令全收不到 |
| 5 | 软 I2C 位操作 | 旧 `HARDWARE/I2C.c` 可原样搬（时序已验证） |
| 6 | OLED 位操作 + 字体表 | 旧 `oled.c` / `oledfont.h` 原样搬，**字体表按二进制复制** |
| 7 | ICM-20948 量程配置 + 舵机帧打包 + 按键消抖 + 激光帧解析 + 速度环 PID + 模式机 + 日志 | 见 §4 映射表 |

启动顺序建议放在 `Core/Src/main.c` 的 `USER CODE BEGIN 2`（`osKernelInitialize()` 之前）：

```c
/* USER CODE BEGIN 2 */
Board_Init();      /* PWM/编码器/ADC/UART-DMA/IMU/OLED 全部启动 */
/* USER CODE END 2 */
```

`App_InitRtos()` 在 `MX_FREERTOS_Init()` 里，只负责建任务；**外设必须先于任务启动**，
否则第一个 `ctrl` tick 就可能读到没跑的编码器。

### 9.3 本车用不到、可以删的配置

| 项 | 原因 |
|---|---|
| TIM8 输入捕获（4 通道） | 航模接收机，旧工程已删除 |
| ADC2 CH8（PB0） | 旧工程里也没用，电池走 ADC1 CH9 |
| USB_OTG_FS + `hal_hcd` + `ll_usb` | 只为主机手柄 |
| CAN1 | 无对外通信 |
| TIM12（PB14/PB15） | 仅老 PWM 舵机；实车用 USART2 总线舵机 |
| BUZZER PA8 | 未使用 |

### 9.4 文件放在哪：三层，不要都塞 `User/`

| 层 | 目录 | 文件数 | 谁管理 | 能不能手改 |
|---|---|---|---|---|
| HAL + CMSIS | `Drivers/` | 93 | CubeMX | ❌ |
| FreeRTOS + CMSIS-OS2 | `Middlewares/` | 33 | CubeMX | ❌ |
| 外设初始化（CubeMX 生成） | `Core/` | 28 | CubeMX | 仅 `USER CODE` 段 |
| **业务逻辑** | `User/` | 9 | 我们 | ✅ |
| **板级驱动** | `User/board/` | **0（待建）** | 我们 | ✅ |

把 HAL 塞进 `User/` 是错的：CubeMX 重新生成时只认 `Drivers/`、`Middlewares/`、`Core/` 三处，
`User/` 之所以安全是因为它完全在 CubeMX 管辖之外。
---

## 10. 最终目录结构（2026-11-24 第三版：按用户要求重组）

`Core/Inc` 与 `Core/Src` **保持 CubeMX 原样**，只有 `Core/Src/freertos.c` 被改写；
业务代码全部在 `Core/App` 与 `Core/BSP`，构建只走根目录 `CMakeLists.txt`。

```
Core/
├─ Inc/                     ← CubeMX 原样，未新增
├─ Src/                     ← CubeMX 原样；仅 freertos.c 改写
│   └─ freertos.c             MX_FREERTOS_Init() = Board_Init() + App_InitRtos()
├─ App/                     ← 任务与逻辑（15 个文件）
│   ├─ app_config.h           全部可调参数
│   ├─ app_if.h               接口契约 + AppCmd/AppMode
│   ├─ app_tasks.c/.h         6 个原生 FreeRTOS 任务（静态分配）
│   ├─ app_control.c          模式机 + 速度环 PI（唯一写执行器的地方）
│   ├─ app_cmd.c/.h           3 路串口 DMA + 指令解析 + 按键映射
│   ├─ app_report.c/.h        1 Hz 遥测 + OLED
│   ├─ fsm2.c/.h              五垄状态机（原样迁移）
│   ├─ safety.c/.h            安全层（原样迁移，尚未接入控制路径）
│   └─ centering.c/.h         4WS 对中（原样迁移，尚未接入模式机）
└─ BSP/                     ← 板级驱动（22 个文件）
    ├─ bsp_types.h            统一整数别名
    ├─ board.c/.h             Board_Init() + DWT µs 延时 + 心跳灯
    ├─ i2c_soft.c/.h          PB10/PB11 位带 I2C
    ├─ motor/                 TIM1/9/10/11 PWM，Set_Pwm 语义
    ├─ encoder/               TIM2..5 编码器
    ├─ servo/                 USART2 总线舵机
    ├─ imu/                   ICM-20948（含 ±500 dps 量程配置）
    ├─ laser/                 UART4 DMA + CRC8 组帧
    ├─ oled/                  OLED + 字形表
    ├─ battery/               ADC1 CH9
    └─ key/                   PE0 单击/双击/三击/长按
```

命名注意：**没有** `adc.c/.h`、`usart.c/.h`、`tim.c/.h`、`gpio.c/.h`、`dma.c/.h`
这类名字，否则会和 `Core/Inc` 下 CubeMX 的同名头文件在 include 路径上冲突。
电池因此叫 `battery/`，不是 `adc/`。

### 10.1 已接入控制路径（真跑）

| 模块 | 谁调用 | 状态 |
|---|---|---|
| Motor / Encoder / Servo | `app_control.c` | ✅ 每 20 ms |
| IMU（含量程、零偏、实测 dt 积分） | `app_tasks.c::sample_inputs` | ✅ 每 20 ms |
| Laser（DMA + CRC8） | `app_tasks.c` 的 laser 任务 | ✅ 每 5 ms |
| Battery / Key / OLED | 各自任务 | ✅ |
| FSM2 状态机 | `app_control.c::AppControl_Tick` | ✅ 模式 FSM2 |
| 速度环 PI | `app_control.c::speed_step` | ✅ |

### 10.2 已迁移但**尚未接入**（下一步）

| 模块 | 现状 |
|---|---|
| ~~`safety.c`~~ | ✅ 2026-11-24 已接入：`safety_eval()` 位于 `speed_step()` 之后、输出之前，同一拍生效；`SF=` 字段可在日志里看 |
| `centering.c` | 编译进镜像但 `Centering_Update()` 没有调用者 |

这两处是「阶段 3」和「阶段 2」的剩余项，不是遗漏。另外默认的 `defaultTask`
已经不存在了（`freertos.c` 不再用 `osThreadNew`），`main.c` 里的
`osKernelInitialize()/osKernelStart()` 原样保留 —— 那也是原生调度器的入口。
---

## 11. 上电时序陷阱：IWDG 早于 bring-up 启动

**现象**：烧进去后 OLED 不亮、心跳灯不闪、GDB 停在 `BSP_DelayUs()` 且反复 `external reset`。

**根因**：`main.c` 的调用顺序是

```
MX_IWDG_Init();        // L118  看门狗在这一刻就开始计时
...
MX_FREERTOS_Init();    // L125  内部调用 Board_Init()
osKernelStart();       // L128
```

而 `Board_Init()` 是阻塞的：

| 阶段 | 耗时 |
|---|---|
| 舵机回中 3 × (帧 + 50 ms) | ~150 ms |
| `OLED_Init()` | ~100 ms |
| `IMU_Init()`：10 + 100 + 200 × 3 ms | ~740 ms |
| **合计** | **≈990 ms** |

IWDG = `PSC 64 / Reload 500`，超时 = 500×64/LSI。LSI 标称 32 kHz → 1000 ms，
但 LSI 实际可在 17–47 kHz，**最快只有约 680 ms**。于是每次都在 IMU 零偏采样循环里被复位，
永远到不了 `osKernelStart()`。

旧固件没这个问题，因为它的 `iwdg_init()` 写在 `imu_init()` **之后**。

**为什么不能改 `main.c`**：`Core/Src` 里只允许改 `freertos.c`，所以只能让 bring-up 自己喂狗。

**修法**：

1. 新增 `Board_DogFeed()`，在 `Board_Init()` 每个阶段边界调用，`IMU_Init()` 的零偏循环里每 32 个样本喂一次；
2. `BSP_DelayUs()` 加了自旋上限——万一 `DWT->CYCCNT` 没跑起来也只会超时返回，不会死等。

> ⚠️ 这条对以后也有用：只要 CubeMX 重新生成 `main.c`，`MX_IWDG_Init()` 就会回到
> `MX_FREERTOS_Init()` 之前。任何新增的阻塞式 bring-up 都必须自己喂狗。