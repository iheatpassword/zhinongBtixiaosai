# Sprint-1 任务分派与验收计划（待审查）

> 状态：**待用户批准后执行**。Lead 不先启动子智能体，等确认。
> 依据：[PROGRESS.md](PROGRESS.md) 进度检查、[STAGES.md](STAGES.md) 阶段计划。
> 当前基线：`main @ 1808d3b`（固件 `6656d99` 已烧录验证，COM18 实机可通信）。

## 1. 目标（本 Sprint 验收线）

| # | 任务 | 验收标准 |
|---|---|---|
| T1 | 修复 RL，四轮 PWM 均衡 | 直行 2 m，四轮 PWM 偏差 ≤20%，yaw 漂移 ≤2°/m |
| T2 | 激光移到 UART4 并打通 | 收到有效帧；拔线 100 ms 内判失效 |
| T3 | S2 横向对中（e→δ、yaw→Δ） | 2–3 m 直行侧向偏差 ≤±3 cm |
| T4 | S3 换垄（90°+70 cm+激光对中） | 连续 5 次换垄误差 ≤±2 cm |
| T5 | S4 状态机全流程 | 空跑 ≤5 min、不碰围栏/垄、遍历 6 垄、停在启动区 |

## 2. 角色与分工

### Lead（我，唯一硬件操作者）
- 维护接口头文件（`USER/app_if.h`：激光/对中/状态机接口契约），审核子智能体代码；
- **独占**烧录（UV4 + WCH DAPLink）与 COM18 串口抓取，避免多 agent 抢硬件；
- 集成 `USER/main.c` 与 `USER/WHEELTEC.uvprojx`，每任务验收、打 tag、推送备份；
- 维护 `TASKS_SPRINT1.md`/`PROGRESS.md` 进度，向用户汇报。

### 子智能体（并行，代码 + 宿主测试，**不碰硬件**）

| Agent             | 任务               | 写作用域（互斥）                                        | 交付物                                     |
| ----------------- | ---------------- | ----------------------------------------------- | --------------------------------------- |
| **A `laser`**     | T2：UART4 激光驱动    | `USER/laser.c/.h`、`tools/laser_sim.py`          | 帧解析（AA55+CRC8）、100 ms 超时、PC 模拟器脚本       |
| **B `centering`** | T3：横向/航向对中控制     | `USER/centering.c/.h`、`tools/test_centering.py` | δ/Δ 解耦 PID、yaw 差速修正、宿主单测与调参说明           |
| **C `fsm`**       | T4/T5：换垄 + 比赛状态机 | `USER/fsm.c/.h`、`tools/test_fsm.py`             | 横移 70 cm 流程、S2→S5 状态机、宿主状态迁移单测          |
| **D `diag`**      | T1：四轮均衡诊断        | `tools/wheel_diag.py`、`docs/wheel_diag_*.md`    | 解析 COM18 日志算 PWM 偏差/偏航趋势，输出排查 Checklist |

> 写作用域互斥：**只有 Lead 改 `main.c`、`uvprojx`、`.uvoptx`**；子智能体代码由 Lead 集成，冲突时以 Lead 为准。
> 子智能体不得打开 COM18、不得调用 UV4/烧录；宿主测试用 Python 完成。

### 用户（硬件动作）
- T1：按 Checklist 重插 **MotorC 6pin**；仍异常则 **C/D 电机线对调**，把现象反馈 Lead；
- T2：把激光 F103 接到 **UART4**：F103 TX → C30D **PC11**，F103 RX ← C30D **PC10**，GND 共地，115200 8N1（若 F103 只能单向发送，RX 可不接）；
- 场地测试时配合布置围栏/垄/卷尺测量。

## 3. 接口契约（Lead 先定，子智能体按此实现）

```c
/* USER/app_if.h */
/* --- 激光（A 实现，Lead 在 main 中调用） --- */
typedef struct { u16 dL, dR, dF; u8 flags, valid; u32 last_ms; } LaserData;
extern volatile LaserData g_laser;
void Laser_Init(void);      /* UART4 + DMA 循环接收 */
void Laser_Poll(void);      /* 5–10 ms 调用，组帧/超时 */
u8   Laser_IsFresh(void);   /* 最近 100 ms 内有无有效帧 */

/* --- 对中（B 实现） --- */
void Centering_Init(void);
/* 输入：激光、IMU yaw(deg)、yaw rate(dps)、基础速度 counts/20ms
   输出：四轮目标速度 spd_tgt[4]（FL,FR,RL,RR） */
void Centering_Update(const LaserData *laser, float yaw_deg, float yaw_rate_dps,
                      s16 base_spd, s16 spd_tgt[4]);

/* --- 状态机（C 实现） --- */
typedef enum { ST_IDLE, ST_ENTRY, ST_RIDGE, ST_SHIFT, ST_EXIT, ST_PARK, ST_FAULT } CarState;
void FSM_Init(void);
void FSM_Tick(void);        /* 10–20 ms 调用，内部读传感器/驱动电机舵机 */
u8   FSM_GetState(void);
```

## 4. 顺序与依赖

```
D0  Lead：定接口 + 建 tag 基线（1808d3b） + 分配任务
 ├─ T1（D+用户）──┐（硬件，可并行）
 ├─ T2（A）───────┼─→ Lead 集成/烧录/验收 → tag `t2-laser-uart4`
 └─ T3 设计（B）──┘
       T2 通过 + T1 通过 → T3 集成验收 → tag `t3-centering`
                                   → T4（C）验收 → tag `t4-lane-shift`
                                                → T5（C/Lead）验收 → tag `t5-fsm`
```

## 5. 验收证据（Lead 逐项核）

| 任务 | 证据 |
|---|---|
| T1 | COM18 日志 2 m 直行四轮 PWM 偏差 ≤20%（`wheel_diag.py` 输出）+ yaw 漂移 ≤2°/m + 用户硬件结论 |
| T2 | PC 模拟帧（`laser_sim.py`）→ OLED/日志显示 L/R/F；F103 实接有效；拔线 100 ms 内 `LOST`/停车；直行/70 cm 无回归 |
| T3 | 场地/台架 2–3 m 侧向偏差 ≤±3 cm（卷尺 + 视频 + 日志 e/yaw） |
| T4 | 5 次换垄每次 ≤±2 cm（卷尺 + 日志计数），无碰撞 |
| T5 | 全流程视频 + 日志：≤5 min、6 垄遍历、不碰围栏/垄、停启动区 |

## 6. 风险与回滚

- **串口/烧录单点**：Lead 独占，子智能体只交付代码；需要实车数据时由 Lead 抓取后共享日志。
- **集成风险**：`main.c` 改动集中在任务验收前一次性合并；失败 `git revert`/回 tag。
- **接口变更**：任一 agent 需要改 `app_if.h` 必须提给 Lead，不允许自行改动（避免并行冲突）。
- **硬件不确定**：T1/T2 若硬件未通，T3 进度阻塞；此时 B/C 继续做宿主测试，不空转。
- **标签**：`t1-wheel`、`t2-laser-uart4`、`t3-centering`、`t4-lane-shift`、`t5-fsm`；每步推送（网络可用时）。

## 7. 时间预估

| 任务 | 预计 | 说明 |
|---|---|---|
| T1 | 0.5 d | 用户排查 + Lead 抓数 |
| T2 | 1 d | A 写驱动 + Lead 集成/PC 模拟/F103 实测 |
| T3 | 1–2 d | 含 PID 调参、2–3 m 场地试验 |
| T4 | 1 d | 换垄逻辑 + 重复精度测试 |
| T5 | 1 d | 状态机串联 + 全场空跑 |

合计约 **4–5 个工作日**（并行后，不含现场调试排队）。

## 8. 待用户确认

1. 是否同意以上分工（Lead 独占硬件/集成，4 个子智能体并行，用户负责 T1/T2 物理接线）？
2. 激光发送端协议：允许我们在 F103 侧用 `AA 55 dL dR dF flags CRC8` 帧（推荐），还是 F103 已有固定格式、由我们适配？
3. RL 排查由用户现场执行（重插/对调 C/D 线）是否可行？
4. 是否需要我把子智能体数量减到 2–3 个（先 T1/T2，再 T3–T5）？
