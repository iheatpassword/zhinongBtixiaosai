# 单轮映射测试（WHEEL ID TEST）审查报告

> 审查时间：2026-11-23
> 审查对象：`C30D_MotorTest`（Git HEAD `830a9fa`，`_build_wheelid.log` 16:04 构建 0 error，`_flash_wheelid_1.log` 16:05 烧录成功）
> 结论一句话：**电机/编码器通道配对可信；四角对应在 2026-11-23 实测为 LF=A / RF=B / LB=C / RB=D（与 README 表一致，但与官方固件“A/B 左”约定不同）。审查时 WHEEL ID TEST 是“死”的（已修复），另外 D+/D- 与 EDGE 的差速轴用错了轮对（已修复）。**

> **修复记录（2026-11-23 16:26）**：§4 的修复已落地到 `USER/main.c` 并编译通过（0 error，仅 2 个原有 warning）。
> 实际版本比 §4 草案多做了两件事：① 每个编码器通道做 1.5 s 内的**带符号峰值保持**，轮子停转后仍能读数；
> ② OLED 改为显示 `MOTOR A..D` + `E1..E4` 四路峰值，直接看哪一路编码器在动。
> 另外 `abort_all()` 现在统一清 `wheel_test_mode`，串口启动别的模式也会自动退出单轮测试。
> 文中行号是修复前版本的，供追溯；最新行为以 `USER/main.c` 为准。

> **2026-11-23 16:45 追加**：实车单轮测试发现 **AB/前轮反转** → `motor_dir` 改为 `{+1,+1,+1,+1}`；
> 同时把 `D+/D-`、EDGE、FSM2 ENTRY 的速度差速从 `[0,1] vs [2,3]`（前/后）改为 `[0,2] vs [1,3]`（左/右）。

---

## 0. 已读文档与关键信息

| 文档 | 与本审查相关的结论 |
|---|---|
| `README.md` | 第 52–61 行给出“FL=A / FR=B / RL=C / RR=D、E1/TIM2…E4/TIM5、舵机 4/3/2/1”的接线表；第 27 行说长按进入单轮标定 |
| `HANDOVER.md` | 第 36–46 行重复同一张表；第 79–82 行给出单轮标定的预期行为（单击切轮、每轮正转 1.5 s、三击退出、OLED 显示轮名/电机号/编码器速度）；第 206 行把“用单轮标定确认映射”列为下一步 |
| `docs/RIDGE_SENSOR.md` | 垄侧 TOF `dB` 协议，与轮子映射无关 |
| `BALANCE/README_heading_control.md` | 官方 FreeRTOS 航向控制；本工程裸机主循环不启动调度器，该模块不生效 |
| `更新记录.txt` | 官方 V5.0 变更记录（V550 四驱车型等），不是本项目改动 |
| `../../C30D_底层主控板_引脚与接口调研.md`、`../../硬件/C30D_引脚说明.md` | 官方资源分配表：TIM2/3/4/5 = 电机 A/B/C/D 编码器；**第 29 行明确“四驱/麦轮 A、B 归左侧，C、D 归右侧”** |

---

## 1. 文档表 vs 代码实现

| 逻辑轮 | 文档电机 | 代码里的电机索引 | 代码里的编码器 | 舵机 ID |
|---|---|---|---|---|
| FL | MotorA | 0 | TIM2 / E1 | 4 |
| FR | MotorB | 1 | TIM3 / E2 | 3 |
| RL | MotorC | 2 | TIM4 / E3 | 2 |
| RR | MotorD | 3 | TIM5 / E4 | 1 |

代码位置：

- `USER/main.c:124`：`WHEEL_MOTOR[4] = {0,1,2,3}`（恒等映射，等于没做映射）
- `USER/main.c:125`：`WHEEL_SERVO_ID[4] = {4,3,2,1}`
- `USER/main.c:128`：`motor_dir[4] = {-1,-1,+1,+1}`
- 编码器通道在 4 处被硬编码为 `Read_Encoder(2 + w)`：`main.c:429`、`471`、`488`、`525`

**可以确认的部分**：A↔TIM2、B↔TIM3、C↔TIM4、D↔TIM5 与官方资源分配表、`HARDWARE/encoder.c` 完全一致。`BALANCE/system.c:227-238` 的中文注释（“编码器 A 读电机 C”）是官方注释写错，`balance.c:1056-1059` 才是真实映射，这一点 `../../硬件/C30D_引脚说明.md` 第 70 行已经勘误过。

**不能确认的部分**：`FL=A / FR=B / RL=C / RR=D` 这个“四角对应”在代码里只是注释和文档，没有任何测量依据。相反，官方固件把 **A、B 当左侧、C、D 当右侧**：

- `BALANCE/balance.c:245-248`（FourWheel 逆解：A/B 用 `Vx - Vz*...`，C/D 用 `Vx + Vz*...`）
- `BALANCE/balance.c:401-402`（`Set_Pwm(A, -B, -C, D)`，B、C 取反）
- `HARDWARE/usartx.c:61-63`（`Z_speed` ∝ `-A-B+C+D`，即右轮减左轮）
- `../../硬件/C30D_引脚说明.md:29`（明确写“A、B 归左侧，C、D 归右侧”）

所以按官方约定，`FR=MotorB`、`RL=MotorC` 这两行是**左右颠倒**的：A、B 里一个是左前、一个是左后，C、D 里一个是右前、一个是右后，但**官方资料没有写哪个前哪个后**（`Set_Pwm` 对 B、C 取反，正好对应底盘 180° 旋转对称的电机安装，这只能解释“B、C 是斜对角”，不能定前后），必须实测。

另外，本工程 `motor_dir = {-1,-1,+1,+1}`，而官方四驱的“前进”极性是 `Set_Pwm(+A,-B,-C,+D)`，两者相差 A、C 两路取反。这说明这台车的电机接线/安装方向**不是原厂四驱状态**（自定义四舵轮底盘，接线顺序由装配决定），因此更不能拿官方极性直接套用——这恰恰是单轮测试存在的意义。

> **2026-11-23 实测更新**：单轮标定结果为 **LF=A、RF=B、LB=C、RB=D**，E1..E4 前进符号 **− + + −**。
> 所以 README 表是对的；官方“A/B 左、C/D 右”不适用于本车接线（本车 A/B 是前轴、C/D 是后轴）。
> 按此映射，`diff_cal`（D+/D-）和 EDGE 速度差速原来的 `[0,1] vs [2,3]` 实际是前/后，已改为 `[0,2] vs [1,3]`（左/右）。

> 顺带：官方自检 `HARDWARE/check.c:37-52` 也只做“四轮同时正转 3 s / 反转 5 s”（`Set_Pwm(Full,-Full,-Full,Full,0)`），没有单轮模式，所以没有现成参考可抄；它同样印证了原厂四驱的极性模板是 `(A,−B,−C,D)`。

---

## 2. 测试程序逐项检查

### 2.1 预期行为（`HANDOVER.md:81-82`）

> 长按进入；单击 = 切到下一轮（FL→FR→RL→RR），每轮正转 1.5 s；长按进入时先转 FL；三击退出。OLED 显示轮名、电机号、编码器速度。

### 2.2 检查结果（修复前）

| # | 检查项 | 结果 | 证据 |
|---|---|---|---|
| D1 | 进入后所选电机真的会转 | ❌ **不会转**（致命） | 见 2.3 |
| D2 | 每轮正转 1.5 s 后自动停 | ❌ **不会停**（致命） | 见 2.4 |
| D3 | 三击退出 | ❌ **退不出**（致命） | 见 2.5 |
| D4 | 能看出转向/编码器极性 | ❌ 编码器取了绝对值，永远显示 `+` | `main.c:472-474`、`658` |
| D5 | 能看出“到底是哪一路编码器在动” | ❌ 只显示假定匹配的那一路 | `main.c:658` |
| D6 | 进入时舵机回中 | ❌ 没做；从蟹行模式进入时四个轮子是横着的 | `main.c:1915-1923` |
| D7 | 正/反两个方向都试 | ❌ 只有固定 `+2500` 开环 | `main.c:475` |
| D8 | 覆盖舵机 ID（表里也有这一列） | ❌ 完全没有舵机项 | 全文 |
| D9 | 显示电机通道是 A/B/C/D | ⚠️ 显示 0/1/2/3，容易被当成序号 | `main.c:656` |
| D10 | 退出后能恢复正式流程 | ❌ 单次点击永远被“切轮”吃掉 | `main.c:1894-1905` |

### 2.3 D1：进入后电机根本不会转（最严重）

`ev == 4`（长按）分支先调用 `abort_all()`：

```c
/* main.c:1915-1923 */
else if (ev == 4u)
{
  abort_all();              /* ← 这里把 motor_run 清 0（main.c:1064） */
  wheel_test_mode = 1u;
  wheel_idx = 0u;
  wheel_t_on = ms_now();
  tx_puts(UART5, "WHEEL ID START\r\n");
  diff_test_mode = 0u;
}
```

之后 `speed_tick()` 的单轮分支虽然把 `pwm_out` 设成 2500，但最终输出要经过 `speed_apply()`，而它是按 `motor_run` 门控的：

```c
/* main.c:444-450 */
for (w = 0u; w < 4u; w++)
{
  if (motor_run == 0) { m[WHEEL_MOTOR[w]] = 0; continue; }   /* ← 全部清 0 */
  p = pwm_out[w];
  p = (motor_run > 0) ? p : -p;
  m[WHEEL_MOTOR[w]] = p * (int)motor_dir[w];
}
Set_Pwm(m[0], m[1], m[2], m[3], 1500);
```

单轮分支自己也不设置 `motor_run`（`main.c:464-478`），在本测试流程内没有任何地方替它设置。旧 FSM 的 `FSM_Tick()` 虽然每 20 ms 还在跑（`main.c:1876`），但 `FSM_Start()` 从未被调用，FSM 始终 IDLE，不会调用 `App_SetWheelSpeed()`。

**结论：`Set_Pwm(0,0,0,0,1500)` 每 20 ms 发一次，四个轮子都不会动。OLED 上的 `SPD` 只有在用手转轮子时才会有数（编码器读取本身是好的），这很容易被误认为“测试在工作”。**

### 2.4 D2：1.5 s 停转被下一句 `speed_tick()` 立刻覆盖

停转代码在 `main.c:1652-1657`，它位于 20 ms 控制块内部；而 `speed_tick()` 在 `main.c:1878`，在同一个 20 ms 块里、位于该内部块**之后**（花括号深度已核对：1652 行 depth=4，1877 行闭合内部块，1878 行 `speed_tick()` 回到 depth=3）。

于是即使补上 `motor_run = 1`，1.5 s 之后每个控制周期的实际顺序是：

1. `main.c:1652` 块把 `pwm_out` 清 0 并 `speed_apply()` → 输出 0；
2. 紧接着 `speed_tick()`（`main.c:464` 分支）又把所选轮 `pwm_out` 设回 2500 并 `speed_apply()` → 输出 2500。

**净结果是电机一直转，直到单击切到下一轮或断电**，“每轮 1.5 s”不成立。

### 2.5 D3：进入后无法退出

全文没有任何 `wheel_test_mode = 0u;`（只有 `= 1u` 一处赋值）。三击（`ev == 3`）只调用 `abort_all()`，而 `abort_all()`（`main.c:1062-1076`）并不清 `wheel_test_mode`：

```c
/* main.c:1910-1914 */
else if (ev == 3u)
{
  abort_all();                  /* 没有 wheel_test_mode = 0 */
  tx_puts(UART5, "ABORT\r\n");
}
```

于是：

- OLED 写着 `TRIPLE=EXIT`，实际三击无效；
- `screen()` 因为 `wheel_test_mode != 0` 永远走测试界面（`main.c:650`）；
- 单次点击永远被“切到下一轮”消费（`main.c:1896-1900`），再也无法启动 FSM2；
- **只能复位/断电才能恢复**。

### 2.6 D4–D10 说明

- **D4**：单轮分支 `if (wr < 0) wr = -wr;`（`main.c:472`），`show_signed()` 于是永远打印 `+`。编码器接反、A/B 相序反、电机接反都看不出来。
- **D5**：OLED 只显示 `meas_spd[wheel_idx]`，也就是“假定匹配”的那一路。若 MotorB 实际接的是 TIM4，屏幕上不会有任何提示（显示 0），你无法知道是哪一路在动。1 Hz 串口日志虽然打印四路，但 1.5 s 窗口里只有 1~2 个样本，而且同样取了绝对值。
- **D6**：`abort_all()` 不回合舵机。若刚跑过蟹行（±90°）再长按进入，轮子是横着的，此时判断“正转是否为前进方向”没有意义。`diff_test` 进入时是回中的（`main.c:1933`），单轮这里漏了。
- **D7**：固定 2500 开环，只往一个方向转。不能验证反向、不能验证堵转、也看不出 `motor_dir` 与真实转向的关系（`pwm_out` 已经乘过 `motor_dir`，等于把待验证的量预先用掉了）。
- **D9**：`OLED_ShowNumber(56,12,WHEEL_MOTOR[wheel_idx],2,12)` 显示 `00~03`，而文档讲的是 A/B/C/D。
- **D10**：退出后 `motor_run`、`app_override` 等由 `abort_all()` 处理，但模式标志没清，所以永远回不到正常界面。

---

## 3. 代码自身的“轮对分组”不一致，而且有两种差速轴

`app_if.h:61,68,72` 明确约定 `App_SetWheelSpeed()` / `App_SetSteer4()` 的入参顺序是 `FL, FR, RL, RR`。按这个顺序：

- “前 / 后”分组 = `[0,1]` vs `[2,3]`
- “左 / 右”分组 = `[0,2]` vs `[1,3]`

但 `main.c` 里两种分组同时存在：

| 功能 | 用的轮对 | 代码/文档宣称的物理含义 |
|---|---|---|
| `cal_mode`（P/M 转向标定，`main.c:1693`） | `[0,1]` vs `[2,3]` | 前 +8° / 后 −8°（`HANDOVER.md:98`） |
| `App_SetSteerRotate()`（`main.c:1274-1282`） | `[0,1]` vs `[2,3]` | 前 + / 后 −（`app_if.h:71`） |
| `diff_cal`（D+/D- 差速标定，`main.c:1675-1676`） | `[0,1]=700` vs `[2,3]=900` | 左慢右快（`HANDOVER.md:96`） |
| EDGE 速度差速（`main.c:1750-1753`） | `[0,1]=+corr` vs `[2,3]=-corr` | 差速航向（未写明轴向） |
| `diff_test`（`main.c:496-499`） | `[0,2]=+` vs `[1,3]=−` | 左 + / 右 −（OLED `CMD L+ R−`） |
| `speed_tick` 航向差速（`main.c:517-520`） | `[0,2]=-corr` vs `[1,3]=+corr` | 左 / 右差速 |

同一组下标在 P/M 里被当作“前/后”，在 D+/D- 里被当作“左/右”——**这两处不可能同时成立**；EDGE 的速度修正又按 `[0,1]` vs `[2,3]` 分组，与 D+/D- 同轴、与 `speed_tick` 正交。换句话说，写 D+/D- 和 EDGE 时作者把 `[0,1]` 当成左侧、`[2,3]` 当成右侧；写 P/M 和 `App_SetSteerRotate()` 时又把 `[0,1]` 当成前轮、`[2,3]` 当成后轮。这正是“README 表可能把 B/C 的左右写反”的直接旁证。如果文档表为真（`FL=A, FR=B, RL=C, RR=D`），则 `[0,1]` 是前轮，D+/D- 的“左慢右快”和 EDGE 实际做的都是前后 toe；如果按官方约定（A,B 左 / C,D 右），则 `[0,1]` 是左侧，P/M 的“前 +8 后 −8”以及 `App_SetSteerRotate()` 的前后转向就把左右舵机当成前后舵机了。**这不是文档笔误级别的问题，已经进入控制律；单轮映射实测之前，无法判断哪一组用法是对的。**

在此之上，两处“差速航向”还用了**互相正交**的轮对：

- `speed_tick()`（`main.c:517-520`，增益 `YAW_KP/YAW_KD`，`main.c:106-107`）：
  `spd_tgt[0]=base-corr, [1]=base+corr, [2]=base-corr, [3]=base+corr`
  → 按文档 FL,FR,RL,RR 的顺序，这是**左/右**差速。
- EDGE / ENTRY 贴边（`main.c:1750-1754`、`1842-1845`，增益 `diff_kp/ki/kd`，可串口 `K` 调）：
  `esp[0]=esp[1]=EDGE_SPD+corr; esp[2]=esp[3]=EDGE_SPD-corr`
  → 按同一顺序，这是**前/后**差速（toe）。

两者**不是同一根轴，也不是同一组增益**，但文档里都叫“差速 PID / 差速航向 PID”。这本身不一定是 bug（贴边时四轮平行，前后 toe 确实能产生 yaw），但有两件事必须注意：

1. 在“文档表为真”和“官方约定为真”两种情况下，这两处的物理含义**互换**：
   - 若文档表为真：EDGE 的 `corr` 实际在做前/后 toe；
   - 若官方约定为真（A,B 左 / C,D 右）：`speed_tick()` 的 `corr` 实际在做前/后 toe。
2. 因此“900 速度差速 PID 能压住右偏、2600 压不住”这个标定结论，只对当时用的那根轴、那组增益有效；在单轮映射实测前，不能直接套到另一种模式，也不能据此断言硬件不对称。等第 5 节的表填完，再决定这两处是否统一到同一根轴，以及 `diff_kp/ki/kd` 与 `YAW_KP/YAW_KD` 是否需要合并。

另外两处文档冲突，建议一并修正：

- `USER/centering.h:24,93` 写“正角度 = 左转（CCW）”，`HANDOVER.md` 第 46 行 / 第 168 行写“正角度 = 俯视顺时针（向右）”。`centering.c` 是历史模块，但文档读者会被误导。
- `README.md:65` 的“3°/m 右偏”与 `motor_dir`、`WHEEL_MOTOR` 的关系没有记录；在映射没实测前，不应把右偏只归因于“轮胎/阻力不对称”。

---

## 4. 修复方案（已应用）

### 4.1 让单轮模式真的能转、能停（替换 `main.c:464-479`）

```c
  if (wheel_test_mode != 0u)
  {
    u8  ww;
    int wr;
    u8  on = (u8)(((ms_now() - wheel_t_on) < 1500u) ? 1u : 0u);

    for (ww = 0u; ww < 4u; ww++)
    {
      wr = Read_Encoder((u8)(2u + ww));      /* 保留符号，不再取绝对值 */
      if (wr >  32000) wr =  32000;
      if (wr < -32000) wr = -32000;
      meas_spd[ww] = (s16)wr;
      pwm_out[ww]  = ((on != 0u) && (ww == wheel_idx)) ? (s16)2500 : (s16)0;
    }
    motor_run = (on != 0u) ? 1 : 0;          /* 关键：让 speed_apply 放行 */
    speed_apply();
    return;
  }
```

同时**删除** `main.c:1652-1657` 的停转块（停转已经由 `on` 统一管理）。

> 取舍说明：这个写法在测试期间每个 tick 把 `motor_run` 置 1，因此低电压停车（`main.c:1948-1954`）会被下一 tick 覆盖；架空单轮测试可以接受，落地前先确认 `VB>10.5 V`。若想保留低电压保护，可改为在 `speed_apply()` 里对 `wheel_test_mode` 放行（`if ((motor_run == 0) && (wheel_test_mode == 0u)) ...`），让 `motor_run` 始终为 0。

### 4.2 进入时回中舵机

```c
/* main.c:1915-1923 */
else if (ev == 4u)
{
  abort_all();
  App_SetServoCenter();        /* 轮子摆正，“正转=前进”才有意义 */
  wheel_test_mode = 1u;
  ...
}
```

### 4.3 三击真正退出

```c
/* main.c:1910-1914 */
else if (ev == 3u)
{
  wheel_test_mode = 0u;        /* 先退出单轮模式 */
  abort_all();
  tx_puts(UART5, "ABORT\r\n");
}
```

### 4.4 OLED 同时显示四路（替换 `main.c:650-662`）

```c
  if (wheel_test_mode != 0u)
  {
    static const u8 *const wn[4] = { (const u8 *)"FL", (const u8 *)"FR",
                                     (const u8 *)"RL", (const u8 *)"RR" };
    u8 k;
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"WHEEL ID TEST");
    for (k = 0u; k < 4u; k++)
    {
      u8 yy = (u8)(12u + 12u * k);
      OLED_ShowString(0, yy, (u8 *)((k == wheel_idx) ? ">" : " "));
      OLED_ShowString(8, yy, wn[k]);
      show_signed(32, yy, (int32_t)meas_spd[k], 5);
    }
    OLED_ShowString(0, 52, (u8 *)"TRIPLE=EXIT");
    OLED_Refresh_Gram();
    return;
  }
```

这样即使 B 接到了 TIM4，`>B` 旁边没数、`RL` 旁边有数，也能立刻看出来。

### 4.5 建议的下一版（自动判定，替代人眼读表）

1. 架空车、舵机回中、使能开关 ON；
2. 依次给 MotorA/B/C/D 发 `+2500`（**不乘 `motor_dir`**）各 1 s，读四路编码器原始计数；
3. 取 `|count|` 最大的通道作为该电机的编码器，符号作为编码器极性；
4. 串口打印 `A -> TIM2 enc, PWM+ = car forward` 这样的四条结论；
5. 断电、落地后，再给每个舵机 ID 单独 `+20°`，人工确认“ID n = 哪个角、正角是否向右”，补全表里的舵机列。

---

## 5. 建议的实车验收流程（修复后）

前置：车架空、四轮离地、舵机 `C` 回中、电机使能开关 ON、电池 > 10.5 V、COM18 已打开。

1. 长按 1.2 s 进入 → OLED 显示 `MOTOR A`、`E1..E4` 四行带符号峰值；
2. 观察 FL 轮：转不转、转向是否前进方向；1.5 s 后看哪一路 `E` 的峰值非零、符号正负；
3. 单击 → `MOTOR B`，重复；再 `MOTOR C`、`MOTOR D`；
4. 三击退出，确认回到正常界面且单击能启动 FSM2；
5. 把结果填进下表，与 `README.md:52-61` 对照：

| 物理轮位 | 被点动的 Motor | 是否只该轮转 | 转向（前进/后退） | 计数符号 | 实际编码器 TIM | 与文档一致？ |
|---|---|---|---|---|---|---|
| FL | A | | | | | |
| FR | B | | | | | |
| RL | C | | | | | |
| RR | D | | | | | |

验收标准：

- 每个 MotorX **只**让一个物理轮转（其余三轮静止）；
- 该轮对应的编码器有数，另外三路接近 0（若反过来，说明编码器插错通道）；
- 四角与文档一致则只更新“已实测”标记；不一致则改 `WHEEL_MOTOR`、新增 `WHEEL_ENC[4]`、同步改 `WHEEL_SERVO_ID`，并更新 `README.md`/`HANDOVER.md` 两张表；
- 转向与 `motor_dir` 不符的通道，改 `motor_dir` 并重跑一次确认。

> 注意：`WHEEL_MOTOR` 目前是恒等数组、编码器通道又在 4 处硬编码 `2 + w`。如果实测发现顺序不是 A,B,C,D，**不能只改 `WHEEL_MOTOR`**，必须同时引入 `WHEEL_ENC[4]` 并在 `main.c:429/471/488/525` 统一使用，否则电机和编码器会各按各的顺序走。

---

## 6. 结论

1. **通道级映射（A↔TIM2 … D↔TIM5）可信**，与官方资源表、`encoder.c` 一致。
2. **四角级映射已实测确认**（2026-11-23：LF=A、RF=B、LB=C、RB=D；E1..E4 符号 − + + −），与 README 表一致；官方“A/B 左”约定不适用于本车。
3. **负责实测的 WHEEL ID TEST 修复前不可用**（2026-11-23 已修复）：D1 电机不转、D2 1.5 s 不停、D3 无法退出；另有 D4–D10 使其无法完成“物理↔软件”判定。
4. 在修复并跑完第 5 节的验收表之前，`HANDOVER.md` 第 28 行“前进右偏”不应排除映射/极性因素；`README.md` 第 52–61 行建议先标注“待实测”。
5. 修复顺序建议：D1+D2+D3（否则测试无法进行）→ D6 → D4+D5（OLED 四路）→ **实测四轮四角 + 舵机 ID** → 再统一 `P/M`、`D+/D-`、`diff_test`、`speed_tick`、EDGE 的轮对分组与差速轴 → 修正文档。

---

## 附：关键行号索引

| 内容 | 位置 |
|---|---|
| 文档接线表 | `README.md:52-61`、`HANDOVER.md:36-46` |
| 单轮测试预期行为 | `HANDOVER.md:79-82`、`README.md:27` |
| 电机/舵机表、极性 | `USER/main.c:123-128` |
| `speed_apply` 的 `motor_run` 门控 | `USER/main.c:444-450` |
| 单轮分支 | `USER/main.c:464-478` |
| 单轮 OLED | `USER/main.c:650-662` |
| `abort_all` | `USER/main.c:1062-1076` |
| 三击 / 长按 | `USER/main.c:1910-1923` |
| 无效的 1.5 s 停转 | `USER/main.c:1652-1657` |
| `speed_tick()` 调用点 | `USER/main.c:1878` |
| 左右差速（speed_tick） | `USER/main.c:517-520` |
| 前后差速（EDGE / ENTRY） | `USER/main.c:1750-1754`、`1842-1845` |
| P/M 转向标定（前后） | `USER/main.c:1693` |
| D+/D- 差速标定（左右） | `USER/main.c:1675-1676` |
| DIFFTEST（左+ 右−） | `USER/main.c:496-499` |
| `App_SetSteerRotate`（前后） | `USER/main.c:1274-1282`，`app_if.h:71` |
| 接口顺序约定 FL,FR,RL,RR | `USER/app_if.h:61,68,72` |
| 官方四驱左右归类 | `BALANCE/balance.c:245-248,401-402` |
| 官方编码器读取 | `BALANCE/balance.c:1056-1059` |
| 官方错误注释 | `BALANCE/system.c:227-238` |
| 官方左右归属结论 | `../../硬件/C30D_引脚说明.md:29` |
| 舵机正方向冲突 | `USER/centering.h:24,93` vs `HANDOVER.md:46,168` |
