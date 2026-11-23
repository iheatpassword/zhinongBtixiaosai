# C30D 串口调参台（serial_tuner）

C30D 四轮四舵小车 STM32 固件的**纯前端、离线可用**串口调参/监视页面。
通过浏览器 Web Serial API 连接 USART1（COM18，115200 8N1）实时解析 1 Hz 日志，
显示 YAW / F2 / FA / 电池 / 四轮速度与 PWM / 激光 / 链路，支持趋势图、命令发送、
差速 PID 参数在线修改和日志导出。

- **无 CDN、无 npm 依赖**，可断网使用；
- **中文界面**，响应式布局（桌面 / 窄屏均可）；
- **演示模式**：没有串口、没有车也能看到完整界面和图表（打开页面自动回放 `sample_log.txt`）；
- 只读取/发送串口，不修改任何固件文件。

> 安全第一：`A/E/F/B/G/D/P/T` 等命令会让小车动作。本页面默认**不发送**危险命令，
> 需要先勾选“确认会动车”，再在 3 秒内二次确认。测试时请架起车或清空场地。

---

## 1. 文件清单

| 文件 | 作用 |
|---|---|
| `index.html` | 页面结构（中文 UI） |
| `style.css` | 深色响应式样式（无外部字体/图片） |
| `parser.js` | 日志解析器，ES module 导出 `parseLine(line)` 及 F2/FA 名称映射 |
| `app.js` | UI / Web Serial / Canvas 趋势图 / 命令 / 参数 / 日志 |
| `test_parser.js` | Node 单元测试（不依赖浏览器） |
| `sample_log.txt` | 演示日志（合成的示例数据，格式与固件 1 Hz 日志一致） |
| `serve.ps1` | 用 `python -m http.server` 启动本地服务器（默认端口 8765） |
| `package.json` | 仅声明 `"type": "module"` 和 `npm test`，**没有任何依赖** |

---

## 2. 快速开始

### 2.1 启动本地服务器

Web Serial 只在**安全上下文**（`http://localhost`、`http://127.0.0.1` 或 https）可用；
另外 ES module 在 `file://` 下会被浏览器拦截，所以**不要直接双击 `index.html`**。

在本目录执行（任选其一）：

```powershell
# 方式 A：脚本（推荐，会自动找 python/py 并尝试打开浏览器）
powershell -ExecutionPolicy Bypass -File .\serve.ps1

# 方式 B：手动
python -m http.server 8765
# 或
py -m http.server 8765
```

然后在浏览器打开：**<http://localhost:8765>**

> 端口被占用时换一个：`python -m http.server 8766`，或 `.\serve.ps1 -Port 8766 -NoBrowser`。

### 2.2 连接串口

1. 用**桌面版 Chrome / Edge**（89+）打开上面的地址；
2. 先关掉可能占用 COM18 的程序（其它串口助手、残留的 PowerShell、IDE 下载器等）；
3. 点页面右上角 **连接串口**；
4. 在弹出的浏览器串口选择器里选择 C30D 的 USB 串口设备（CH9102 / USB-SERIAL），
   波特率固定 115200、8 数据位、1 停止位、无校验、无流控；
5. 连接成功后，1 Hz 日志会自动刷新卡片和曲线。点 **断开** 释放串口。

浏览器出于隐私**不显示 COM 号**，选择器里通常只显示设备名/VID-PID。可以用“拔插对比”确认：
先插着打开选择器，再拔掉看哪个条目消失。页面状态栏会显示 `USB VVVV:PPPP`。

> Firefox / Safari 当前不支持 Web Serial（本页面会提示改用 Chrome/Edge，并自动进入演示模式）。

### 2.3 无串口：演示模式

打开页面默认就是演示模式：自动读取 `sample_log.txt`，以 1 Hz 循环回放并预填充趋势图。
点右上角 **停止演示** 可停；再次点 **演示模式** 可重新开始。
演示模式下命令不会真正发送（日志里会记录 `→ 命令（演示模式，未真正发送）`）。

---

## 3. 页面功能

### 3.1 状态条

数据源（串口/演示）、收到的日志行数、解析错误数、最后更新时间、链路信息（115200 8N1）。

### 3.2 实时卡片

- **YAW 航向**：`YAW/10` 转为度，显示正负、与上一帧比较的趋势（↑/↓/→）；
- **F2 状态机**：英文状态名 + 中文短标签（见 `USER/fsm2.h`）；
- **FA 故障**：中文故障名；非 0 时卡片变红（故障码见 `USER/fsm2.h`）；
- **电池电压**：`VB/10` 转为 V；< 11.5 V 黄色提醒，< 10.5 V 红色（固件低于 10.5 V 会 `LOW BATT STOP`）；
- **运行模式**：`SPD FWD/BACK/STOP`（或开环 `OL`）、`CMD`、`MR/CM/P`；
- **目标速度**：`tgt`（counts/20 ms）与 `T0`；
- **激光/链路**：`dL/dR/dF`（mm，`-1` 显示 `--`）、`dB`（来自 `RG`，垄侧/后向 TOF）、
  `LZ=OK/LOST` 链路、LS flags 的 F/L/R 有效位、`RG` 详情；
- **四轮 FL/FR/RL/RR**：速度（进度条 + 数值，counts/20 ms）与 PWM（进度条 + 数值）。

### 3.3 趋势图（Canvas，无外部库）

三张图：YAW、四轮速度、激光 `dL/dR/dF`。时间轴按浏览器接收时间，保留最近 **240 点**
（1 Hz 约 4 分钟）。支持 **暂停曲线**（暂停画图，解析和卡片照常）与 **清空曲线**。

### 3.4 命令控制台

- 文本输入 + **发送**，发送时自动追加 `\r\n`（固件按 `\r`/`\n` 断行，`cmd_poll()` 逐行执行）；
- 历史记录（本机 localStorage，最多 50 条，点击填回输入框，↑/↓ 键调出）；
- 快捷按钮：`S` / `Q` / `F` / `B` / `C` / `T` / `T-` / `H`；
- **危险命令安全门**：`A/E/F/B/G/D/P/T`（另外把实际也会驱动电机的 `M/O/N` 一并纳入）
  必须：
  1. 勾选 **“确认会动车”**（默认不勾选）；
  2. 在 3 秒内**再次点击/回车**做二次确认。
  任一条件不满足都不会发送。按 Esc 取消待确认状态。

### 3.5 参数表单

- 差速航向 PID：`K kp ki kd max [ff]`。
  - 页面上的 Kp/Ki/Kd 是**工程值**（如 2.0），发送时按固件约定 **×100 取整** 变成 `200`；
  - `max` 直接发送；
  - 预览区实时显示将要发送的原始命令；
  - 默认值来自 `USER/main.c`：`0 / 0 / 0 / 600`，`ff=-55`（即发送 `K 100 0 0 600`；表单保留 ff，第 5 个参数用命令台发）。
  - 页面表单只发 4 个参数，固件会保留当前 `ff`；要调固定差速偏置，请在命令台发 `K 578 144 0 433 -80`（负值 = 向左修正）。
- 目标速度：`V<n>`（n = counts/20 ms）。
  - 固件代码 `SPD_MAX=2000`，页面允许 0~2000。注意 `cmd_exec()` 的 `V` 错误提示写的是
    “0..600”，那是旧值，以 `SPD_MAX` 为准。
- 显示“最近发送”命令与时间；**保存到本机** / **恢复默认**。
- 参数保存于 localStorage，键名 `c30d.serialTuner.params.v1`；
  发送 K/V 成功后也会自动记住当前值。

### 3.6 日志面板

- 显示原始行 + 接收时间（毫秒）；
- 日志行 / 命令回显（灰）/ 系统消息（蓝）分色；**解析失败的行标红**，不会中断程序；
- **暂停滚动**（只停自动滚动，数据继续接收）、**只显示日志行**过滤；
- **导出 .txt**：带 UTF-8 BOM 和中文文件头，时间戳 + 原始行，Windows 记事本可直接打开；
- **清空** 只清当前会话日志。

### 3.7 更多字段（可选，默认折叠）

`DD` 里程（cm）、`RD/DF/AF`、`LE/LD`、`LC`、`CE/CD/CT`、`SY`、`C`、`SVO`、`P`、`K`、
`R1/R2/R3`、`ST`（旧状态机，已删除）、`LZN`、`LB`（激光原始帧 8 字节）。

---

## 4. 日志格式与解析（以固件代码为准）

固件在 `USER/main.c` 约 2015–2119 行、每 1000 ms 向 USART1/USART3/UART5 各打印一行：

```text
SPD FWD tgt=150 MR=1 CM=0 T0=150 2400/4123 2390/4150 2370/4090 2385/4110
F2=2 FA=0 K=5 VB=118 SY=3 R1=120,5 R2=340,-2 R3=560,8 LE=0 LD=1 LC=175620
DD=123 RD=148 DF=7 AF=2 CE=12 CD=3 CT=1 ST=4 LZ=OK LS=450,600,206,7
RG=137,1,1 LZN=1234 LB=AA 55 01 02 03 04 05 06 YAW=12 C=3 SVO=0 P=1 CMD=F
```

（上面为便于阅读折行，实际是单行。开环诊断时前缀变为 `OL PWM=<pwm>`。）

解析规则（`parser.js`）：

- 按 `KEY=value` 切分，**不依赖字段顺序**；KEY 大小写不敏感；
- **缺失字段** → `null`，不报错（例如 `ST=` 已删除，样例里前半段有、后半段没有）；
- **未知/多余字段** → 放入 `extras`，不报错；
- **已知字段值非法** → `errors` 记录且 `ok=false`，界面标红，但永不抛异常；
- 四轮是无键位置字段 `<spd>/<pwm>`，顺序 **FL, FR, RL, RR**，解析时先摘出再切 key=value；
- `LZ=OK ` / `LZ=LOST `（固件带尾随空格，可能与下一字段间有双空格）→ 归一化为 `OK/LOST`；
- `CMD=` 是 `last_cmd`（最长 7 字符，**可能为空**或含空格，如 `K 200 5`），按 key 分段完整保留；
- 开环行 `OL PWM=2500 ...` 单独识别为 `open-loop`。

字段映射：

| 日志字段 | 含义 | 解析结果 |
|---|---|---|
| `tgt` | `spd_target` 目标速度 | `tgt`（counts/20 ms） |
| `MR` | `motor_run`（1 前进 / -1 后退 / 0 停） | `mr` |
| `CM` | `cent_mode` | `cm` |
| `T0` | `spd_tgt[0]` | `t0` |
| `<spd>/<pwm>` ×4 | `meas_spd` / `pwm_out`，FL FR RL RR | `wheels[0..3]` |
| `F2` | `Fsm2_GetState()` | `f2` + `f2Name` + `f2Label` |
| `FA` | `Fsm2_GetFaultCode()` | `fa` + `faName` |
| `K` | `key_raw` | `k` |
| `VB` | 电池电压 ×10 | `vbRaw`、`vb`（V） |
| `SY` | `run_yaw_deg_dbg` | `sy`（度） |
| `R1/R2/R3` | `run_hist_cnt, run_hist_yaw` | `r1/r2/r3 = {count,yaw}` |
| `LE/LD/LC` | `run_lat_off_dbg / run_lat_deg_dbg / last_run_counts` | 同名 |
| `DD` | `App_GetDistanceCm() - shift_base_cm` | `dd`（cm） |
| `RD/DF/AF` | `shift_ridge_dbg / shift_doff_dbg / shift_af_dbg` | 同名 |
| `CE/CD/CT` | 旧对中模块 ×10 调试值 | `ce/cd/ct` |
| `ST` | 旧 `FSM_GetState()`（**已删除**，缺失属正常） | `st` |
| `LZ` | `Laser_IsFresh()` → `OK` / `LOST` | `lz` |
| `LS` | `dL,dR,dF,flags`（mm，`-1` 无效） | `ls.{dL,dR,dF,flags,valid}` |
| `RG` | `dB,ridge_flags,ridge_valid`（mm，`-1` 无效） | `ridge`、`dB` |
| `LZN` | 激光原始帧计数 | `lzn` |
| `LB` | 最后 8 个激光原始字节（十六进制） | `lb`（字节数组） |
| `YAW` | `yaw_deg × 10` | `yawRaw`、`yaw`（度） |
| `C` | `yaw_corr` | `c` |
| `SVO` | `servo_angle` | `svo`（度） |
| `P` | `last_port`（1/3/5 → 哪个串口收到的命令） | `p` |
| `CMD` | `last_cmd` | `cmd`（字符串，可为空） |

LS flags 位定义（`USER/main.c`）：bit0 = F 有效，bit1 = L 有效，bit2 = R 有效。
`dB` 不在 `LS` 里，而是 `RG` 的第一个字段（`app_if.h` 的 `dRidge`，垄侧/后向 TOF）。

结构示例（`parseLine()` 返回）：

```js
{
  ok: true, type: 'log', errors: [], dir: 'FWD',
  tgt: 150, mr: 1, cm: 0, t0: 150,
  wheels: [{spd: 2400, pwm: 4123}, ...4 个],
  f2: 2, f2Name: 'CRAB_R1', f2Label: '右蟹行 R1',
  fa: 0, faName: '无故障',
  vbRaw: 118, vb: 11.8, yawRaw: 12, yaw: 1.2,
  ls: { dL: 450, dR: 600, dF: 206, flags: 7, valid: {f:true,l:true,r:true} },
  ridge: { dist: 137, flags: 1, valid: true }, dB: 137,
  lb: [0xAA, 0x55, ...], cmd: 'F',
  // 其余字段见 emptyResult()
}
```

---

## 5. 串口命令（HANDOVER.md 第 4 节 / `cmd_exec()`）

| 命令 | 作用 | 危险 |
|---|---|---|
| `A` | 启动 FSM2（等同单击，保留调试） | ⚠ |
| `E` | 贴边模式（等同双击） | ⚠ |
| `Q` | 中止所有动作 / 回 IDLE | 安全 |
| `S`（或 `X`） | 停车，清所有模式 | 安全 |
| `F` / `B` | 前进 / 后退（速度闭环） | ⚠ |
| `V<n>` | 设置目标速度（counts/20 ms），如 `V150` | 安全（只改目标） |
| `K kp ki kd max [ff]` | 在线设置差速航向 PID（整数 ×100；`ff` 为固定差速偏置，可省略） | 安全 |
| `O<n>` | 开环 PWM 诊断，如 `O2500` | ⚠（电机会转） |
| `G` | 70 cm 直行测试 | ⚠ |
| `D+` / `D-` | 差速标定（前进 1 m 后打印 yaw） | ⚠ |
| `P` / `M` | 转向标定（前 +8°/后 −8° 及反向） | ⚠ |
| `T` / `T-` | 四舵机 +90° / −90° | ⚠（会打舵） |
| `C` | 四舵机回中 | 安全 |
| `N` | 旧对中测试 | ⚠ |
| `H` / `?` | 帮助 | 安全 |

固件会回显 `OK ...` / `ERR ...`，同时 `CMD=` 字段会显示最后收到的命令。
注意 `cmd_exec()` 在 FSM2 运行中会拒绝 `F/B/S/G/V/O` 并回 `ERR BUSY (RSM: A/Q)`。

---

## 6. 测试

```powershell
cd C30D_MotorTest\tools\serial_tuner
node test_parser.js
```

用 Node（建议 18+）直接运行，无需安装任何包。当前覆盖 **29 项**断言：

- 完整行、字段乱序、缺字段、未知字段、大小写、CRLF；
- F2/FA 名称、YAW/10、VB/10、四轮顺序、R1/R2/R3、LB/LZN；
- LS 无效值 `-1`、flags 位映射、RG/dB、LS/RG 缺字段与空值；
- 开环 `OL PWM=` 行、命令回显/帮助文本、空行/null、损坏行（标红但崩溃不了）、
  畸形输入不抛异常；
- `sample_log.txt` 每行可解析，且包含 F2/FA/LS/RG/YAW/LZ、FAULT、LOST、激光无效、
  带/不带 `ST=` 的行。

`npm test` 等价于 `node test_parser.js`（`package.json` 只声明 `type: module`，无依赖）。

页面侧无自动化浏览器测试；手工验收：

1. `python -m http.server 8765`；
2. 打开 <http://localhost:8765>，应自动看到演示数据、卡片、三张趋势图；
3. 若安装了串口，点“连接串口”按第 2.2 节操作。

---

## 7. 故障排查

| 现象 | 处理 |
|---|---|
| 打开页面白屏，提示“页面脚本没有加载成功” | 用了 `file://`。改用 `serve.ps1` / `python -m http.server`，访问 `http://localhost:8765` |
| “当前浏览器不支持 Web Serial” | 换桌面版 Chrome/Edge 89+；Firefox/Safari 不支持；演示模式仍可用 |
| 连接按钮点不动 | 页面不在安全上下文：必须 `http://localhost` / `127.0.0.1` / https |
| 串口选择器里没有设备 | 换数据线/USB 口；确认驱动已装（设备管理器出现 COM 口）；关闭其它串口程序 |
| 打开串口失败 / `NotFoundError` | COM18 被占用（串口助手、Keil 下载器、残留进程）；关掉后重试或重插 DAPLink |
| 连上后没有数据 | 确认接的是 USART1（COM18）、115200 8N1；固件是否在跑；看日志面板有无任何行 |
| 趋势图不动 | 检查是否点了“暂停曲线”；确认“数据源”是串口/演示且“最后更新”在变 |
| 解析错误数增长 | 有损坏/截断行，日志面板红色行即原文；不影响后续解析 |
| 参数没保存 | 隐私模式/无痕窗口可能禁用 localStorage；本次会话仍可正常发送 |
| 端口 8765 被占用 | `python -m http.server 8766`，或 `.\serve.ps1 -Port 8766 -NoBrowser` |
| `serve.ps1` 报语法错误 / 中文乱码 | 脚本是 **UTF-8 BOM** 编码；若被编辑器另存为 ANSI 或无 BOM，Windows PowerShell 5.1 会按本地 ANSI 解码导致解析失败。请保留 UTF-8 BOM，或直接用 `python -m http.server 8765` |

---

## 8. 已知限制

1. **仅 Chromium 桌面浏览器**支持 Web Serial；浏览器不暴露 COM 号，串口选择器只能靠 VID/PID/拔插区分。
2. **串口独占**：同一 COM 口只能被一个页面/程序打开；页面刷新后需要重新选择串口。
3. 数据只有 **1 Hz**，卡片和曲线更新慢；趋势图上限 240 点（约 4 分钟）。
4. 日志行**不带时间戳**，页面时间轴使用浏览器接收时间；串口缓冲/阻塞会造成抖动。
5. `parser.js` 按当前固件字段实现；如果固件改字段名/顺序/单位，需要同步修改解析器。
6. 开环 `OL` 行能解析，但卡片仍按同一套字段显示（方向显示 `OL`），不会额外区分开环模式。
7. 演示数据 `sample_log.txt` 是**合成的**（字段格式与真实固件一致，但数值不是实车抓取）。
8. 危险命令安全门只是软件层提醒，**不能替代物理急停**；真正安全的是架起车、断电、有人看管。
9. `serve.ps1` 依赖系统已安装的 `python` 或 `py`；没有 Python 时请用任意静态服务器。
   该脚本必须是 **UTF-8 BOM** 编码（Windows PowerShell 5.1 才能正确读取中文），编辑后请勿存成 ANSI。
10. 未实现固件烧录/OTA、日志持久化存储、多串口同时监视。

---

## 9. 安全注意（务必阅读）

- **实车移动一律优先用按键触发**（项目约定），串口命令只在明确批准时使用；
- 页面打开时默认是演示模式，**不会**连接或发送任何东西；
- 危险命令默认被拦截；勾选“确认会动车”只代表你确认了风险，仍需二次点击/回车；
- 发 `F/B/A/E/G/D/P/T/O/N/M` 前确认：车架空或场地清空、有人看管、随时能按 `S` 或断开串口；
- 激光安全阈值：任意有效激光 < 30 mm 固件会停车；电池 < 10.5 V 会 `LOW BATT STOP`；
- 串口日志/命令是本机明文，不要在连接实车时运行来路不明的页面。

---

## 10. 实现备注

- `parser.js` 为纯函数、无 DOM/串口依赖，浏览器与 Node 共用；
- `app.js` 负责 Web Serial 读写、三张 Canvas 趋势图、卡片刷新、命令安全门、参数与日志；
- `package.json` 的 `"type": "module"` 只是为了让 Node 按 ES module 加载 `parser.js`，
  从而支持 `node test_parser.js`；它不是 npm 工程，**没有 dependencies**；
- 趋势图直接用 Canvas 2D 绘制（网格、坐标轴、时间标签、断线跳过无效点），无图表库；
- 未修改 `C30D_MotorTest/USER/` 或其它固件文件，本目录可整体拷贝/离线打开（需本地服务器）。
