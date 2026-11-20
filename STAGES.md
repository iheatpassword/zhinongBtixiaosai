# 分阶段实施计划（可验证 / 可回滚）

每个阶段 = 一个 git tag + 一次实车/台架验证；验证不通过就 `git checkout <上一个 tag>` 重新编译烧录回滚。
所有阶段在 `main` 分支上顺序推进，远程备份仓库 `c30d-motor-test`（ZeroTier `10.206.159.40`，局域网时用 `192.168.1.1`）。

## 阶段总览

| 阶段 | tag | 内容 | 验证标准 | 当前状态 |
|---|---|---|---|---|
| S0 | `v0.1-baseline` | 官方源码 + 直行/原地 90° 演示 | 车能直行、能原地转 90° | ✅ 完成 |
| S1 | `v0.2-laser-link`（计划）/ `v0.3.2-laser-mon`（已做） | 激光链路：UART4（原计划 UART5）+ DMA 接收 3 路激光帧、解析/滤波/超时、显示 | 收到有效帧；拔线 100 ms 内判失效；运动逻辑不回归 | ⚠️ 阻塞（F103 接线未通，接口改到 UART4） |
| S2 | `v0.3-centering` | 对中直行：δ（四轮同向纠横向）+ Δ（前后反向纠航向）+ 编码器速度环 + IMU 航向 | 跨垄直行 2~3 m 侧向偏差 ≤±3 cm，不碰垄；激光失效 100 ms 内停车 | 🟡 35%（速度环/70cm/IMU 已完成，横向对中未开始） |
| S3 | `v0.4-lane-shift` | 端部横移换垄：四舵机 90°、编码器 70 cm 计程、激光精对中 | 连续换垄 5 次，每次对中误差 ≤±2 cm；轴距/通道不干涉 | 🔴 未开始 |
| S4 | `v0.5-fsm` | 比赛状态机：进场→6 垄蛇形→出场→启动区停车 | 全流程空跑，不碰围栏/垄，5 min 内完成 | 🔴 未开始 |
| S5 | `v0.6-robust` | 安全/打滑检测、参数掉电保存、速度与时间优化 | 地毯路段不跑偏、激光拔线 100 ms 内停车、重启参数保持 | 🔴 未开始 |

## 串口分配（2026-11-20 调整）

| 链路 | 外设 | 引脚 | 说明 |
|---|---|---|---|
| 3 路激光（F103） | **UART4** | PC10=TX / PC11=RX | 原方案为 UART5；UART5 现已作无线调试指令口 |
| 无线调试/指令 | UART5 | PC12=TX / PD2=RX | 与 USART1/USART3 三路并存 |
| 板载调试 | USART1 | PA9/PA10 | Type-C 串口1 |
| 备用 | USART3 | PD8/PD9 | Type-C 串口3 |
| 总线舵机 | USART2 | PD5/PD6 | BusLinker + 4×HTS-35H |
| 板载 IMU | I2C | PB10/PB11 | ICM-20948 |

## 每阶段工作流

```powershell
Set-Location 'C:\Users\iheat\Desktop\smart wheels2\C30D_MotorTest'
git add -A
git commit -m "S2: centering straight"
git tag -a v0.3-centering -m "verified: centering + no regression"
# 烧录验证通过后再打 tag；不通过则修改后重新验证
```

回滚 + 重烧：

```powershell
git checkout v0.4-speed-loop          # 或 git checkout -b hotfix <tag>
& 'D:\mdk5\UV4\UV4.exe' -b .\USER\WHEELTEC.uvprojx -o build.log
& 'D:\mdk5\UV4\UV4.exe' -f .\USER\WHEELTEC.uvprojx -o flash.log
git checkout main                     # 验证完回到主线
```

## S2 具体内容（下一步实施）

1. 激光链路（UART4）：恢复 `laser.c` 并接入主循环，帧格式 `AA 55 dL(2) dR(2) dF(2) flags CRC8`，超时 100 ms 判失效；
2. 对中控制（50 Hz）：
   - 横向：`e=(dL−dR)/2` → PD → δ（四轮同向偏转，限幅 ±8°）；
   - 航向：IMU yaw → PD → Δ（前轮 +Δ、后轮 −Δ，限幅 ±5°）；
   - 速度：现有编码器速度环（左右同步）；
3. 安全：激光失效/超量程 → 停车；前激光 < 阈值 → 减速/停车。

## S3 具体内容（S2 通过后）

四舵机 90° 横移，编码器计程 70 cm + 激光识别垄边沿精对中；单次换垄对中误差 ≤±2 cm。

## S4/S5

S4：状态机串联进场→6 垄蛇形→出场→启动区停车，空跑计时。
S5：地毯段打滑检测、参数写 Flash、激光故障降级、速度优化。

## 参数与标定（各阶段共用）

舵机中位 trim、舵机方向、电机方向、轮径（标定 76.4 mm）、轮距/轴距、垄距 70 cm、
激光安装偏移、对中 PID、航向 PID、速度 PID、安全阈值（前激光停距、最大偏差、通信超时）。
