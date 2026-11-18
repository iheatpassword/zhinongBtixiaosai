# 分阶段实施计划（可验证 / 可回滚）

每个阶段 = 一个 git tag + 一次烧录验证；验证不通过就 `git checkout <上一个 tag>` 重新编译烧录回滚。
所有阶段都在 `main` 分支上按顺序推进，远程备份仓库 `c30d-motor-test`（192.168.1.1）。

## 阶段总览

| 阶段 | tag | 内容 | 验证标准 | 回滚 |
|---|---|---|---|---|
| S0 | `v0.1-baseline` | 轮趣官方源码 + 直行/原地90°演示（已烧录验证） | 车能直行、能原地转90° | — |
| S1 | `v0.2-laser-link` | UART5 + DMA 接收 3 路激光数据帧、解析/滤波/超时、OLED 与串口显示；**运动逻辑不变** | 发模拟帧/接 F103 后屏幕显示 L/R/F 数值和有效标志；直行/转90°与基线完全一致 | `git checkout v0.1-baseline` |
| S2 | `v0.3-centering` | 直行对中闭环：δ（四轮同向）纠横向、Δ（前后反向）纠航向、编码器速度环 | 在垄上直行 2~3 m，左右偏差 ≤±3 cm，不碰垄；停车/急停正常 | `git checkout v0.2-laser-link` |
| S3 | `v0.4-lane-shift` | 端部横移换垄：四舵机 90°、编码器 70 cm 计程、激光精对中 | 连续换垄 5 次，每次对中偏差 ≤±2 cm；轴距/通道不干涉 | `git checkout v0.3-centering` |
| S4 | `v0.5-fsm` | 比赛状态机：进场→6 垄蛇形→出场→启动区停车 | 全流程空跑一遍，不碰围栏/垄，5 min 内完成 | `git checkout v0.4-lane-shift` |
| S5 | `v0.6-robust` | 安全/打滑检测、参数掉电保存、速度与时间优化 | 地毯路段不打滑跑偏、激光拔线 100ms 内停车、重启参数保持 | `git checkout v0.5-fsm` |

## 每阶段工作流

```powershell
Set-Location 'C:\Users\iheat\Desktop\smart wheels2\C30D_MotorTest'
git add -A
git commit -m "S1: laser UART link"
git tag -a v0.2-laser-link -m "verified: laser frames + no motion regression"
# 烧录验证通过后再打 tag；验证不通过则修改后再提交
```

回滚 + 重烧：

```powershell
git checkout v0.1-baseline           # 或 git checkout -b hotfix v0.1-baseline
& 'D:\mdk5\UV4\UV4.exe' -b .\USER\WHEELTEC.uvprojx -o build.log
& 'D:\mdk5\UV4\UV4.exe' -f .\USER\WHEELTEC.uvprojx -o flash.log
# 验证完回到主线：
git checkout main
```

## S1 具体内容（本次实施）

1. 新增 `USER/laser.c/.h`：
   - UART5 RX 用 **DMA1 Stream0/Ch4 循环缓冲**（256 B），避免 115200 下轮询丢字节；
   - 帧格式：`AA 55 | dL(2) dR(2) dF(2) | flags | CRC8`（CRC8 多项式 0x07，覆盖 AA..flags）；
   - 提供 `Laser_Data_t {dL,dR,dF,flags,valid,last_ms}`，超时 100 ms 判失效；
   - 关闭官方 UART5 的 RXNE 中断（避免与 DMA 抢数据）。
2. `USER/main.c`：初始化 DMA+激光，100 Hz 主循环里 `Laser_Poll()`；OLED 增加一行显示三路距离/有效位；USART1 每秒打印一次。
3. 运动逻辑（直行/90°）保持与 v0.1 一致，确保可回归。

**S1 验证方法**

- 无 F103 时：用 USB-TTL 接 UART5（PC12=TX 不用，**PD2=RX 接 USB-TTL 的 TX**，GND 共地），
  115200 发 10 字节帧，例如 `AA 55 96 00 8C 00 2C 01 07 <CRC>`，屏幕应显示 L=150 R=140 F=300。
- 有 F103 时：直接接上，看屏幕数值是否随遮挡变化、拔线 100 ms 内变为 `--`。
- 回归：上电后直行/原地转 90° 与之前一致。

## 参数与标定（各阶段共用，集中到 `USER/config.h`）

舵机中位 trim、舵机方向、电机方向、轮径/轮距/轴距、垄距 70 cm、激光安装偏移、
对中 PID、航向 PID、速度 PID、安全阈值（前激光停距、最大偏差、通信超时）。
