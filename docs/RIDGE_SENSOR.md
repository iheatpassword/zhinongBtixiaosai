# 第 4 路激光（垄侧测距 dB）协议与控制约定（v2 帧）

> 状态：C30D 解析器已升级为 **v2 12 字节帧**（保留 v1 兼容），已烧录。
> 传感器：TOF200C / VL53L0X，接 F103 软件 I2C（PB13=SCL、PB15=SDA），在 F103 侧作为 `dB`。

## 1. 用途

TOF200C 装在**前轮附近、面向车所跨的那条垄**，测「车到垄侧壁」的距离。
它直接给出**横向偏移量**，与 IMU 航向解耦：

- 航向：IMU + 前后轮差速（已验证，100 cm 漂移 ≤0.6°）；
- 横向：`dB` → 四轮同向小偏角 δ（90°±δ），不改变航向。

## 2. 帧格式（F103 → C30D，USART3 → UART4，115200 8N1，50 Hz）

```
AA 55 | dL_L dL_H | dR_L dR_H | dF_L dF_H | dB_L dB_H | flags | CRC8
 0  1  |  2     3  |  4     5  |  6     7  |  8     9  |  10   |  11
```

| 字段 | 说明 |
|---|---|
| `dL/dR/dF` | 左/右/前，mm，小端；VB02A 原始值透传（2000=超量程），`0xFFFF`=100ms 无帧 |
| **`dB`** | **垄侧（原“后”）距离，mm，小端**；TOF200C 最近有效值，`0xFFFF`=无有效样本 |
| `flags` | bit0/1/2=前/左/右有帧；bit3=至少一路 UART 超时；**bit4=dB 有有效数据** |
| `CRC8` | poly `0x07`、init `0x00`，覆盖前 11 字节 |

C30D 解析：窗口缓冲同时兼容 v2（12 字节）与 v1（10 字节，无 dB）；CRC 优先按 v2 校验，失败再按 v1。

## 3. C30D 侧实现

- `LaserData`：`dRidge`（=dB）、`ridge_flags`（bit0=有效）、`ridge_valid`、`ridge_ms`；
- `ridge_valid` 由 flags bit4 置位，100 ms 无 v2 帧清零；
- 日志：`RG=dRidge,ridge_flags,ridge_valid`（`-1` 表示 `0xFFFF`）；
- 平移控制（仅 `ridge_valid=1` 时）：

```
e  = dRidge - RIDGE_TARGET_MM
δ  = clamp(RIDGE_SIGN * RIDGE_KP * e, ±RIDGE_OFFSET_MAX)
四轮角度 = 90° + δ
```

当前参数：`RIDGE_TARGET_MM=120`、`RIDGE_KP=0.05`、`RIDGE_OFFSET_MAX=6°`、`RIDGE_SIGN=+1`。
无效时 δ=0，自动退回纯航向保持。

## 4. 标定与验收

1. **链路**：日志 `RG=` 持续刷新、`ridge_valid=1`，与卷尺读数一致（±2 mm）；
2. **标定目标**：车正确跨垄摆好，读 `RG` 作为 `RIDGE_TARGET_MM`；
3. **方向**：手推靠近垄壁（RG 变小），观察 δ 是否让车远离；反了翻 `RIDGE_SIGN`；
4. **平移 100 cm**：`RG` 波动 ≤±15 mm，yaw ≤2°；
5. **平移 220 cm**：不碰垄、不丢数据；
6. 通过后 tag `p2-ridge-sensor`。

## 5. 备注

- TOF200C 克隆片有已知坑（见 `F103RCT6_3LIDAR/README.md` 第 6 节）；若精度/速率/抗干扰不够，换 VB02A（UART）需再占一路串口；
- 新传感器与电机共地噪声会影响测距，建议独立供电/加滤波。
