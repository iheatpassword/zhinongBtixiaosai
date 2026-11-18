# 鍒嗛樁娈靛疄鏂借鍒掞紙鍙獙璇?/ 鍙洖婊氾級

姣忎釜闃舵 = 涓€涓?git tag + 涓€娆＄儳褰曢獙璇侊紱楠岃瘉涓嶉€氳繃灏?`git checkout <涓婁竴涓?tag>` 閲嶆柊缂栬瘧鐑у綍鍥炴粴銆?鎵€鏈夐樁娈甸兘鍦?`main` 鍒嗘敮涓婃寜椤哄簭鎺ㄨ繘锛岃繙绋嬪浠戒粨搴?`c30d-motor-test`锛?92.168.1.1锛夈€?
## 闃舵鎬昏

| 闃舵 | tag | 鍐呭 | 楠岃瘉鏍囧噯 | 鍥炴粴 |
|---|---|---|---|---|
| S0 | `v0.1-baseline` | 杞叮瀹樻柟婧愮爜 + 鐩磋/鍘熷湴90掳婕旂ず锛堝凡鐑у綍楠岃瘉锛?| 杞﹁兘鐩磋銆佽兘鍘熷湴杞?0掳 | 鈥?|
| S1 | `v0.2-laser-link` | UART5 + DMA 鎺ユ敹 3 璺縺鍏夋暟鎹抚銆佽В鏋?婊ゆ尝/瓒呮椂銆丱LED 涓庝覆鍙ｆ樉绀猴紱**杩愬姩閫昏緫涓嶅彉** | 鍙戞ā鎷熷抚/鎺?F103 鍚庡睆骞曟樉绀?L/R/F 鏁板€煎拰鏈夋晥鏍囧織锛涚洿琛?杞?0掳涓庡熀绾垮畬鍏ㄤ竴鑷?| `git checkout v0.1-baseline` |
| S2 | `v0.3-centering` | 鐩磋瀵逛腑闂幆锛毼达紙鍥涜疆鍚屽悜锛夌籂妯悜銆佄旓紙鍓嶅悗鍙嶅悜锛夌籂鑸悜銆佺紪鐮佸櫒閫熷害鐜?| 鍦ㄥ瀯涓婄洿琛?2~3 m锛屽乏鍙冲亸宸?鈮ぢ? cm锛屼笉纰板瀯锛涘仠杞?鎬ュ仠姝ｅ父 | `git checkout v0.2-laser-link` |
| S3 | `v0.4-lane-shift` | 绔儴妯Щ鎹㈠瀯锛氬洓鑸垫満 90掳銆佺紪鐮佸櫒 70 cm 璁＄▼銆佹縺鍏夌簿瀵逛腑 | 杩炵画鎹㈠瀯 5 娆★紝姣忔瀵逛腑鍋忓樊 鈮ぢ? cm锛涜酱璺?閫氶亾涓嶅共娑?| `git checkout v0.3-centering` |
| S4 | `v0.5-fsm` | 姣旇禌鐘舵€佹満锛氳繘鍦衡啋6 鍨勮泧褰⑩啋鍑哄満鈫掑惎鍔ㄥ尯鍋滆溅 | 鍏ㄦ祦绋嬬┖璺戜竴閬嶏紝涓嶇鍥存爮/鍨勶紝5 min 鍐呭畬鎴?| `git checkout v0.4-lane-shift` |
| S5 | `v0.6-robust` | 瀹夊叏/鎵撴粦妫€娴嬨€佸弬鏁版帀鐢典繚瀛樸€侀€熷害涓庢椂闂翠紭鍖?| 鍦版璺涓嶆墦婊戣窇鍋忋€佹縺鍏夋嫈绾?100ms 鍐呭仠杞︺€侀噸鍚弬鏁颁繚鎸?| `git checkout v0.5-fsm` |

## 姣忛樁娈靛伐浣滄祦

```powershell
Set-Location 'C:\Users\iheat\Desktop\smart wheels2\C30D_MotorTest'
git add -A
git commit -m "S1: laser UART link"
git tag -a v0.2-laser-link -m "verified: laser frames + no motion regression"
# 鐑у綍楠岃瘉閫氳繃鍚庡啀鎵?tag锛涢獙璇佷笉閫氳繃鍒欎慨鏀瑰悗鍐嶆彁浜?```

鍥炴粴 + 閲嶇儳锛?
```powershell
git checkout v0.1-baseline           # 鎴?git checkout -b hotfix v0.1-baseline
& 'D:\mdk5\UV4\UV4.exe' -b .\USER\WHEELTEC.uvprojx -o build.log
& 'D:\mdk5\UV4\UV4.exe' -f .\USER\WHEELTEC.uvprojx -o flash.log
# 楠岃瘉瀹屽洖鍒颁富绾匡細
git checkout main
```

## S1 鍏蜂綋鍐呭锛堟湰娆″疄鏂斤級

1. 鏂板 `USER/laser.c/.h`锛?   - UART5 RX 鐢?**DMA1 Stream0/Ch4 寰幆缂撳啿**锛?56 B锛夛紝閬垮厤 115200 涓嬭疆璇涪瀛楄妭锛?   - 甯ф牸寮忥細`AA 55 | dL(2) dR(2) dF(2) | flags | CRC8`锛圕RC8 澶氶」寮?0x07锛岃鐩?AA..flags锛夛紱
   - 鎻愪緵 `Laser_Data_t {dL,dR,dF,flags,valid,last_ms}`锛岃秴鏃?100 ms 鍒ゅけ鏁堬紱
   - 鍏抽棴瀹樻柟 UART5 鐨?RXNE 涓柇锛堥伩鍏嶄笌 DMA 鎶㈡暟鎹級銆?2. `USER/main.c`锛氬垵濮嬪寲 DMA+婵€鍏夛紝100 Hz 涓诲惊鐜噷 `Laser_Poll()`锛汷LED 澧炲姞涓€琛屾樉绀轰笁璺窛绂?鏈夋晥浣嶏紱USART1 姣忕鎵撳嵃涓€娆°€?3. 杩愬姩閫昏緫锛堢洿琛?90掳锛変繚鎸佷笌 v0.1 涓€鑷达紝纭繚鍙洖褰掋€?
**S1 楠岃瘉鏂规硶**

- 鏃?F103 鏃讹細鐢?USB-TTL 鎺?UART5锛圥C12=TX 涓嶇敤锛?*PD2=RX 鎺?USB-TTL 鐨?TX**锛孏ND 鍏卞湴锛夛紝
  115200 鍙?10 瀛楄妭甯э紝渚嬪 `AA 55 96 00 8C 00 2C 01 07 <CRC>`锛屽睆骞曞簲鏄剧ず L=150 R=140 F=300銆?- 鏈?F103 鏃讹細鐩存帴鎺ヤ笂锛岀湅灞忓箷鏁板€兼槸鍚﹂殢閬尅鍙樺寲銆佹嫈绾?100 ms 鍐呭彉涓?`--`銆?- 鍥炲綊锛氫笂鐢靛悗鐩磋/鍘熷湴杞?90掳 涓庝箣鍓嶄竴鑷淬€?
## 鍙傛暟涓庢爣瀹氾紙鍚勯樁娈靛叡鐢紝闆嗕腑鍒?`USER/config.h`锛?
鑸垫満涓綅 trim銆佽埖鏈烘柟鍚戙€佺數鏈烘柟鍚戙€佽疆寰?杞窛/杞磋窛銆佸瀯璺?70 cm銆佹縺鍏夊畨瑁呭亸绉汇€?瀵逛腑 PID銆佽埅鍚?PID銆侀€熷害 PID銆佸畨鍏ㄩ槇鍊硷紙鍓嶆縺鍏夊仠璺濄€佹渶澶у亸宸€侀€氫俊瓒呮椂锛夈€?
---

## 附：每阶段详细验证步骤

### 烧录方式（已统一为 WCH 无线 DAPLink / CMSIS-DAP）

工程 `USER/WHEELTEC.uvoptx` 已设为 `<pMon>BIN\CMSIS_AGDI.dll</pMon>`，Keil 里直接 **F8**，
或命令行：

```powershell
$uv='D:\mdk5\UV4\UV4.exe'; $proj='C:\Users\iheat\Desktop\smart wheels2\C30D_MotorTest\USER\WHEELTEC.uvprojx'
& $uv -b $proj -o build.log      # 0 Error / 0 Warning
& $uv -f $proj -o flash.log      # 看 "Verify OK. Application running"
```

> WCH DAPLink 的 USB 复合设备带 COM18。COM18 是调试器自带的串口，**是否接到目标板 UART 需要确认线序**；
> C30D 的调试日志默认仍从板上「串口1」Type-C（USART1，115200）输出。

### S1 激光链路验证

1. **接线（无 F103 时用 PC 模拟）**：USB-TTL 的 TX → C30D 扩展排针 **PD2**（UART5 RX），GND 共地；
   不要接 5V；USB-TTL 设为 115200 8N1。
2. **发帧**：下面三帧都是有效帧（含 CRC8），用串口助手“HEX 发送”，每 20~50 ms 重复发一帧：
   ```
   150/140/300 mm : AA 55 96 00 8C 00 2C 01 07 93
   200/200/500 mm : AA 55 C8 00 C8 00 F4 01 07 1A
   100/300/800 mm : AA 55 64 00 2C 01 20 03 07 A2
   ```
   期望：OLED 第 3/4 行显示 `L 150 R 140`、`F 300 OK`；停止发送 100 ms 后变成 `----` / `---`（LOST）。
3. **回归**：上电后仍执行“直行 1.5 s → 原地转 +90° → 直行 → 转 −90°”，与 v0.1 行为一致。
4. 需要时用串口助手接板上「串口1」Type-C（115200），每秒会打印 `L=150 R=140 F=300 fl=7 laser=OK`。

### S2 对中直行验证

- 场地：可用两根 30×12 cm 木条/泡沫条模拟垄侧壁，间距 40 cm；或直接上比赛场地的一条垄。
- 步骤：车跨在垄上 → 上电 → 低速直行 2~3 m。
- 通过标准：左右侧向偏差 ≤ ±3 cm、车身 yaw 波动 ≤ ±2°、不碰垄侧壁；
  人为挡住一侧激光 → 100 ms 内停车；拿走遮挡后恢复。
- 记录：把 USART1 日志（e、yaw、速度）存一份，便于调 PID。

### S3 横移换垄验证

- 在端部通道内：车停在垄 1 末端 → 四舵机 90° → 横移 70 cm → 对正垄 2 → 回 0°。
- 通过标准：连续 5 次换垄，每次横向对中误差 ≤ ±2 cm；无碰撞；单次耗时记录。
- 若轴距放不进 40 cm 端部通道 → 停止该阶段，改路径方案（原地转向）。

### S4 全流程验证

- 完整跑：进场 → 垄 1..6 蛇形 → 出场 → 启动区停车。
- 通过标准：5 min 内完成、全程不碰围栏/垄、6 条垄都走完、停稳在启动区；
  全程记录日志 + 手机录像，用于复盘。

### S5 鲁棒性验证

- 地毯路段（两条垄沟）直行/换垄不跑偏、不堵转；
- 激光拔线 100 ms 内停车；重新插上恢复；
- 参数改完断电重启仍保持（Flash 保存）；
- 连续 3 次全流程用时对比。

### 回滚方法

```powershell
Set-Location 'C:\Users\iheat\Desktop\smart wheels2\C30D_MotorTest'
git checkout v0.2-laser-link     # 回到上一个已验证阶段（tag 名见上表）
& 'D:\mdk5\UV4\UV4.exe' -b .\USER\WHEELTEC.uvprojx -o build.log
& 'D:\mdk5\UV4\UV4.exe' -f .\USER\WHEELTEC.uvprojx -o flash.log
git checkout main                # 验证完回到主线继续开发
```