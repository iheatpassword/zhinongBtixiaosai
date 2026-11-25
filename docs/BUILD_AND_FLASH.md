# 编译与烧录（2026-11-25 实测）

本文记录这台机器上 **C30D_CubeMX_Route** 的实际构建/烧录方式。命令都跑过；
标注"已验证"的是本次执行过的，标注"待现场"的是需要接真实板子才能确认的。

## 1. 工具链从哪来

工具链不装进系统 PATH，而是由 **STM32CubeIDE for VS Code** 扩展的 bundle 管理：

| 组件 | 版本 | 路径（`%LOCALAPPDATA%\stm32cube\bundles\<包>\<版本>\bin`） |
|---|---|---|
| arm-none-eabi-gcc | 15.3.1 (ST 15.3.rel2) | `gnu-tools-for-stm32\15.3.1+st.1\bin` |
| arm-none-eabi-gdb | 15.3.1 | `gnu-gdb-for-stm32\15.3.1+st.1\bin` |
| cmake | 4.4.3 | `cmake\4.4.3+st.1\bin` |
| ninja | 1.13.2 | `ninja\1.13.2+st.1\bin` |
| STM32_Programmer_CLI | 2.24.0 | `programmer\2.24.0+st.1\bin` |
| OpenOCD（DAPLink 用） | 0.12.1+st.2 | `openocd-gdbserver\0.12.1+st.2\bin` |

一次性安装（需要能访问 `developer.st.com`，本机走 `HTTPS_PROXY=http://127.0.0.1:7892`）：

```powershell
$cube = Get-ChildItem "$env:USERPROFILE\.vscode\extensions" -Directory |
        Where-Object Name -match 'stm32cube-ide-core' | Select-Object -First 1
$cube = "$($cube.FullName)\resources\binaries\win32\x86_64\cube.exe"
& $cube bundle install gnu-tools-for-stm32 gnu-gdb-for-stm32 cmake ninja programmer openocd-gdbserver openocd-script --yes
```

每个新终端拼一次 PATH：

```powershell
$b = "$env:LOCALAPPDATA\stm32cube\bundles"
$env:PATH = "$b\gnu-tools-for-stm32\15.3.1+st.1\bin;$b\ninja\1.13.2+st.1\bin;$b\cmake\4.4.3+st.1\bin;$b\gnu-gdb-for-stm32\15.3.1+st.1\bin;$b\programmer\2.24.0+st.1\bin;$b\openocd-gdbserver\0.12.1+st.2\bin;$env:PATH"
```

> 注意 `cube` 本身只能直接跑 `cmake` / `ninja` / `programmer` / `arm-none-eabi-gdb`
> 这些 runtime，**`gcc` 不是 cube 的 runtime**（`cube gcc` 会报 No such command），
> 编译必须靠 PATH。

## 2. 编译（已验证）

```powershell
cd CALIBRATION_WORKFLOW\C30D_CubeMX_Route
cmake --preset Debug        # 约 7 s
cmake --build --preset Debug
```

实测（提交 `cc91f5d`）：

```
RAM:   34456 B / 128 KB (26.29%)
FLASH: 74180 B / 512 KB (14.15%)
[69/69] Linking C executable C30D_CubeMX.elf
```

产物：`build/Debug/C30D_CubeMX.elf`（约 1.66 MB，含调试信息）与 `.map`。
**CubeMX 的 CMake 模板不生成 hex/bin**，只有 elf；烧录工具都能直接吃 elf。

`Release` 用同一套命令换 `-Preset Release` 即可。

## 3. 烧录

### 3.1 ST-Link（已验证命令，未在本次硬件上执行）

```powershell
STM32_Programmer_CLI.exe -c port=SWD -w build\Debug\C30D_CubeMX.elf -v -rst
```

带上 `-v` 校验、`-rst` 复位运行；先 `-l` 看探针有没有认到。

### 3.2 DAPLink / CMSIS-DAP（OpenOCD）

**STM32CubeProgrammer 不支持 CMSIS-DAP**——它只列 ST-LINK / J-Link / UART / DFU，
插着 DAPLink 也会报 `No ST-Link detected!`。这是工具限制，不是探针坏。
DAPLink 必须走 OpenOCD：

```powershell
$ocd = "$env:LOCALAPPDATA\stm32cube\bundles\openocd-gdbserver\0.12.1+st.2\bin\openocd.exe"
$scr = "$env:LOCALAPPDATA\stm32cube\bundles\openocd-gdbserver\0.12.1+st.2\scripts"

& $ocd -s $scr -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg `
  -c 'program "build/Debug/C30D_CubeMX.elf" verify reset exit'
```

成功标志：`** Verified OK **` 与 `shutdown command invoked`。

只读探测（不擦芯片，用来确认探针和芯片）：

```powershell
& $ocd -s $scr -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg `
  -c init -c targets -c "flash probe 0" -c "mdw 0xE0042000 1" -c shutdown
```

本次实测读数（Horco CMSIS-DAP v2 + C30D 板）：

| 项 | 值 |
|---|---|
| 探针 | `CMSIS-DAPv2 VID:PID=0xfaed:0x4873 serial=1b117a26 FW=V208` |
| 内核 | `SWD DPIDR 0x2ba01477`，Cortex-M4 r0p1，2000 kHz |
| 芯片 | `DBGMCU_IDCODE 0x101f6413`（0x413 系列），`flash size = 512 KiB` → STM32F407VE |

替代方案：`pip install pyocd` → `pyocd flash -t stm32f407ve <elf>`，对 DAPLink 支持最原生。

⚠️ 烧录会覆盖板上现有固件。C30D 是比赛用车时，烧录前先确认当前固件版本。

## 4. 在 VS Code 里用

- 扩展 `stmicroelectronics.stm32-vscode-extension`（会带齐 `stm32cube-ide-*` 子扩展）。
- 设置 `stm32cube-ide-core.configuration.productSTM32CubeMX.executablePath`
  指向 `D:\fuckcubemx\STM32CubeMX.exe`，`.ioc` 双击即可打开 CubeMX。
- 调试：ST-Link 用 `stm32cube-ide-debug-stlink-gdbserver`；
  DAPLink 用 `stm32cube-ide-debug-openocd-gdbserver`（参数同上
  `-f interface/cmsis-dap.cfg -f target/stm32f4x.cfg`）。

## 5. 故障对照

| 现象 | 处置 |
|---|---|
| `cmake` 报找不到 `arm-none-eabi-gcc` | PATH 没拼（§1），或 bundle 没装：`cube bundle install gnu-tools-for-stm32 --yes` |
| 插着 DAPLink 仍报 `No ST-Link detected!` | CubeProgrammer 不支持 CMSIS-DAP，改用 §3.2 |
| openocd `unable to find a matching CMSIS-DAP device` | 探针没插好/驱动异常，先跑只读探测 |
| openocd `Can't find interface/cmsis-dap.cfg` | 少写 `-s <bundle>\scripts` |
| openocd `Error: init mode failed` | 目标没供电或 SWDIO/SWCLK 没接；可试 `-c "adapter speed 1000"` |
| 只有 `.elf` 没有 `.hex` | 正常；要 hex 加 `arm-none-eabi-objcopy -O ihex` 到 CMake 的 post-build |
| 重新生成 `.ioc` 后手写代码丢失 | 只写在 `USER CODE BEGIN/END` 之间；`ProjectManager.KeepUserCode=true` |
