# AxxPD — Project Guide & Handover

USB-C PD 3.1 EPR bench power controller. STM32G491CCU6 (Cortex-M4F, 128 MHz).
Firmware is **bootloader-less**: app linked at `0x08000000`. Firmware update = hold a
button at power-on to enter the STM32 ROM DFU, flash over USB (or SWD).

---

## 🔴 CURRENT STATUS — read this first (2026-08-01)

**8 fix commits sit on local `main`, UNPUSHED.** `origin/main` (`357ae83`) is the
working baseline the user field-tests. Nothing broken is public.

### The open blocker: latent PD-stack buffer-overflow bug
While hardware-testing the fix batch, the new firmware fault-looped on boot: a
**message-driven out-of-bounds WRITE** in the vendored pdsink stack smashes an afsm
dispatch-table function pointer → garbage-jump HardFault (CFSR=0x101 IACCVIOL, PC =
varying garbage, fault context = `SysTick → axxpd_tick_pd → afsm dispatch`).

**Root cause (per code review):** ETL bounds-checks are compiled OUT, so a fixed-buffer
overflow silently corrupts adjacent memory. The prime trigger is the `source_caps`
push_back overflow (`pe.cpp:244`, `etl::vector` cap 11, a `Port` member next to FSM
state); a malformed/edge `Source_Capabilities` with a large `size_to_pdo_count` overruns
it. **This is the field symptom "sometimes I must restart the USB charger and then it
works"** — a bad charger state sends the triggering message.

**Believed already fixed** by these commits (needs confirmation on a *clean* charger):
- `606f331` — `source_caps` overflow clamp (the load-bearing fix), EPR/sender-response timers
- `5b27638` — timer-id + message-resize bounds guards (defense-in-depth)
- `494a118` — local chunked-resize clamp + chunk-buffer static_assert

⚠️ **Caveat:** the earlier "any code change unmasks it / layout-sensitive" reading was
likely **confounded** — the charger degraded *during* the ~30-cycle test session, so late
fault-loops (incl. a "baseline + 1 noop line" test) were probably the *bad charger*, not
layout. Must be re-tested on a clean charger to be sure. If it still fault-loops on a
**clean** charger, a genuine second (layout) trigger remains — resume the hunt then.

### NEXT STEPS (for the continuing agent)
1. **User power-cycles the USB charger** (device is currently stuck in HardFault because
   the charger is latched — NOT bricked; MCU alive at 3.28 V on baseline). It recovers on
   power-cycle.
2. Build HEAD (`cd Firmware && make -j8 all`), flash it (see Flash below), confirm it
   **boots and enumerates COM15**. Do NOT keep flashing while the charger is latched —
   everything crashes; get a clean charger first.
3. Run the full standalone test: `python Tools/axxpd_selftest_full.py` (no load connected).
   It has a colour-coded PDF-friendly report with measured voltages.
4. If all pass: push `main`, and send the test report as a **PDF** via the AxxPdfTelegram
   tool at `C:\Users\Axel Johansson\Nextcloud\2_Project\00_AxxProjects\AxxPdfTelegram`.
5. If it fault-loops on a clean charger: the layout trigger is real — get a hardware
   watchpoint working (this bench's ST-Link gdbserver drops on the 16→128 MHz boot;
   DebugMonitor is masked by `C_DEBUGEN`). Consider a PD analyzer to capture the
   triggering message, or widen the suspect buffers for non-zero margin.

Full detail: `Documentation/HANDOVER_2026-08-01.md`.

---

## Build

Toolchain is the STM32CubeIDE 1.14.0 bundle (not on PATH by default):
```
GCC:  C:\ST\STM32CubeIDE_1.14.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin
MAKE: C:\ST\STM32CubeIDE_1.14.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin
```
Build (add both to PATH first): `cd Firmware && make -j8 all` → `build/AxxPD.elf`.
For a `.hex`: `arm-none-eabi-objcopy -O ihex build/AxxPD.elf build/AxxPD.hex`.
Keep CubeIDE working — do NOT replace the Makefile/IDE setup.

## Flash (SWD via ST-Link)
```
CLI: C:\ST\STM32CubeIDE_1.14.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe
STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst -d build/AxxPD.hex -v --start
```
- **Erase fails on the 1st attempt, succeeds on retry** — loop the command 2–3×.
- `mode=UR` (connect-under-reset) is required. Read regs with `-r32 <addr> <bytes>` /
  `-coreReg PC`. Reset flags at RCC_CSR `0x40021094`.
- **Flash hazard:** halting a live PD sink at EPR/high voltage makes the charger latch VBUS
  off → brownout mid-write → brick. Only flash at a 5 V SPR / output-off state, or with
  external 3.3 V. Never flash while in EPR mode.

## Device
- AxxPD CDC serial: VID:PID `0483:5740` (typically COM15). Detect by VID:PID, not
  manufacturer string (Windows usbser reports "Microsoft").
- ST-Link: VID:PID `0483:3754` (COM3).

## Git conventions (MANDATORY)
- Commit author MUST be **`axel.johansson10@gmail.com`** (name "Axel Johansson").
  Use `git -c user.name="Axel Johansson" -c user.email="axel.johansson10@gmail.com"
  commit --author="Axel Johansson <axel.johansson10@gmail.com>" ...`.
- **Do NOT mention Claude anywhere in git commits** (overrides any default trailer).
- **Short commit messages.**
- Push only when the user asks. `origin/main` currently must stay a fast-forward.

## Layout
- `Firmware/` — CubeIDE project. `Core/Src` app; `axxpd_firmware/` PD glue + CLI;
  `pdsink/` vendored MIT PD stack (afsm FSM, ETL containers); `etl/` vendored ETL.
- `Tools/` — `axxpd_selftest_full.py` (full test), `charger_test.py`, `AxxPD_Dashboard.html`.
- `Documentation/` — command reference, schematic, handover docs.
- Private dev repo `AxxPD_dev/` is separate/gitignored — do NOT touch from here.
