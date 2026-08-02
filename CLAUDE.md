# AxxPD — Project Guide & Handover

USB-C PD 3.1 EPR bench power controller. STM32G491CCU6 (Cortex-M4F, 128 MHz).
Firmware is **bootloader-less**: app linked at `0x08000000`. Firmware update = hold a
button at power-on to enter the STM32 ROM DFU, flash over USB (or SWD).

---

## CURRENT STATUS — read this first (2026-08-02)

**Firmware verified good: the full standalone self-test passes 53/53** on hardware —
all voltages (5/9/15/20/28 V EPR + PPS + AVS ranges), EPR mode entry, measurements,
output on/off, protection config, and telemetry stream. **PD/EPR negotiation works.**
Commits are on local `main`, ready to push. `origin/main` (`357ae83`) is the prior baseline.

### The "unfindable layout-sensitive bug" was a BUILD-SYSTEM artifact — RESOLVED
For days the new firmware appeared to fault-loop on boot, and "any code change (even a
no-op line) unmasked it." **Root cause: the Makefile did not track header dependencies**,
so editing a header (e.g. `afsm.h`) left stale `.o` files and produced inconsistent
binaries that crashed. Every "bisect contradiction", apparent "layout sensitivity", and
the phantom "afsm dispatch corruption" was stale incremental builds. **A clean rebuild of
the real fixes boots and passes 53/53.** Fixed at the source: the `Makefile` now uses
`-MMD -MP` + `-include *.d` (`d63c723`). **If you ever suspect a stale build, `make clean`.**

Follow-ups from the hunt:
- The `afsm` dispatch guard added mid-hunt was a **false positive** (it required
  `interceptor_table` to be in flash, but that table is a *runtime-initialized static in
  RAM*) — reverted (`86d333a`).
- The real fixes (`92cf4a9`..`494a118`: source_caps clamp, timer/resize bounds guards,
  fault-handler safe-off + fast reset, contract-relative sw OVP, docs, host tools) are all
  good and verified on hardware.

The field symptom "sometimes I must restart the charger" is a SEPARATE real issue — a
crash/latch interaction analysed in §5b/§6 of the handover; the charger-hang hardening
(fast reset + output-off fault handlers + boot loop-breakers, and the message-size bounds
guards as defense-in-depth) is in place.

### NEXT STEPS
1. `cd Firmware && make -j8 all` (clean incremental builds are safe now); flash (see Flash
   below); confirm COM15 enumerates and `python Tools/axxpd_selftest_full.py` -> 53/53.
2. Push `main`; send the PDF report via the AxxPdfTelegram tool at
   `C:\Users\Axel Johansson\Nextcloud\2_Project\00_AxxProjects\AxxPdfTelegram`.

Full detail: `Documentation/HANDOVER_2026-08-01.md`.

---

## Build

Toolchain is the STM32CubeIDE 1.14.0 bundle (not on PATH by default):
```
GCC:  C:\ST\STM32CubeIDE_1.14.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin
MAKE: C:\ST\STM32CubeIDE_1.14.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin
```
Build (add both to PATH first): `cd Firmware && make -j8 all` -> `build/AxxPD.elf`.
For a `.hex`: `arm-none-eabi-objcopy -O ihex build/AxxPD.elf build/AxxPD.hex`.
The Makefile now tracks header deps (`-MMD`), so incremental builds are reliable — but a
`make clean` is always the safe fallback. Keep CubeIDE working — do NOT replace the setup.

## Flash (SWD via ST-Link)
```
CLI: C:\ST\STM32CubeIDE_1.14.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506\tools\bin\STM32_Programmer_CLI.exe
STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst -d build/AxxPD.hex -v --start
```
- **Erase fails on the 1st attempt, succeeds on retry** — loop the command 2–3×.
- `mode=UR` (connect-under-reset) is required. Read regs with `-r32 <addr> <bytes>` /
  `-coreReg PC`. Reset flags at RCC_CSR `0x40021094`.
- **Flash hazard:** halting a live PD sink at EPR/high voltage makes the charger latch VBUS
  off -> brownout mid-write -> brick. Only flash at a 5 V SPR / output-off state, or with
  external 3.3 V. Never flash while in EPR mode. (With external 3.3 V, SWD halt/debug is
  fully safe — that is how the 53/53 test build was verified.)

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
