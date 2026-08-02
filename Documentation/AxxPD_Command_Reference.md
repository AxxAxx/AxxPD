# AxxPD Command Reference

AxxPD provides a dual-layer command interface over USB CDC serial (115200 baud). Commands can be sent from any serial terminal, the built-in WebSerial dashboard (`Tools/AxxPD_Dashboard.html`), or programmatically via SCPI.

**Connection:** USB-C data port. The serial terminal must be enabled in Settings > Mode > Serial terminal (default: ON).

**DTR required:** AxxPD only emits CDC output while the host has the port open with DTR asserted. Serial libraries (pyserial) and terminal programs assert DTR on open by default; if you see no responses or events, check that your client raises DTR.

**Conventions:**
- Commands are case-insensitive
- Numeric arguments accept V/mV/A/mA suffixes (e.g. `setpps 7500mV 2A`)
- Multiple commands can be chained with `;` (e.g. `mode avs; set 24V`)
- Lines prefixed with `#` are asynchronous events (not command responses)
- SCPI commands start with `*` or `:`; everything else is a shortcut command

## Table of Contents

- [Quick Start](#quick-start)
- [Shortcut Commands](#shortcut-commands)
  - [State Inspection](#state-inspection)
  - [Voltage Control](#voltage-control)
  - [Mode Control](#mode-control)
  - [Output Control](#output-control)
  - [Protection](#protection)
  - [Sequences](#sequences)
  - [Data Streaming](#data-streaming)
  - [Diagnostics](#diagnostics)
- [SCPI Commands](#scpi-commands)
  - [IEEE 488.2 Common Commands](#ieee-4882-common-commands)
  - [:OUTPut](#output)
  - [:SOURce](#source)
  - [:MEASure](#measure)
  - [:PD](#pd)
  - [:SYSTem](#system)
  - [:CONFigure](#configure)
- [Asynchronous Events](#asynchronous-events)
- [Data Stream Format](#data-stream-format)
- [Scripting Examples](#scripting-examples)

---

## Quick Start

A typical interactive session:

```
list                    # see what PDOs the charger offers
setpdo 3                # select 15V fixed PDO
on                      # enable output
meas                    # read voltage, current, temperatures
setpps 9V 3A            # switch to PPS 9V / 3A
epr                     # enter EPR mode (list expands to include 28-48V)
setavs 24V              # select EPR AVS 24V
rst                     # renegotiate back to 5V
off                     # disable output
```

---

## Shortcut Commands

Short, human-friendly commands for interactive use. These are the primary interface.

### State Inspection

| Command | Description | Example Response |
|---------|-------------|------------------|
| `list` (alias `pdos`) | List available source PDOs (SPR only until EPR entered) | `1,FIXED,5.000,3.000,SPR` |
| `list all` | List all PDOs including EPR (auto-enters EPR if capable) | Includes PDO8+ |
| `ct` (alias `contract`) | Show the active contract | `PDO4,FIXED,20.000V,3.000A,SPR` |
| `meas` | Measured V, I, die/NTC temperatures + I2C diagnostics | `V=20.012 I=0.000 Tdie=32.5C Tntc=28.3C i2c=0 s=32 e=0x0` |
| `fault` (alias `faults`) | Show fault status (same as `protect status`) | `fault=0 src=0` |
| `help` / `h` / `?` | Print built-in command reference | |

**`meas` format:** `V=%.3f I=%.3f Tdie=%.1f<u> Tntc=%.1f<u> i2c=%d s=%lu e=0x%lX` where `<u>` is `C` or `F` per the temperature-unit setting, `i2c` is the HAL status of the INA228 read, and `s`/`e` are the I2C peripheral state and error code. Note that `meas` does **not** report power, Wh or Ah -- use `:MEAS:ALL?` (CSV incl. W/Wh/Ah) or `stream on` for those.

### Voltage Control

| Command | Description | Notes |
|---------|-------------|-------|
| `setpdo <N>` | Request fixed PDO at slot N (1-11) | N >= 8 auto-enters EPR |
| `setpps <V> [<I>]` | SPR PPS at V volts, optional current limit | 20 mV voltage step, 50 mA current step |
| `setavs <V>` | EPR AVS at V volts | 100 mV effective step. Requires `epr` first |
| `set <V> [<I>]` | Auto-select best PDO type for voltage V | Filtered by current `mode` setting. Auto-enters EPR when V > 21 V and the source is EPR-capable |
| `mode AUTO\|FIX\|PPS\|AVS` | Set PDO type filter for `set` command | Default: AUTO |

**Voltage examples:**
```
setpdo 1                # 5V fixed
setpdo 4                # 20V fixed
setpps 7.5V 2A          # PPS 7.5V / 2A
setpps 11V              # PPS 11V (keeps current unchanged)
setavs 24V              # EPR AVS 24V
setavs 27.5V            # EPR AVS 27.5V
set 12V                 # auto-select: finds best PDO for 12V
set 9V 2A               # auto-select with current limit
```

### Mode Control

| Command | Description | Notes |
|---------|-------------|-------|
| `epr` | Enter EPR mode | Resets target to 5V; use `setpdo`/`setavs` next |
| `spr` | Leave EPR mode | PDO list shrinks back to 1-7 |
| `rst` | Renegotiate to 5V PDO1 | Also disables the output. Stays in EPR if already entered. Not a hard reset -- unplug for full link reset |

### Output Control

| Command | Description |
|---------|-------------|
| `on` | Enable output (LTC4368 SHDN HIGH). Blocked during the 1.5 s toggle cooldown or thermal cooldown |
| `off` | Disable output (LTC4368 SHDN LOW) |
| `lock` | Lock UI buttons |
| `unlock` | Unlock UI buttons |
| `clear` | Clear active fault and reset fault state |

### Protection

| Command | Description |
|---------|-------------|
| `protect` or `protect status` | Show current fault state and source (`fault=<n> src=<n> [NAME]`) |
| `protect ocp <A>` | Set over-current protection threshold (0.1-6 A, default 5.5 A) |
| `protect ovp <V>` | Set over-voltage protection threshold (5-55 V, default 55 V) |
| `protect clear` | Clear fault latch |

`prot` is accepted for `protect`; `stat` and `clr` are accepted for the `status` / `clear` sub-commands. Thresholds persist to flash settings; `:CONF:OCP?` / `:CONF:OVP?` query the stored values.

### Sequences

Programmable voltage sequences with configurable step times.

| Command | Description |
|---------|-------------|
| `seq add <V> <t_ms>` | Add a voltage step (voltage in V, dwell time in ms). Legacy 3-arg form: `seq add <mV> <mA> <t_ms>` |
| `seq clear` | Clear all sequence steps (rejected while running) |
| `seq list` | List current sequence steps |
| `seq run` | Execute the voltage sequence (non-blocking; prints a measurement after each step) |
| `seq stop` | Stop a running sequence |

Maximum 16 steps; voltage must be 3.3-48 V and dwell time > 0 ms.

**Sequence example:**
```
seq clear
seq add 5V 2000         # 5V for 2 seconds
seq add 12V 3000        # 12V for 3 seconds
seq add 20V 5000        # 20V for 5 seconds
seq run                 # execute the sequence
```

### Data Streaming

| Command | Description |
|---------|-------------|
| `stream on [rate_hz]` | Enable CSV data streaming (default 20 Hz, 1-1000 Hz, e.g. `stream on 100`) |
| `stream off` | Disable data streaming |
| `stream` | Toggle streaming at the current rate |

When streaming is enabled, lines prefixed with `#S` are emitted at the configured rate. See [Data Stream Format](#data-stream-format) for the field layout.

### Diagnostics

| Command | Description |
|---------|-------------|
| `selftest` (alias `test`) | Walk all advertised PDOs and report pass/fail per step |
| `reboot` | Full MCU reset (NVIC) |
| `fwup` (aliases `fwupd`, `bootloader`) | Step down to 5 V, then enter the custom AxxPD bootloader for a USB firmware update via the web dashboard. Replies `+FWUP` and resets, or `-FWUP <reason>` if it cannot reach a 5 V contract |
| `dfu` | Jump to the STM32 ROM (system memory) DFU bootloader for recovery via USB DFU / USART1. Power cycle or reset to return to AxxPD |
| `trace on\|off` | Enable/disable diagnostic prints ([CC]/[RX]/[TX]/[UCPD] + PE state trace / `#EVT PE_STATE`). `trace` alone queries the state |
| `stat` | Dump UCPD ISR counters (TX ok/fail/requests, RX msgend/err/ovr/etc.) |

**Hardware test commands** (bench/bring-up diagnostics, no arguments):

| Command | Description |
|---------|-------------|
| `gpio` | Read the four button GPIO pins once |
| `btntest` (alias `btnt`) | Poll raw button state for 5 seconds |
| `i2cscan` | Scan the I2C3 bus and list responding addresses |
| `ina228diag` (alias `ina228`) | Reset I2C3, probe the INA228 and dump its registers |
| `lcdtest` (alias `lcdt`) | Low-level SPI display test (draws red pixels) |
| `colortest` | Draw labeled color swatches (blocks until a button is pressed) |
| `swaptest` | LCD byte-order test, Fill vs DrawPixel (blocks until a button is pressed) |
| `buzzsweep` | Sweep the buzzer 800-5000 Hz |

**Selftest:** Walks every advertised PDO: one step per Fixed PDO, min/mid/max for each PPS and AVS APDO, plus 5 random voltage steps. Auto-enters EPR if the source supports it. Takes roughly 20-60 seconds. `selftest` first prints a safety warning and waits for a confirmation line — type `OK` to start (anything else aborts). Do not have a load on VBUS during the test (the output voltage jumps between all PDO voltages). The output bleed resistor is engaged during the run so the reported measurements track VBUS on downward steps. Responds with PASS/FAIL per step (judged on the negotiated contract) and prints the measured voltage for each.

**Host-side test tools:** for automated bench/CI validation over this command interface (no load required), the repository ships `Tools/charger_test.py` (walks every PDO and checks voltage tolerance) and `Tools/axxpd_selftest_full.py` (a comprehensive standalone feature test: SCPI plumbing, capability discovery, the full voltage sweep with per-step measured voltages, measurements, output control, protection configuration, EPR mode, the fault log and the telemetry stream). Both write a timestamped report and exit non-zero on failure. See the README "Testing" section.

---

## SCPI Commands

Standard Commands for Programmable Instruments. Use these for automated scripting. SCPI commands accept both short and long forms (the uppercase letters in the canonical spelling are the short form).

### IEEE 488.2 Common Commands

| Command | Description |
|---------|-------------|
| `*IDN?` | Returns identification string: `AxxPD,USBPD-Sink,<serial>,<version>` -- the serial field is the MCU's 96-bit unique device ID as 24 hex digits |
| `*RST` | Reset to 5V PDO1 and disable output (equivalent to `rst`) |
| `*CLS` | Clear error queue and status |
| `*OPC?` | Returns `1` when the last voltage request has settled, `0` if still pending |
| `*WAI` | Block until operation complete; prints `OK`, or `TIMEOUT` after 5 seconds |

### :OUTPut

| Command | Description |
|---------|-------------|
| `:OUTP ON` or `:OUTP 1` | Enable output |
| `:OUTP OFF` or `:OUTP 0` | Disable output |
| `:OUTP?` | Query output state (returns `1` or `0`) |

### :SOURce

| Command | Description |
|---------|-------------|
| `:SOUR:VOLT <value>` | Set target voltage (3.3-48 V) |
| `:SOUR:VOLT?` | Query target voltage |
| `:SOUR:CURR <value>` | Set target current limit (0-6 A) |
| `:SOUR:CURR?` | Query target current |
| `:SOUR:MODE <mode>` | Set PDO type filter (AUTO/FIX/PPS/AVS) |
| `:SOUR:MODE?` | Query current mode |
| `:SOUR:APPL` | Apply pending voltage/current changes |

### :MEASure

| Command | Description | Response Format |
|---------|-------------|-----------------|
| `:MEAS:VOLT?` | Measured VBUS voltage | Volts, `%.3f` |
| `:MEAS:CURR?` | Measured current | Amps, `%.3f` |
| `:MEAS:ALL?` | All measurements | Unlabeled CSV: `<V>,<I>,<W>,<Wh>,<Ah>,<Tdie>,<Tntc>` (e.g. `20.012,1.503,30.078,0.1234,0.0062,32.5,28.3`) |
| `:MEAS:POW?` | Measured power | Watts, `%.3f` |
| `:MEAS:TEMP?` | INA228 die and board NTC temperatures | CSV: `<Tdie>,<Tntc>` (`%.1f`, unit per temp setting) |
| `:MEAS:ENER?` | Energy accumulators | CSV: `<Wh>,<Ah>` (`%.4f`) |
| `:MEAS:ENER:RES` | Reset energy counters (also resets the Energy screen's session runtime and peak stats) | `Energy counters reset` |

### :PD

| Command | Description |
|---------|-------------|
| `:PD:MODE EPR` | Enter EPR mode |
| `:PD:MODE SPR` | Leave EPR mode |
| `:PD:MODE?` | Query current PD mode (returns `EPR` or `SPR`) |
| `:PD:CONTR?` | Query active contract |
| `:PD:PDO?` | List available PDOs |
| `:PD:PDO? ALL` | List all PDOs including EPR (auto-enters EPR; `EPR` is accepted as a synonym for `ALL`) |
| `:PD:PDO:COUN?` | Query PDO count |
| `:PD:PDO:LIST?` | List PDOs (explicit; also accepts `ALL`/`EPR`) |
| `:PD:PDO<n>?` | Query single PDO by index |

### :SYSTem

| Command | Description |
|---------|-------------|
| `:SYST:HELP?` | Print built-in command reference |
| `:SYST:ERR?` | Pop one entry from the SCPI error queue (`<code>,"<message>"`, or `0,"No error"`) |
| `:SYST:EVEN ON\|OFF` | Enable/disable unsolicited `#EVT` event notifications (default ON) |
| `:SYST:EVEN?` | Query whether event notifications are enabled (`ON`/`OFF`) |
| `:SYST:TRAC ON\|OFF` | Enable/disable diagnostic trace (query with `:SYST:TRAC?`) |
| `:SYST:REB` | Reboot MCU |
| `:SYST:FWUP` | Enter the custom AxxPD bootloader (web dashboard update) |
| `:SYST:DFU` | Enter the STM32 ROM DFU bootloader (recovery) |
| `:SYST:TEST` | Run self-test (requires the same `OK` confirmation as `selftest`) |
| `:SYST:LOCK ON\|OFF` | Lock/unlock UI |
| `:SYST:LOCK?` | Query lock state (returns `1` or `0`) |

### :CONFigure

| Command | Description |
|---------|-------------|
| `:CONF:OCP <value>` | Set OCP threshold (accepts A/mA suffix; stored in 100 mA steps, clamped 100-6000 mA; default 5500 mA) |
| `:CONF:OCP?` | Query OCP threshold -- returns **milliamps** (e.g. `5500`) |
| `:CONF:OVP <value>` | Set OVP threshold (accepts V/mV suffix; stored in 1 V steps, clamped 5000-55000 mV; default 55000 mV) |
| `:CONF:OVP?` | Query OVP threshold -- returns **millivolts** (e.g. `55000`) |
| `:CONF:OPP?` | Query OPP threshold (100 mW units; query only) |
| `:CONF:WH?` | Query energy limit (mWh; query only) |
| `:CONF:AH?` | Query charge limit (mAh; query only) |
| `:CONF:CC?` | Query charge-complete detection (`<mA threshold>,<seconds>`; query only) |

---

## Asynchronous Events

Events are emitted automatically when state changes. All event lines are prefixed with `#EVT`.

| Event | Description | Example |
|-------|-------------|---------|
| `#EVT PDO_COUNT <n>` | Number of available PDOs changed | `#EVT PDO_COUNT 7` |
| `#EVT PDO <line>` | PDO details (emitted once on first cap reception) | `#EVT PDO 1,FIXED,5.000,3.000,SPR` |
| `#EVT EPR_CAPABLE <0\|1>` | Source EPR capability | `#EVT EPR_CAPABLE 1` |
| `#EVT PD_MODE <EPR\|SPR>` | PD mode changed | `#EVT PD_MODE EPR` |
| `#EVT CONTRACT <info>` | Active contract changed | `#EVT CONTRACT PDO4,FIXED,20.000V,3.000A,SPR` |
| `#EVT CONTRACT NONE` | No active contract | |
| `#EVT PE_STATE <id>` | PE state machine transition (trace mode only) | `#EVT PE_STATE 12` |

---

## Data Stream Format

When streaming is enabled (`stream on`), lines are emitted at 20 Hz with the prefix `#S`:

```
#S <V>,<I>,<W>,<Wh>,<Ah>,<Tdie>,<Tntc>,<output>
```

| Field | Description | Unit | Format |
|-------|-------------|------|--------|
| V | VBUS voltage | Volts | `%.3f` |
| I | Load current | Amps | `%.3f` |
| W | Power | Watts | `%.3f` |
| Wh | Accumulated energy | Watt-hours | `%.4f` |
| Ah | Accumulated charge | Amp-hours | `%.4f` |
| Tdie | INA228 die temperature | deg C | `%.1f` |
| Tntc | Board NTC temperature | deg C | `%.1f` |
| output | Output state | 0 or 1 | `%u` |

**Example:**
```
#S 20.012,1.503,30.078,0.1234,0.0062,32.5,28.3,1
```

---

## Scripting Examples

**Python -- read voltage and current:**
```python
import serial
ser = serial.Serial('COM3', 115200, timeout=1)
ser.write(b'meas\r\n')
print(ser.readline().decode())
```

**Python -- set voltage and wait for settle:**
```python
ser.write(b'set 12V\r\n')
ser.write(b'*WAI\r\n')
response = ser.readline().decode().strip()
assert response == 'OK'
```

**Python -- record 10 seconds of data:**
```python
ser.write(b'stream on\r\n')
import time
t0 = time.time()
while time.time() - t0 < 10:
    line = ser.readline().decode().strip()
    if line.startswith('#S '):
        fields = line[3:].split(',')
        v, i, w = float(fields[0]), float(fields[1]), float(fields[2])
        print(f'{v:.3f}V  {i:.3f}A  {w:.2f}W')
ser.write(b'stream off\r\n')
```

**Command chaining:**
```
mode pps; set 9V 2A; *WAI; meas
```
