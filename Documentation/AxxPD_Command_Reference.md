# AxxPD Command Reference

AxxPD provides a dual-layer command interface over USB CDC serial (115200 baud). Commands can be sent from any serial terminal, the built-in WebSerial dashboard (`Tools/dashboard.html`), or programmatically via SCPI.

**Connection:** USB-C data port. The serial terminal must be enabled in Settings > System > Serial Terminal (default: ON).

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
meas                    # read voltage, current, power, temperature
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
| `list` | List available source PDOs (SPR only until EPR entered) | `1,FIXED,5.000V,3.000A,SPR` |
| `list all` | List all PDOs including EPR (auto-enters EPR if capable) | Includes PDO8+ |
| `ct` | Show the active contract | `PDO4,FIXED,20.000V,3.000A,SPR` |
| `meas` | Measured V, I, W, Wh, Ah, temperatures | `V=20.012 I=0.000 W=0.000 ...` |
| `help` or `?` | Print built-in command reference | |

### Voltage Control

| Command | Description | Notes |
|---------|-------------|-------|
| `setpdo <N>` | Request fixed PDO at slot N (1-11) | N >= 8 auto-enters EPR |
| `setpps <V> [<I>]` | SPR PPS at V volts, optional current limit | 20 mV voltage step, 50 mA current step |
| `setavs <V>` | EPR AVS at V volts | 100 mV effective step. Requires `epr` first |
| `set <V> [<I>]` | Auto-select best PDO type for voltage V | Filtered by current `mode` setting |
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
| `rst` | Renegotiate to 5V PDO1 | Stays in EPR if already entered. Not a hard reset -- unplug for full link reset |

### Output Control

| Command | Description |
|---------|-------------|
| `on` | Enable output (LTC4368 SHDN HIGH) |
| `off` | Disable output (LTC4368 SHDN LOW) |
| `lock` | Lock UI buttons |
| `unlock` | Unlock UI buttons |
| `clear` | Clear active fault and reset fault state |

### Protection

| Command | Description |
|---------|-------------|
| `protect` or `protect status` | Show current protection status and thresholds |
| `protect ocp <A>` | Set over-current protection threshold |
| `protect ovp <V>` | Set over-voltage protection threshold |
| `protect clear` | Clear fault latch |

### Sequences

Programmable voltage sequences with configurable step times.

| Command | Description |
|---------|-------------|
| `seq add <V> <t_ms>` | Add a voltage step (voltage in V, dwell time in ms) |
| `seq clear` | Clear all sequence steps |
| `seq list` | List current sequence steps |
| `seq run` | Execute the voltage sequence |
| `seq stop` | Stop a running sequence |

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
| `stream on` | Enable 20 Hz CSV data streaming |
| `stream off` | Disable data streaming |
| `stream` | Toggle streaming |

When streaming is enabled, lines prefixed with `#S` are emitted at 20 Hz. See [Data Stream Format](#data-stream-format) for the field layout.

### Diagnostics

| Command | Description |
|---------|-------------|
| `selftest` | Walk all advertised PDOs and report pass/fail per step |
| `reboot` | Full MCU reset (NVIC) |
| `dfu` | Enter STM32 DFU bootloader for firmware update |
| `trace on\|off` | Enable/disable diagnostic prints ([CC]/[RX]/[TX] + PE state) |

**Selftest:** Walks every advertised PDO: one step per Fixed PDO, min/mid/max for each PPS and AVS APDO. Auto-enters EPR if the source supports it. Takes 10-30 seconds. Do not have a load on VBUS during the test (the output voltage jumps between all PDO voltages). Responds with PASS/FAIL per step.

---

## SCPI Commands

Standard Commands for Programmable Instruments. Use these for automated scripting. SCPI commands accept both short and long forms (the uppercase letters in the canonical spelling are the short form).

### IEEE 488.2 Common Commands

| Command | Description |
|---------|-------------|
| `*IDN?` | Returns identification string: `AxxPD,USBPD-Sink,0,<version>` |
| `*RST` | Reset to 5V PDO1 (equivalent to `rst`) |
| `*CLS` | Clear error queue and status |
| `*OPC?` | Returns `1` when the last voltage request has settled, `0` if still pending |
| `*WAI` | Block until operation complete (5 second timeout) |

### :OUTPut

| Command | Description |
|---------|-------------|
| `:OUTP ON` or `:OUTP 1` | Enable output |
| `:OUTP OFF` or `:OUTP 0` | Disable output |
| `:OUTP?` | Query output state (returns `1` or `0`) |

### :SOURce

| Command | Description |
|---------|-------------|
| `:SOUR:VOLT <value>` | Set target voltage |
| `:SOUR:VOLT?` | Query target voltage |
| `:SOUR:CURR <value>` | Set target current limit |
| `:SOUR:CURR?` | Query target current |
| `:SOUR:MODE <mode>` | Set PDO type filter (AUTO/FIX/PPS/AVS) |
| `:SOUR:MODE?` | Query current mode |
| `:SOUR:APPL` | Apply pending voltage/current changes |

### :MEASure

| Command | Description | Response Format |
|---------|-------------|-----------------|
| `:MEAS:VOLT?` | Measured VBUS voltage | Volts |
| `:MEAS:CURR?` | Measured current | Amps |
| `:MEAS:ALL?` | All measurements | `V=x.xxx I=x.xxx Tdie=x.x Tntc=x.x` |
| `:MEAS:POW?` | Measured power | Watts |
| `:MEAS:TEMP?` | INA228 die and board temperatures | |
| `:MEAS:ENER?` | Energy accumulators | Wh and Ah |
| `:MEAS:ENER:RES` | Reset energy counters | |

### :PD

| Command | Description |
|---------|-------------|
| `:PD:MODE EPR` | Enter EPR mode |
| `:PD:MODE SPR` | Leave EPR mode |
| `:PD:MODE?` | Query current PD mode (returns `EPR` or `SPR`) |
| `:PD:CONTR?` | Query active contract |
| `:PD:PDO?` | List available PDOs |
| `:PD:PDO? ALL` | List all PDOs including EPR |
| `:PD:PDO:COUN?` | Query PDO count |
| `:PD:PDO:LIST?` | List PDOs (explicit) |
| `:PD:PDO<n>?` | Query single PDO by index |

### :SYSTem

| Command | Description |
|---------|-------------|
| `:SYST:HELP?` | Print built-in command reference |
| `:SYST:ERR?` | Query SCPI error queue |
| `:SYST:EVEN?` | Query event log |
| `:SYST:TRAC ON\|OFF` | Enable/disable diagnostic trace |
| `:SYST:REB` | Reboot MCU |
| `:SYST:DFU` | Enter DFU bootloader |
| `:SYST:TEST` | Run self-test |
| `:SYST:LOCK ON\|OFF` | Lock/unlock UI |
| `:SYST:LOCK?` | Query lock state |

### :CONFigure

| Command | Description |
|---------|-------------|
| `:CONF:OCP <value>` | Set OCP threshold (mA) |
| `:CONF:OCP?` | Query OCP threshold |
| `:CONF:OVP <value>` | Set OVP threshold (mV) |
| `:CONF:OVP?` | Query OVP threshold |
| `:CONF:OPP?` | Query OPP threshold (100 mW units) |
| `:CONF:WH?` | Query energy limit (mWh) |
| `:CONF:AH?` | Query charge limit (mAh) |
| `:CONF:CC?` | Query charge-complete detection (mA threshold, seconds) |

---

## Asynchronous Events

Events are emitted automatically when state changes. All event lines are prefixed with `#EVT`.

| Event | Description | Example |
|-------|-------------|---------|
| `#EVT PDO_COUNT <n>` | Number of available PDOs changed | `#EVT PDO_COUNT 7` |
| `#EVT PDO <line>` | PDO details (emitted once on first cap reception) | `#EVT PDO 1,FIXED,5.000V,3.000A` |
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
