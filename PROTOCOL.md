# Rover Jetson ↔ Pico Communication Protocol (v0.1 — early version)

**Task 3: Jetson and Microcontroller communication**
**Scope:** Onboard computer (Jetson Orin Nano) ↔ embedded controller (Raspberry Pi Pico), single async serial link (UART, 115200 baud, 8N1).

This is a first, working version of the link: reliable framing, a CRC, and
timeout/fail-safe handling, with the required CONTROL and TELEMETRY fields.
It is deliberately hardware-agnostic — every value, scaling constant, and
field width is a placeholder that can be tightened once final motors,
encoders, and sensors are chosen. Those placeholders are called out
explicitly in section 4 so nothing here is mistaken for a finalized spec.

---

## 1. Design goals

- **No hardware dependency.** The format, parser, and demo all run without
  the final Jetson or Pico in hand — proven in section 7 using a software
  loopback instead of a physical cable.
- **Reliable framing** over a byte stream that gives no message boundaries
  for free.
- **Corruption detection** via CRC, with a defined recovery behavior rather
  than "hope it doesn't happen."
- **Fail-safe on silence.** If the Jetson goes quiet, the rover stops. This
  is the single most important property of the whole protocol.
- **Room to grow.** Adding a new message type or field doesn't require
  changing the framing.

## 2. Link basics

| Parameter | Value |
|---|---|
| Physical layer | UART, 115200 baud, 8 data bits, no parity, 1 stop bit |
| Direction | Full duplex, asynchronous — each side sends on its own timer |
| CONTROL rate | 20 Hz (Jetson → Pico) |
| TELEMETRY rate | 20 Hz (Pico → Jetson) |
| Byte order | Little-endian for all multi-byte fields |

20 Hz was picked as a reasonable starting control-loop rate for a
teleoperated/early-autonomy rover — easy to change later (see section 8).

## 3. Frame format

```
 byte:   0        1        2        3 .. 3+LEN-1      3+LEN .. 3+LEN+1
       +--------+--------+--------+ ~~~~~~~~~~~~~~~ +-------------------+
       | START  | MSG_ID |  LEN   |     PAYLOAD     |   CRC16 (LE)      |
       | 0xAA   | 1 byte | 1 byte |    LEN bytes    |     2 bytes       |
       +--------+--------+--------+ ~~~~~~~~~~~~~~~ +-------------------+
```

- **START (0xAA)** — marks a possible frame start. Not escaped/reserved
  elsewhere in the frame (see the resync rule below for why that's safe).
- **MSG_ID** — identifies which message this is (table in section 4).
- **LEN** — payload length in bytes (0–255).
- **PAYLOAD** — the message-specific fields, packed with `struct` (see
  `framing.py` / `pico_main.py`).
- **CRC16** — CRC-16/CCITT (XModem: poly `0x1021`, init `0xFFFF`, no
  reflection), computed over `MSG_ID + LEN + PAYLOAD` (not the START byte,
  not the CRC field itself). Little-endian on the wire.

No byte-stuffing/escaping is used. Instead, the receiver trusts `LEN` to
know exactly how many bytes make up the frame, then verifies the whole
thing with the CRC before accepting it. This keeps both implementations
simple (a real concern for a first firmware project) while still being
robust — see section 5 for how corruption and false starts are handled.

## 4. Messages

### 4.1 CONTROL (Jetson → Pico), `MSG_ID = 0x01`, 6-byte payload

| Field | Type | Range | Meaning |
|---|---|---|---|
| `drive_cmd` | int16 | -1000..1000 | Desired drive command, tenths of a percent (±100.0%) of full drive effort |
| `steer_cmd` | int16 | -1000..1000 | Desired steering command, tenths of a percent of full steering range |
| `mode` | uint8 | 0/1/2 | Operating mode: `0=DISABLED, 1=MANUAL, 2=AUTONOMOUS` |
| `stop` | uint8 | 0/1 | Explicit stop command. `1` forces an immediate stop regardless of `mode` |

`stop` is deliberately a separate field from `mode` rather than a third
mode value, so an emergency stop can be asserted (and, just as important,
*released*) without needing a mode-change round trip, and so it's
continuously refreshed alongside the rest of CONTROL rather than being a
one-shot event that could be missed.

### 4.2 TELEMETRY (Pico → Jetson), `MSG_ID = 0x02`, 15-byte payload

| Field | Type | Range | Meaning |
|---|---|---|---|
| `enc_left` | int32 | full range | Left-side cumulative encoder ticks |
| `enc_right` | int32 | full range | Right-side cumulative encoder ticks |
| `steer_fb` | int16 | -1000..1000 | Measured steering position, same scale as `steer_cmd` |
| `current_ma` | uint16 | 0..65535 | Motor current draw, milliamps |
| `fault_status` | uint8 | bitmask | See table below |
| `cmd_age_ms` | uint16 | 0..65535 | Milliseconds since the last **valid** CONTROL frame was received (`0xFFFF` = none ever received / saturated) |

**Fault bitmask (`fault_status`):**

| Bit | Name | Meaning |
|---|---|---|
| 0x01 | `COMM_TIMEOUT` | Watchdog expired — no valid CONTROL frame within the timeout |
| 0x02 | `OVER_CURRENT` | Current reading exceeded a threshold |
| 0x04 | `ESTOP_ACTIVE` | Hardware or software E-stop engaged |
| 0x08 | `ENCODER_FAULT` | Encoder reading invalid or stalled |
| 0x10 | `UNDERVOLTAGE` | Supply voltage low (placeholder, not wired up yet) |
| 0x20–0x80 | *reserved* | Free for future use |

`cmd_age_ms` is the field the task calls out explicitly, and it's the
mechanism the whole fail-safe design hangs off: the Jetson can see, frame
by frame, how stale its own commands look to the Pico — which is a much
more direct link-health signal than just noticing telemetry stopped
arriving.

## 5. Framing robustness: corruption and resync

Every candidate frame is CRC-checked before being accepted. On a mismatch
— whether from genuine line noise/corruption, or from a data byte that
happened to equal `0xAA` and caused a false-positive frame start — the
parser drops **just the leading START byte** and resumes scanning for the
next `0xAA`, rather than discarding a whole guessed-length block or giving
up. This makes the parser self-resynchronizing: one bad frame doesn't
bring down the link, and the very next valid frame after it is parsed
normally. This is proven directly in the demo (section 7, Part A).

A **stale partial frame** — bytes started arriving but the rest never
showed up — is cleared after `INTER_BYTE_TIMEOUT_MS` (50 ms) of silence,
so a truncated transmission can't wedge the parser waiting forever for
bytes that aren't coming.

## 6. Timeout / watchdog behavior

Two independent timeouts exist, at two different layers:

1. **Frame-level (parser):** the 50 ms inter-byte timeout above, purely
   about not getting stuck mid-frame.
2. **Command-level (Pico watchdog):** if `WATCHDOG_TIMEOUT_MS` (300 ms)
   passes without a valid CONTROL frame, the Pico:
   - forces `drive_cmd` and `steer_cmd` to 0 (effective stop) regardless
     of the last commanded values,
   - sets the `COMM_TIMEOUT` fault bit in the next TELEMETRY frame,
   - keeps reporting `cmd_age_ms` so the Jetson can see exactly how stale
     things are.

   This is the safety-critical line in `pico_main.py` — the same
   `effective_stop` check also honors the explicit `stop` flag and
   `mode == DISABLED`, so all three fail-safe triggers (comm loss, explicit
   stop, disabled mode) go through one code path.

3. **Link-level (Jetson, informational):** `jetson_test.py` separately
   considers the *link* down if no TELEMETRY frame has arrived in 500 ms —
   this doesn't affect the Pico's own safety behavior (which never depends
   on the Jetson noticing anything), but it's what a higher-level
   autonomy/teleop layer would check before trusting telemetry.

300 ms and 500 ms are early-version placeholders (see section 8) chosen to
be comfortably longer than one CONTROL/TELEMETRY period (50 ms) plus
margin, not tuned against any real actuator response time yet.

## 7. Demonstration (no hardware required)

Two things are demonstrated, both runnable directly:

```
python3 demo.py
```

**Part A — CRC catches corruption and the parser resyncs.** A CONTROL
frame is encoded, one payload byte is deliberately flipped, and the
corrupted bytes are fed into a parser immediately followed by a valid
TELEMETRY frame. Result: the corrupted frame is rejected (CRC mismatch
counted), and the following valid frame is still parsed correctly —
proving the link recovers from a single corrupted frame without operator
intervention.

**Part B — live CONTROL/TELEMETRY exchange with a simulated link cut.**
Using `mock_link.py` (an in-memory duplex "cable" with the same
`write`/`read`/`in_waiting` interface as a real `pyserial.Serial` object)
and `pico_sim.py` (a pure-Python mirror of `pico_main.py`'s protocol logic,
needed only because this sandbox can't run real MicroPython on Pico
hardware), the Jetson side sends CONTROL at 20 Hz while receiving and
decoding TELEMETRY, for 1.5 s of normal operation, then a deliberate 0.6 s
pause in sending (longer than the 300 ms watchdog), then resumes.

Actual captured output from a run:

```
-- Normal operation: sending CONTROL at 20Hz --
  t= 1.45s  cmd_age=    5ms  enc_left=+13600  current= 7500mA  faults=none
-- Simulated link cut: Jetson stops sending (600ms > 300ms watchdog) --
  t= 1.51s  cmd_age=   10ms  enc_left=+14600  current= 7500mA  faults=none
  t= 1.67s  cmd_age=  159ms  enc_left=+16050  current= 7500mA  faults=none
  t= 1.84s  cmd_age=  308ms  enc_left=+17400  current=    0mA  faults=COMM_TIMEOUT
  t= 2.00s  cmd_age=  458ms  enc_left=+17400  current=    0mA  faults=COMM_TIMEOUT
-- Link restored: sending CONTROL again --
  t= 2.12s  cmd_age=  607ms  enc_left=+17400  current=    0mA  faults=COMM_TIMEOUT
  t= 2.28s  cmd_age=   10ms  enc_left=+18210  current= 4500mA  faults=none
```

`cmd_age_ms` climbs during the cut, `COMM_TIMEOUT` appears once it crosses
300 ms, current drops to 0 (motor stopped) and the encoder stops advancing
— then everything clears and resumes normally once CONTROL frames start
arriving again. (Full log: run `python3 demo.py`.)

**Against real hardware**, once the Pico is flashed with `pico_main.py`
(copy it on as `main.py`) and wired to the Jetson (or any USB-serial
adapter for bench testing), the exact same test script runs unmodified:

```
python3 jetson_test.py --port /dev/ttyACM0 --baud 115200 --duration 10
```

## 8. Known placeholders / what's explicitly deferred

This is an early version by design (per the task), so the following are
called out as **not yet finalized**, so nobody mistakes them for settled
decisions:

- **`drive_cmd`/`steer_cmd` scaling.** Currently a unitless ±1000
  percent-of-range. Once motor controllers and steering geometry are
  chosen, this likely becomes a real physical unit (e.g. mm/s, degrees).
- **Encoder count/layout.** Currently assumes two drive encoders
  (left/right). If the final drivetrain has a different encoder
  arrangement (e.g. per-wheel on a 6-wheel rocker-bogie), TELEMETRY's
  payload will need to change.
- **Timing constants** (20 Hz rates, 50 ms/300 ms/500 ms timeouts) are
  reasonable starting points, not derived from real actuator/latency
  measurements yet.
- **Motor/sensor I/O in `pico_main.py`** (`read_sensors`,
  `set_motor_outputs`) are explicitly stubbed — that's the whole point of
  `SIMULATE_SENSORS`, so the protocol layer is testable before the final
  drivetrain hardware exists. Swapping in real GPIO/ADC code there should
  not require touching anything else in the file.
- **Not yet included, left for a later version:** a heartbeat/ping message
  independent of CONTROL, an ACK/NACK scheme, a firmware-version or
  capability query, and any kind of message versioning field. None of
  these block an early working link, but they're natural next additions
  once this version is validated on real hardware.

## 9. File map

| File | Runs on | Purpose |
|---|---|---|
| `PROTOCOL.md` | — | This document |
| `framing.py` | Jetson (CPython) | Reference CRC/framing/encode/decode, imported by the Jetson-side files |
| `jetson_test.py` | Jetson (CPython) | The actual computer-side test script — real serial (`--port`) or mock (`--mock`) |
| `mock_link.py` | Jetson (CPython) | In-memory duplex loopback, no hardware needed |
| `pico_sim.py` | Jetson (CPython) | Demo-only mirror of the Pico's protocol logic, for `--mock` runs |
| `pico_main.py` | **Pico (MicroPython)** | The actual firmware — copy this one file onto the board as `main.py` |
| `demo.py` | Jetson (CPython) | Runs the full demonstration in section 7 |
