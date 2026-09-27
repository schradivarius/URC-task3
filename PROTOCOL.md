# Rover Onboard Computer ↔ Embedded Controller Communication Protocol (v0.2 — early version)

**Task 3: Jetson and Microcontroller communication**

**Scope:** Onboard computer (Jetson Orin Nano, candidate) ↔ embedded
controller (**Adafruit Feather RP2040 RFM9x**, candidate, running
**CircuitPython**), over a single asynchronous serial link.

This is a working version of the link: reliable framing, a CRC, and
timeout/fail-safe handling, with the required CONTROL and TELEMETRY fields.
It is deliberately hardware-agnostic — every value, scaling constant, and
field width is a placeholder that can be tightened once final motors,
encoders, and sensors are chosen. Those placeholders are called out
explicitly in section 9 so nothing here is mistaken for a finalized spec.

> **Changes from v0.1.** The CRC is correctly identified (v0.1 called it
> XModem; it is not — see section 3.1, this was an interoperability bug).
> `LEN` is now validated before the parser commits to waiting on it
> (section 5.1). `current_ma` became signed `current_ca` (section 4.2).
> `cmd_age_ms` saturation no longer collides with its "never received"
> sentinel. The firmware gained a hardware watchdog and an exception guard
> (section 6). Encoder counters wrap instead of overflowing. The wire format
> is now implemented **once** and shared by both sides (section 8).

---

## 1. Design goals

- **No hardware dependency.** The format, parser, firmware and demo all run
  without the final Jetson or Feather in hand — proven in section 7 using a
  software loopback and stubbed hardware modules instead of a physical cable.
- **Reliable framing** over a byte stream that gives no message boundaries
  for free.
- **Corruption detection** via CRC, with a defined recovery behavior rather
  than "hope it doesn't happen" — and with a bounded recovery *cost*, which
  is a distinct requirement that v0.1 got wrong.
- **Fail-safe on silence.** If the onboard computer goes quiet, the rover
  stops. This is the single most important property of the whole protocol.
- **One implementation.** The same `framing.py` and `controller.py` run on
  the Jetson and on the microcontroller. Nothing safety-critical is
  hand-duplicated.
- **Room to grow.** Adding a new message type or field doesn't require
  changing the framing, and an older peer tolerates an unknown message
  rather than desynchronizing.

## 2. Link basics

| Parameter | Value |
|---|---|
| Physical layer | Asynchronous serial: USB CDC data channel, or UART 115200 8N1 |
| Direction | Full duplex — each side sends on its own timer |
| CONTROL rate | 20 Hz (onboard computer → controller) |
| TELEMETRY rate | 20 Hz (controller → onboard computer) |
| Byte order | Little-endian for all multi-byte fields |

20 Hz is a reasonable starting control-loop rate for a teleoperated/early-
autonomy rover — easy to change later (see section 9).

### 2.1 Which serial device, and why it matters

On CircuitPython the default USB serial device (`/dev/ttyACM0`) **is the REPL
console**. Frames written there are fed to the Python interpreter, not to the
firmware. `boot.py` therefore calls `usb_cdc.enable(console=True, data=True)`,
which exposes a second CDC endpoint — normally `/dev/ttyACM1` — carrying only
protocol frames, and leaves the console on the first endpoint for debugging.

| Transport | Host device | Notes |
|---|---|---|
| USB CDC data channel | `/dev/ttyACM1` | Preferred. Requires `boot.py` and one **power-cycle** (soft reset is not enough). |
| Hardware UART0 | `/dev/ttyUSB0` | Feather `TX`/`RX` pads = GP0/GP1, via a USB-TTL adapter. Fallback path in `open_link()`. |

v0.1 documented `--port /dev/ttyACM0` while the firmware listened only on
UART0 pins. Those are different physical wires; the documented command could
not have worked.

## 3. Frame format

```
 byte:   0        1        2        3 .. 3+LEN-1      3+LEN .. 3+LEN+1
       +--------+--------+--------+ ~~~~~~~~~~~~~~~ +-------------------+
       | START  | MSG_ID |  LEN   |     PAYLOAD     |   CRC16 (LE)      |
       | 0xAA   | 1 byte | 1 byte |    LEN bytes    |     2 bytes       |
       +--------+--------+--------+ ~~~~~~~~~~~~~~~ +-------------------+
```

- **START (`0xAA`)** — marks a possible frame start. Not escaped or reserved
  elsewhere in the frame (see section 5 for why that's safe).
- **MSG_ID** — identifies the message (table in section 4).
- **LEN** — payload length in bytes. Validated on receipt; see section 5.1.
- **PAYLOAD** — message-specific fields, packed with `struct`.
- **CRC16** — computed over `MSG_ID + LEN + PAYLOAD` (not the START byte, not
  the CRC field itself). Little-endian on the wire.

No byte-stuffing or escaping is used. The receiver instead uses `LEN` to know
how many bytes make up the frame, then verifies the whole thing with the CRC
before accepting it.

### 3.1 CRC algorithm — read this before writing a second implementation

**CRC-16/CCITT-FALSE** (also catalogued as CRC-16/IBM-3740):

| Parameter | Value |
|---|---|
| Polynomial | `0x1021` |
| Initial value | **`0xFFFF`** |
| Reflect in / out | No / No |
| Final XOR | None |
| **Check value** (`"123456789"`) | **`0x29B1`** |

**This is not CRC-16/XMODEM.** XMODEM uses the same polynomial but
initialises to `0x0000`, giving a check value of `0x31C3` and completely
different output for every input. v0.1 of this document and of the code both
labelled the algorithm "XModem" while implementing CCITT-FALSE. Anyone
implementing "CRC-16/XMODEM" from that description — a base-station tool, a
C++ reimplementation, an off-the-shelf library call — would have had **every
frame rejected**, with no obvious clue why.

Init `0xFFFF` is the better of the two choices here because it makes the CRC
sensitive to leading zero bytes, which init `0x0000` is not. So the
implementation stays and the name is corrected.
`tests/test_protocol.py::TestCRC::test_known_answer_vector` pins the check
value so the spec and the code cannot drift apart silently again.

## 4. Messages

### 4.1 CONTROL (onboard computer → controller), `MSG_ID = 0x01`, 6-byte payload

| Field | Type | Range | Meaning |
|---|---|---|---|
| `drive_cmd` | int16 | -1000..1000 | Desired drive command, tenths of a percent (±100.0%) of full drive effort |
| `steer_cmd` | int16 | -1000..1000 | Desired steering command, tenths of a percent of full steering range |
| `mode` | uint8 | 0/1/2 | Operating mode: `0=DISABLED, 1=MANUAL, 2=AUTONOMOUS` |
| `stop` | uint8 | 0/1 | Explicit stop command. `1` forces an immediate stop regardless of `mode` |

`stop` is deliberately a separate field rather than a third mode value, so an
emergency stop can be asserted **and released** without a mode-change round
trip, and so it is continuously refreshed alongside the rest of CONTROL
rather than being a one-shot event that could be missed. The stop takes
effect on the same control cycle it arrives — there is no coast frame.

### 4.2 TELEMETRY (controller → onboard computer), `MSG_ID = 0x02`, 15-byte payload

| Field | Type | Range | Meaning |
|---|---|---|---|
| `enc_left` | int32 | wraps | Left-side cumulative encoder ticks. **Wraps**; treat as relative. |
| `enc_right` | int32 | wraps | Right-side cumulative encoder ticks |
| `steer_fb` | int16 | -1000..1000 | Measured steering position, same scale as `steer_cmd` |
| `current_ca` | **int16** | ±327.67 A | Motor current in **centiamps** (1 cA = 10 mA). **Signed.** |
| `fault_status` | uint8 | bitmask | See table below |
| `cmd_age_ms` | uint16 | see below | Milliseconds since the last **valid** CONTROL frame |

**On `current_ca`.** v0.1 used unsigned milliamps, which caps at 65.5 A and
cannot express negative current. A multi-motor rover exceeds that ceiling,
and regenerative braking or a reversed sense line produces genuinely negative
readings that would have wrapped to a huge positive value — the worst kind of
bad telemetry, because it looks plausible. Signed centiamps gives ±327 A at
10 mA resolution in the same two bytes.

**On `cmd_age_ms`.** Two reserved values, deliberately distinct:

| Value | Meaning |
|---|---|
| `0xFFFF` (`CMD_AGE_UNKNOWN`) | No valid CONTROL frame has **ever** arrived |
| `0xFFFE` (`CMD_AGE_MAX`) | Saturation ceiling for a real measurement (≥ 65.534 s) |

v0.1 used `0xFFFF` for both, so the onboard computer could not tell a
controller that had never heard from it apart from one it had stopped talking
to a minute ago. Those call for different operator responses.

**Fault bitmask (`fault_status`):**

| Bit | Name | Meaning |
|---|---|---|
| `0x01` | `COMM_TIMEOUT` | Watchdog expired — no valid CONTROL frame within the timeout |
| `0x02` | `OVER_CURRENT` | Current reading exceeded a threshold |
| `0x04` | `ESTOP_ACTIVE` | Hardware or software E-stop engaged |
| `0x08` | `ENCODER_FAULT` | Encoder reading invalid or stalled |
| `0x10` | `UNDERVOLTAGE` | Supply voltage low (placeholder, not wired up yet) |
| `0x20` | `FIRMWARE_FAULT` | Main loop raised an exception; see section 6.2 |
| `0x40`–`0x80` | *reserved* | Free for future use |

`cmd_age_ms` is the field the task calls out explicitly, and it is the
mechanism the whole fail-safe design hangs off: the onboard computer can see,
frame by frame, how stale its own commands look to the controller — a much
more direct link-health signal than merely noticing telemetry stopped
arriving.

## 5. Framing robustness: corruption and resync

Every candidate frame is CRC-checked before being accepted. On a mismatch —
whether from line noise, or from a data byte that happened to equal `0xAA` and
caused a false-positive frame start — the parser drops **just the leading
START byte** and resumes scanning for the next `0xAA`. One bad frame does not
bring down the link, and the very next valid frame is parsed normally. Proven
in the demo, section 7 Part A.

A **stale partial frame** — bytes started arriving but the rest never showed
up — is cleared after `inter_byte_timeout_ms` (50 ms) of silence, so a
truncated transmission can't wedge the parser waiting forever.

The receive buffer is additionally **hard-capped**, so a peer emitting endless
junk cannot grow it without bound on a memory-constrained board.

### 5.1 `LEN` validation — why "trust LEN" is not enough

A parser that trusts `LEN` blindly will, on receiving a `LEN` of `0xFF`, block
until 3+255+2 = **260 bytes** have arrived before it can even check the CRC
and discover the frame was garbage. At 20 Hz with 11-byte CONTROL frames that
is roughly **1.18 s of blackout — nearly four times the 300 ms command
watchdog.** So in v0.1 a single corrupted `LEN` byte was enough to trip
`COMM_TIMEOUT` and stop the rover.

So `MSG_ID` and `LEN` are sanity-checked *before* the parser agrees to wait:

- A **known** `MSG_ID` must carry exactly its defined payload length
  (CONTROL 6, TELEMETRY 15). Anything else is corruption by definition.
- An **unknown** `MSG_ID` is tolerated up to `MAX_PAYLOAD_LEN` (32) and still
  CRC-checked, so a newer firmware can introduce message types without an
  older peer choking. Above that, it is treated as certain corruption.
- A rejected header costs **one byte** of resync instead of 260.

Regression test:
`tests/test_protocol.py::TestParserRobustness::test_corrupted_len_does_not_stall_the_link`.

## 6. Timeout, watchdog and fault behavior

### 6.1 The three timeouts

1. **Frame-level (parser).** The 50 ms inter-byte timeout above — purely
   about not getting stuck mid-frame.
2. **Command-level (controller watchdog).** If `WATCHDOG_TIMEOUT_MS` (300 ms)
   passes without a valid CONTROL frame, the controller forces drive and
   steering to 0, sets `COMM_TIMEOUT` in the next TELEMETRY frame, and keeps
   reporting `cmd_age_ms` so the staleness is visible.
3. **Link-level (onboard computer, informational).** `jetson_test.py`
   separately considers the *link* down after 500 ms with no TELEMETRY. This
   does not affect the controller's own safety behavior, which never depends
   on the onboard computer noticing anything.

All three fail-safe triggers — comm loss, explicit `stop`, and
`mode == DISABLED` — go through a single code path,
`controller.RoverController.effective_stop()`. Commands are zeroed by
`commanded_outputs()` before they ever reach the motor layer, so a future
rewrite of `set_motor_outputs()` cannot accidentally honour a stale command.

**A CRC-invalid frame never refreshes the watchdog.** Otherwise corruption
would keep the watchdog alive while the rover acted on stale data
(`test_corrupt_control_does_not_refresh_the_watchdog`).

### 6.2 Surviving a firmware fault

A watchdog that only covers the *link* leaves the larger hole: if the
firmware itself dies, the PWM registers keep their last value. The motors
stay energized and nothing is left running to notice. v0.1 had exactly this
hole, and an unbounded encoder counter that would eventually trigger it after
about 12 hours of driving (`struct.pack` raising on int32 overflow).

Three layers now cover it:

1. **Hardware watchdog.** `microcontroller.watchdog` in `WatchDogMode.RESET`,
   fed once per loop iteration. If the loop stops for `HW_WATCHDOG_TIMEOUT_S`
   (1.0 s), the RP2040 resets — which drives every pin high-Z and
   de-energizes the drives. If a build does not expose a watchdog, the
   firmware still boots and says so loudly in its startup banner.
2. **Exception guard.** Anything escaping the loop body calls `safe_stop()`
   *first*, before logging or anything else that could itself fail, then
   re-raises so the watchdog completes the reset. `safe_stop()` is written to
   never raise.
3. **Bounded counters.** Encoder accumulators pass through
   `framing.wrap_i32()`, so they wrap rather than overflow. Counts are
   relative anyway; the onboard computer handles rollover when differencing.

300 ms, 500 ms and 1.0 s are early-version placeholders (section 9), chosen to
be comfortably longer than one CONTROL/TELEMETRY period (50 ms) plus margin,
not tuned against any measured actuator response.

## 7. Demonstration (no hardware required)

```
python3 demo.py                              # the three-part demonstration
python3 jetson_test.py --mock --duration 5   # the host harness, mock link
python3 -m unittest discover -s tests -v     # 67 tests, no dependencies
```

**Part A — CRC catches corruption and the parser resyncs.** A CONTROL frame is
encoded, one payload byte is flipped, and the corrupted bytes are fed in
immediately followed by a valid TELEMETRY frame. The corrupted frame is
rejected and the following valid frame still parses.

**Part C — a corrupted `LEN` byte costs one byte, not 260.** The v0.1
regression, made visible:

```
Fed 1 frame with LEN corrupted to 0xFF, then 5 valid CONTROL frames
  a LEN-trusting parser would stall for 260 bytes = 1182ms at 20Hz (vs the 300ms watchdog)
Frames recovered immediately: 5  (expected 5)
Parser stats: {'frames_ok': 5, 'crc_errors': 0, 'bad_headers': 1, ...}
PASS: implausible LEN rejected after 1 byte; no frames were delayed.
```

**Part B — live CONTROL/TELEMETRY exchange with a simulated link cut.** Using
`mock_link.py` (an in-memory duplex "cable" with the same
`write`/`read`/`in_waiting` interface as a real `pyserial.Serial`) and
`mcu_sim.py` (which drives the **same** `controller.py` the firmware runs, with
a faked plant), the host sends CONTROL at 20 Hz for 1.5 s, then deliberately
pauses for 0.6 s — longer than the 300 ms watchdog — then resumes:

```
-- Normal operation: sending CONTROL at 20Hz --
  t= 1.46s  cmd_age=   15ms  enc_left=+14100  current=  +7.50A  faults=none
-- Simulated link cut: Jetson stops sending (600ms > 300ms watchdog) --
  t= 1.68s  cmd_age=  153ms  enc_left=+16050  current=  +7.50A  faults=none
  t= 1.84s  cmd_age=  305ms  enc_left=+17400  current=  +0.00A  faults=COMM_TIMEOUT
  t= 2.00s  cmd_age=  454ms  enc_left=+17400  current=  +0.00A  faults=COMM_TIMEOUT
-- Link restored: sending CONTROL again --
  t= 2.28s  cmd_age=   11ms  enc_left=+18180  current=  +4.50A  faults=none
```

`cmd_age_ms` climbs during the cut, `COMM_TIMEOUT` appears once it crosses
300 ms, current drops to zero and the encoders stop advancing — then
everything clears and resumes once CONTROL frames return.

**Against real hardware**, once the Feather is running `feather_main.py`, the
same script runs unmodified against the data channel:

```
python3 jetson_test.py --port /dev/ttyACM1 --baud 115200 --duration 10
```

### 7.1 The firmware itself is tested, not just the simulator

`tests/fake_hardware.py` supplies stub `board` / `busio` / `digitalio` /
`microcontroller` / `usb_cdc` modules, so `feather_main.py` — the exact file
that gets flashed — is imported and driven under CPython in CI. That covers
the safe boot state, acting on a valid CONTROL frame, ignoring a CRC-invalid
one, honouring `stop`, feeding the hardware watchdog, arming it in RESET mode,
and stopping the motors from the exception guard. Otherwise the firmware
would be the one file no test ever touches, which is where a bug costs the
most.

## 8. One implementation, not two

v0.1 hand-copied the CRC, the struct formats and the frame parser into the
firmware, with a comment asking maintainers to "keep the two in sync". The
simulator separately re-implemented the watchdog and fault logic. Two
hand-maintained copies of a safety rule is how you get a simulator that
passes while the real firmware misbehaves — the tests would have been
checking the wrong copy.

The justification given was that the board cannot `import` a file living on
the other side of a serial link. True, but irrelevant: you copy two files onto
CIRCUITPY instead of one. Both `framing.py` and `controller.py` import nothing
but `struct`, which CPython and CircuitPython both provide.

| Module | Runs on | Holds |
|---|---|---|
| `framing.py` | Both | CRC, struct formats, encode/decode, `FrameParser` |
| `controller.py` | Both | Command/safety state machine: watchdog, `effective_stop`, telemetry pacing |

`tests/test_protocol.py::TestNoDuplicatedProtocol` enforces this structurally
— it fails if the firmware redefines the CRC, the struct formats, or the
parser, or if the simulator grows its own copy of the stop rule.

## 9. Known placeholders / what's explicitly deferred

This is an early version by design, so the following are **not yet
finalized**, so nobody mistakes them for settled decisions:

- **`drive_cmd`/`steer_cmd` scaling.** Currently a unitless ±1000
  percent-of-range. Once motor controllers and steering geometry are chosen,
  this likely becomes a real physical unit (mm/s, degrees).
- **Encoder count and layout.** Two drive encoders (left/right) is a
  placeholder. A 6-wheel rocker-bogie needs six, which changes the TELEMETRY
  payload.
- **One aggregate current reading.** The task asks for current *readings*.
  A single number cannot identify *which* wheel is stalling. Expect this to
  become per-motor, which changes the payload length.
- **Timing constants** (20 Hz rates; 50 / 300 / 500 ms and 1.0 s timeouts) are
  reasonable starting points, not derived from measured actuator or latency
  behavior.
- **Motor/sensor I/O in `feather_main.py`** (`read_sensors`,
  `set_motor_outputs`) is explicitly stubbed — that is what `SIMULATE_SENSORS`
  is for. Swapping in real drivers should not require touching anything above
  those functions.
- **Not yet included:** a sequence number (would let either side measure drop
  rate directly rather than inferring it), a heartbeat/ping independent of
  CONTROL, ACK/NACK, a firmware-version or capability query, and a protocol
  version field. None block an early working link.

### 9.1 Two open decisions this protocol does not settle

**The I/O budget does not close.** GPIO16–23 on the Feather RP2040 RFM9x are
consumed by the RFM9x module and are not broken out. After reserving the
Jetson UART (GP0/GP1), I2C (GP2/GP3), radio SPI (GP8/GP14/GP15), the NeoPixel
(GP4) and the boot button (GP7), roughly **13 GPIO remain, with only 4 ADC
channels** (A0–A3 = GP26–29). A 6-wheel rover with 4-corner steering needs
~29 pins on direct GPIO, before the arm and science payload that the current
planning assumption also routes through this controller.

The likely resolution is to push I/O off-board — motor controllers over one
serial link, servos via PCA9685 (I2C), current sense via ADS1115 (I2C),
encoders via RP2040 PIO or dedicated counter chips — but that is an
electrical/architecture decision, not a protocol one. It is recorded here
because it must be settled before the module stubs are filled in.

**What the RFM9x radio is for.** The onboard computer ↔ controller link is
on-board and wired, so the 900 MHz LoRa radio is presumably a rover ↔ base
station link (plausibly a wireless E-stop path). If so, the frame format
carries over unchanged — it is just bytes — but **the rate profile cannot**.
At SF7/BW125 kHz a 20-byte packet is roughly 40–60 ms of airtime, so 20 Hz is
not physically achievable; expect 2–5 Hz sustained, before duty-cycle limits.
That implies a per-transport profile (a slower rate and a longer watchdog over
the radio) and probably a slimmer heartbeat/E-stop message for that link.
The radio's own packets carry framing and a CRC, making our layer redundant
there — harmless, and worth keeping so there is one codec.

## 10. File map and build instructions

| File | Runs on | Purpose |
|---|---|---|
| `PROTOCOL.md` | — | This document |
| `framing.py` | **Both** | CRC, framing, encode/decode, `FrameParser` |
| `controller.py` | **Both** | Command/safety state machine |
| `feather_main.py` | **Feather (CircuitPython)** | The firmware — flash as `code.py` |
| `boot.py` | **Feather (CircuitPython)** | Enables the USB CDC data channel |
| `jetson_test.py` | Jetson (CPython) | Host-side test script — real serial or mock |
| `mock_link.py` | Jetson (CPython) | In-memory duplex loopback with fault injection |
| `mcu_sim.py` | Jetson (CPython) | Demo-only fake plant driving the real `controller.py` |
| `demo.py` | Jetson (CPython) | The three-part demonstration in section 7 |
| `tests/` | Jetson (CPython) | 67 tests; stdlib `unittest`, no dependencies |

### 10.1 Host setup

```bash
python3 -m unittest discover -s tests -v   # no dependencies needed
python3 demo.py                            # no dependencies needed
pip install -r requirements.txt            # pyserial, for real hardware only
```

### 10.2 Flashing the Feather RP2040 RFM9x

1. Install CircuitPython: download the **Feather RP2040 RFM9x** `.uf2` from
   <https://circuitpython.org/board/adafruit_feather_rp2040_rfm9x/>,
   double-tap `RESET` to get the `RP2040` bootloader drive, and copy the
   `.uf2` onto it. The board reboots as `CIRCUITPY`.
2. Copy these files onto `CIRCUITPY`:

   | From | To |
   |---|---|
   | `feather_main.py` | `code.py` |
   | `framing.py` | `framing.py` |
   | `controller.py` | `controller.py` |
   | `boot.py` | `boot.py` |

3. **Power-cycle the board** (unplug and replug). `boot.py` only runs on hard
   reset, so a soft reset will not create the data channel.
4. Confirm both endpoints exist, then run the host script:

   ```bash
   ls /dev/ttyACM*        # expect ttyACM0 (console) and ttyACM1 (data)
   python3 jetson_test.py --port /dev/ttyACM1
   ```

5. The startup banner on the console endpoint reports which link was chosen
   and whether the hardware watchdog armed:

   ```
   rover firmware up: link = usb_cdc.data (second CDC endpoint), hw watchdog = on
   ```

   `hw watchdog = UNAVAILABLE` means the build exposes no watchdog and the
   exception guard is your only backstop — worth fixing before field use.
