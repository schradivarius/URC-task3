# Rover Onboard Computer ↔ Embedded Controller Protocol (v0.3.1 — CAN)

**Scope:** Onboard computer (Jetson Orin Nano, candidate) ↔ embedded controller
(**Teensy 4.1**, NXP i.MX RT1062, running C++ / Teensyduino), over **CAN**.

> **What changed from v0.2, and why the spec got shorter.** v0.2 ran over a
> UART and carried a start byte, a length byte, a CRC-16 and a 108-line
> resynchronizing stream parser — all of it there because a byte stream has no
> message boundaries. CAN delivers whole frames or nothing, with a 15-bit CRC
> and automatic retransmission **in silicon**. So sections 3 and 5 of v0.2 are
> gone, and with them an entire class of bug: a corrupted length byte stalling
> the link past the watchdog is now *structurally impossible* rather than
> merely tested for.
>
> What survived untouched is everything that was never about the wire — the
> message fields, the fault bitmask, the command-age semantics and the
> fail-safe rule. That is not luck; it is why the safety logic was a separate
> module from the framing in the first place.

---

## 1. Design goals

- **No hardware dependency for development.** The protocol, the safety logic
  and the full exchange all run and are tested on a laptop with no board, no
  CAN interface and no drivers (section 7).
- **Fail-safe on silence.** If the onboard computer goes quiet, the rover
  stops. This is the single most important property here.
- **One implementation of the safety rule.** It exists once, in C++, and the
  simulator used by the host tests *is that compiled code* (section 8).
- **Portable across CAN flavours.** Every message fits 8 bytes, so this runs on
  Classic CAN or CAN FD, alongside Classic-only motor controllers.

## 2. Link basics

| Parameter | Value |
|---|---|
| Physical layer | CAN 2.0B, differential twisted pair, **120 Ω termination at both ends** |
| Bit rate | 500 kbps (placeholder; 1 Mbps is fine at these frame sizes) |
| Bus | Teensy 4.1 **CAN3** (the FD-capable bus) — CAN1/CAN2 work unchanged |
| Frame format | Standard, 11-bit identifiers |
| CONTROL rate | 20 Hz (Jetson → controller) |
| TELEMETRY rate | 20 Hz (controller → Jetson) |
| Byte order | Little-endian, written byte-by-byte (not struct-punned) |

At 500 kbps an 8-byte frame is roughly 220 µs of airtime — about 0.4% of the
50 ms period. **Throughput is nowhere near the constraint**; CAN was chosen for
determinism and noise immunity, not speed. Say so plainly when anyone asks.

### 2.1 Identifiers *are* priorities

CAN arbitration is bitwise and dominant-low, so the **numerically lowest id
wins the bus**, and the loser backs off without its message being corrupted.
The id table is therefore a priority ordering, not just a set of names.

| ID | Message | Direction | DLC |
|---|---|---|---|
| `0x000`–`0x0FF` | *reserved* — headroom for a dedicated e-stop frame | — | — |
| **`0x100`** | `CONTROL` | Jetson → controller | 8 |
| **`0x200`** | `TELEM_MOTION` | controller → Jetson | 8 |
| **`0x201`** | `TELEM_STATUS` | controller → Jetson | 8 |

Commands outrank telemetry because a late command can hurt the rover and late
telemetry only annoys an operator. The block below `0x100` is deliberately
empty so a future e-stop frame can outrank everything here without renumbering.

## 3. Messages

Telemetry is **split across two frames** so each fits Classic CAN's 8-byte
limit. A single 15-byte frame would have required CAN FD transceivers on every
node of the bus, including motor controllers that are commonly Classic-only.
Separate ids also mean a lost motion frame does not cost the Jetson its fault
status.

### 3.1 `CONTROL` — `0x100`, DLC 8

| Field | Type | Range | Meaning |
|---|---|---|---|
| `drive_cmd` | int16 | -1000..1000 | Desired drive, tenths of a percent of full effort |
| `steer_cmd` | int16 | -1000..1000 | Desired steering, tenths of a percent of full range |
| `mode` | uint8 | 0/1/2 | `0=DISABLED, 1=MANUAL, 2=AUTONOMOUS` |
| `stop` | uint8 | 0/1 | Forces an immediate stop regardless of `mode` |
| `indicator_request` | uint8 | 0/1/2/3 | Status light the Jetson **asks** for — see §3.4. The MCU decides what is actually shown. |
| `c2_lost` | uint8 | 0/1 | `1` = the Jetson has lost the base-station (C2) link. Decided by the Jetson, see below. |

`stop` is a separate field rather than a third mode value so an e-stop can be
asserted **and released** without a mode round trip. It takes effect on the
control cycle it arrives — there is no coast frame.

**`c2_lost` is the Jetson's report on a link the controller cannot see.** The
controller can detect a silent Jetson itself (the command watchdog), but it has
no view of the radio link between the base station and the Jetson. So the
Jetson watches that link (`C2Monitor` in `host/c2_link.py`: no base-station
heartbeat within `C2_timeout_s`, default 1.0 s, means lost) and reports the result in every
`CONTROL` frame. The two conditions are deliberately separate: C2 loss with a
healthy Jetson is expected on the autonomy course, where line of sight to the
base station drops while onboard autonomy keeps running.

- It is appended as the last byte (byte 7, after `indicator_request`), so no
  existing field moved. That fills `CONTROL` to 8 bytes, the Classic CAN
  maximum: any further field needs CAN FD or a second frame.
- The controller boots with `c2_lost = 1` and holds it until the first valid
  frame says otherwise, matching every other boot default (assume the worst).
- `encode_control()` has **no default** for it. A caller that forgot the flag
  would otherwise silently report the link as healthy.
- **What the controller does with it** depends on who is driving:

  | Mode | `c2_lost = 1` → | Why |
  |---|---|---|
  | `MANUAL` | **stop** | The operator's commands can no longer arrive |
  | `AUTONOMOUS` | **keep driving** | The Jetson is driving, and is provably alive or the command watchdog would already have stopped the rover |
  | `DISABLED` | stopped anyway | — |

  `FAULT_C2_LINK_LOST` is raised in **every** mode. During an autonomous run
  it is the only sign that the rover is out of contact. This per-mode policy is
  an initial choice and may be revised.
- **Once `JETSON_HEARTBEAT_LOST` is set, the C2 bit is unknown, not OK or
  LOST.** The controller only knows `c2_lost` from the last `CONTROL` frame, so
  when the Jetson goes silent that value goes stale: a clear bit ten seconds
  later does not mean C2 is fine, and a set bit does not mean it is still down.
  This needs no wire change, because the heartbeat bit travels in the same
  frame. Receivers apply the rule: `describe_faults()` in `host/rover_protocol.py`
  hides the stale `C2_LINK_LOST` and reports `C2_UNKNOWN` instead. Keep the two
  apart: LOST (decided by the Jetson's `C2Monitor`) is what lets autonomy keep
  driving; UNKNOWN only stops a dashboard from trusting a stale value.

**`mode` is validated, and an undefined value stops the rover.** The field is a
uint8, so it carries 256 possible values where three are defined. Two separate
questions follow, and conflating them was [issue #4](https://github.com/schradivarius/URC-task3/issues/4):

| Question | Answer |
|---|---|
| Is this value *defined*? (`isKnownMode`) | `0`, `1`, `2` — **`DISABLED` is valid**, a legitimate command meaning "do not move" |
| May the rover *move*? (`modePermitsMotion`) | `MANUAL` or `AUTONOMOUS` only — a **whitelist** |

A receiver **rejects** a frame carrying an undefined mode, exactly as it
rejects a wrong DLC: the sender disagrees with us about the protocol. The
rejected frame does not refresh the command watchdog, and `PROTOCOL_ERROR` is
raised so the stop is explainable rather than silent.

This is deliberately **not** forward-compatible the way an unknown CAN id is.
A newer peer sending a mode this firmware does not implement must stop this
rover, not be tolerated. If you add a fourth mode, every controller on the bus
needs the update before a host may send it.

### 3.2 `TELEM_MOTION` — `0x200`, DLC 8

| Field | Type | Meaning |
|---|---|---|
| `enc_left` | int32 | Left cumulative encoder ticks. **Wraps** — treat as relative. |
| `enc_right` | int32 | Right cumulative encoder ticks |

### 3.3 `TELEM_STATUS` — `0x201`, DLC 8

| Field | Type | Range | Meaning |
|---|---|---|---|
| `steer_fb` | int16 | -1000..1000 | Measured steering, same scale as `steer_cmd` |
| `current_ca` | **int16** | ±327.67 A | Current in **centiamps** (1 cA = 10 mA). **Signed.** |
| `fault_status` | uint8 | bitmask | See below |
| `cmd_age_ms` | uint16 | see below | ms since the last valid `CONTROL` frame |
| `indicator_state` | uint8 | 0/1/2/3 | Status light the MCU is **actually showing** — see §3.4 |

`current_ca` is signed because a braking motor genuinely produces negative
current; unsigned would wrap that to a large positive value, which is the worst
kind of bad telemetry because it looks plausible on a dashboard.

**Fault bitmask.** Powers of two, so faults combine:

| Bit | Name | Meaning |
|---|---|---|
| `0x01` | `JETSON_HEARTBEAT_LOST` | No valid `CONTROL` frame within the watchdog |
| `0x02` | `OVER_CURRENT` | Current exceeded a threshold |
| `0x04` | `ESTOP_ACTIVE` | Hardware or software e-stop engaged |
| `0x08` | `ENCODER_FAULT` | Encoder reading invalid or stalled |
| `0x10` | `UNDERVOLTAGE` | Supply voltage low (placeholder) |
| `0x20` | `FIRMWARE_FAULT` | This boot followed a watchdog reset (section 5.2) |
| `0x40` | `PROTOCOL_ERROR` | The last frame on our id was uninterpretable: wrong DLC, an undefined `mode`, or an undefined `indicator_request`. Self-clears when a valid frame arrives. |
| `0x80` | `C2_LINK_LOST` | The last `CONTROL` frame reported the base station ↔ Jetson link lost (`c2_lost = 1`). Raised in every mode; stops the rover only in `MANUAL` (section 3.1). **Stale while `JETSON_HEARTBEAT_LOST` is set**: receivers must then treat the C2 state as unknown. |

**Command age**, with two distinct reserved values:

| Value | Meaning |
|---|---|
| `0xFFFF` (`CMD_AGE_UNKNOWN`) | No valid `CONTROL` frame has **ever** arrived |
| `0xFFFE` (`CMD_AGE_MAX`) | Saturation ceiling for a real measurement |

"You have never spoken to me" is a wiring or bus-configuration problem.
"You stopped speaking 65 seconds ago" is something that died mid-mission.
Different diagnoses, so they must be different values.

### 3.4 Status indicator

URC requires a status light: **red = autonomous operation, blue =
teleoperation, flashing green = successful arrival**. The Jetson commands it and
the MCU represents it, through two fields:

| Field | Frame | Byte | Meaning |
|---|---|---|---|
| `indicator_request` | `CONTROL` | 6 | What the Jetson **asks** the light to show |
| `indicator_state` | `TELEM_STATUS` | 7 | What the MCU is **actually** showing |

Both use the same values:

| Value | Name | Meaning |
|---|---|---|
| `0` | `INDICATOR_OFF` | Not operating: `DISABLED`, or the watchdog has tripped. Also the neutral "no special request" |
| `1` | `INDICATOR_BLUE` | Teleoperation (`MANUAL`) |
| `2` | `INDICATOR_RED` | Autonomous operation |
| `3` | `INDICATOR_GREEN_FLASH` | Autonomous arrival at a target |

**The two fields can differ, on purpose.** The light exists so a judge can see
what the rover is *actually* doing, and the MCU knows its own mode. So the
Jetson's request is honoured only when it agrees with that mode; otherwise the
MCU overrides it. The rules, checked top to bottom, first match wins
(`RoverController::indicatorState()`):

| # | Condition | Shown | Request |
|---|---|---|---|
| 1 | Watchdog tripped | `OFF` | ignored |
| 2 | Mode `DISABLED` | `OFF` | ignored |
| 3 | Mode `MANUAL` | `BLUE` | ignored — a human-driven rover never shows red or green |
| 4 | Mode `AUTONOMOUS` | `GREEN_FLASH` if green was requested, else `RED` | only green matters |

`stop` does not change the light: a paused autonomous rover is still in
autonomous operation, so it stays red.

**Unknown values, two directions, two different answers.**

- **`indicator_request` (Jetson → MCU): the whole frame is rejected**, exactly
  like an undefined `mode` (§3.1). The sender disagrees with us about the
  protocol, so the frame does not refresh the command watchdog and
  `PROTOCOL_ERROR` is raised. Rejecting a command is safe: the rover stops.
- **`indicator_state` (MCU → Jetson): the frame is kept; only that field
  decodes as `None`.** The same status frame carries `fault_status` and
  `cmd_age_ms`. Dropping it over a cosmetic field would blind the operator to
  real faults, so rejecting telemetry is *not* the safe choice.

## 4. What CAN provides, so we do not

| Concern | v0.2 (UART) | v0.3 (CAN) |
|---|---|---|
| Message boundaries | start byte + length byte | hardware |
| Corruption detection | our CRC-16 bit loop | hardware 15-bit CRC |
| Corruption recovery | slide one byte, rescan | hardware auto-retransmit, µs |
| Addressing | none | 11-bit identifier |
| Priority | none | non-destructive arbitration |
| A faulty node | corrupts the link | goes bus-off, isolates itself |

**The checks we still owe.** CAN proves a frame arrived *intact*; it cannot
prove the sender agrees on what the bytes *mean*, and it cannot catch
corruption that happens in the software path after the CRC has passed. So
every decoder validates:

1. **DLC** — a mismatch is the cheap signal that a peer is on a different
   protocol version.
2. **`mode`** — an undefined value is the same class of error (section 3.1).

Critically, a rejected frame **does not refresh the command watchdog**, or a
mismatched node could keep the rover alive while sending commands it never
understood. `PROTOCOL_ERROR` is raised so the resulting stop is diagnosable.

## 5. Safety

### 5.1 The fail-safe rule

Four triggers — **jetson heartbeat lost**, **explicit `stop`**, **any mode that
does not positively permit motion**, and **C2 lost while in `MANUAL`** — all
force a stop through a single function, `RoverController::effectiveStop()`.
There is exactly one place in the codebase where "should this rover be moving?"
is answered.

The first and fourth are deliberately different. Jetson heartbeat loss stops
the rover in every mode, because nothing is driving it. C2 loss stops it only in
`MANUAL`, because an autonomous Jetson can keep driving without the base station
(section 3.1).

That third trigger is a **whitelist**, and that matters. The original version
asked `mode == MODE_DISABLED` and stopped only then, so every undefined mode
value read as "not disabled" and permitted full throttle — a safety predicate
that failed *open*. It now asks `modePermitsMotion()`, so anything the
firmware does not positively recognise as drivable means stop. Found by
@k1ngsyph1ll1is in [issue #4](https://github.com/schradivarius/URC-task3/issues/4). `commandedOutputs()` then
zeroes drive and steer *before* they reach the motor layer, so a future edit to
`setMotorOutputs()` cannot accidentally act on a stale command.

If no valid `CONTROL` frame arrives within `WATCHDOG_TIMEOUT_MS` (300 ms), the
controller stops, raises `JETSON_HEARTBEAT_LOST` and keeps reporting `cmd_age_ms` so the
staleness is visible frame by frame.

The Jetson separately considers the *link* down after 500 ms with no telemetry.
That is informational only; the rover's safety never depends on the Jetson
noticing anything.

### 5.2 Surviving a firmware fault

A watchdog that covers only the *link* leaves the larger hole: if the firmware
itself hangs, the outputs keep their last value and nothing is left running to
notice.

1. **Hardware watchdog** (`WDT_T4`, 1 s), fed once per loop. A hang resets the
   Teensy, driving outputs to a safe state.
2. **Boot-fault reporting.** The reset is otherwise invisible to the Jetson
   except as a telemetry gap — indistinguishable from a flaky bus. So the next
   boot reads `SRC_SRSR`, and if the last reset was the watchdog it raises
   `FIRMWARE_FAULT` for that whole session and says so in the startup banner.
3. **Bounded counters.** Encoder accumulators pass through `wrapI32()`. In
   Python an unwrapped counter raised inside `struct.pack`; in C++ signed
   overflow is **undefined behaviour**, which is worse — no exception, just a
   compiler free to do anything. CI runs the suite under UBSan to prove it.

> **Honest difference from v0.2.** There is no try/except guard, because
> Arduino builds run without exceptions. The hardware watchdog is the entire
> story for an unexpected fault, which is why arming it is not optional.

### 5.3 `millis()` wraps — a hazard v0.2 did not have

CircuitPython's `time.monotonic_ns()` is a 64-bit nanosecond counter that never
wraps in practice. Arduino's `millis()` is `uint32_t` and **wraps every ~49.7
days**.

Unsigned subtraction handles this correctly on its own: `(now - then)` is
computed modulo 2³², so it stays right across the wrap. What is *not* safe is
comparing timestamps directly (`now >= then + timeout`), which breaks the
moment the counter wraps past the deadline. **Every time comparison in
`rover_controller.cpp` is written as an elapsed-time subtraction.**
`test_controller.cpp` crosses the wrap boundary explicitly, and that test was
verified to *fail* against a naive timestamp-comparison implementation — it is
a real regression test, not decoration.

## 6. Timing constants

| Constant | Value | Notes |
|---|---|---|
| `WATCHDOG_TIMEOUT_MS` | 300 | Initial design value for testing (6× the 50 ms control period, tolerates 5 lost frames). Tunable. |
| `TELEMETRY_PERIOD_MS` | 50 | 20 Hz |
| `HW_WATCHDOG_MS` | 1000 | Teensy reset if the loop stalls |
| `LINK_TIMEOUT_S` (Jetson) | 0.5 | Informational only |
| `CONTROL_RATE_HZ` (Jetson) | 20 (50 ms) | Required heartbeat rate. Jetson must send CONTROL continuously, even when idle (drive=0). |
| `C2_timeout_s` (Jetson, `C2Monitor`) | 1.0 | Initial design value for testing. No base-station heartbeat for this long sets `c2_lost`. Longer than the 300 ms CAN watchdog because a radio drops packets far more often than a CAN bus. Tunable, and the base-station heartbeat rate is not yet defined. |

All placeholders, chosen to sit comfortably above one period plus margin. None
are derived from measured actuator response yet.

## 7. Demonstration and tests — no hardware required

```
make test     # 45 C++ tests + 20 host tests
make demo     # the message-exchange demonstration
```

`make demo` runs a live exchange against `tools/rover_sim`, then cuts the link
for 800 ms (well past the 300 ms watchdog) and restores it. Captured output:

```
-- B: link cut -- Jetson stops sending (800ms > 300ms watchdog) --
  t= 1.54s  cmd_age=309ms    enc_left=  +36550  current=  +0.00A  faults=JETSON_HEARTBEAT_LOST,C2_UNKNOWN
  t= 1.70s  cmd_age=459ms    enc_left=  +36550  current=  +0.00A  faults=JETSON_HEARTBEAT_LOST,C2_UNKNOWN
  t= 1.86s  cmd_age=659ms    enc_left=  +36550  current=  +0.00A  faults=JETSON_HEARTBEAT_LOST,C2_UNKNOWN
```

`cmd_age` climbs, `JETSON_HEARTBEAT_LOST` appears (with `C2_UNKNOWN`, since the
C2 bit is now stale), current falls to zero and the encoders freeze — then it
all clears by itself once `CONTROL` resumes.

Against real hardware, the same host script runs unchanged:

```
python3 host/jetson_test.py --channel can0 --bitrate 500000
```

## 8. One implementation, pinned across two languages

The safety logic exists **once**, in `firmware/src/rover_controller.cpp`. The
simulator the host tests run against is *that same source compiled natively* —
not a Python mock. A hand-written mock would recreate the exact failure this
project already had once: two copies of a safety rule, where the simulator
passes while the firmware misbehaves and the tests check the wrong copy.

The **codec** genuinely must exist twice, since the Teensy runs C++ and the
Jetson runs Python and neither can import the other. So it is pinned rather
than trusted: `tests/host/test_golden_vectors.py` compiles the C++ encoder,
runs it, and asserts Python produces identical bytes for identical inputs.
This was verified to fail on an injected endianness change.

## 9. Known placeholders

- **`drive_cmd`/`steer_cmd` scaling** is unitless ±1000; expect real units once
  motors and steering geometry are chosen.
- **Two encoders and one aggregate current** reading. A 6-wheel rover needs
  six, and one current figure cannot identify *which* wheel is stalling. Both
  change the payload when they grow — likely into per-corner frames, which CAN
  makes cheap.
- **Motor and sensor I/O is stubbed** behind `readSensors()` and
  `setMotorOutputs()` in `rover_firmware.ino`. Nothing above those functions
  should need to change when real hardware arrives.
- **`SRC_SRSR` watchdog bit mask is unverified on hardware.** It fails *safe*
  (an unrecognised reset cause reports nothing), but confirm it by deliberately
  hanging the loop once on the bench.
- **No sequence number.** CAN's retransmission covers corruption, but a counter
  would let either side measure genuine loss.
- **Not yet used:** CAN FD, the second and third CAN buses, and a dedicated
  high-priority e-stop frame in the reserved `0x000`–`0x0FF` block.

## 10. Repository layout and build

| Path | Runs on | Purpose |
|---|---|---|
| `firmware/rover_firmware.ino` | Teensy 4.1 | Hardware wiring only |
| `firmware/src/rover_protocol.*` | **Both** | Message definitions and codec |
| `firmware/src/rover_controller.*` | **Both** | Command/safety state machine |
| `host/rover_protocol.py` | Jetson | Python codec (pinned to the C++ one) |
| `host/can_link.py` | Jetson | Sim and python-can backends |
| `host/c2_link.py` | Jetson | C2 (base-station) link loss detection, and a fake link for testing |
| `host/jetson_test.py` | Jetson | Live host harness |
| `host/demo.py` | Jetson | The demonstration in section 7 |
| `tools/rover_sim.cpp` | dev machine | Simulator: real controller, fake plant |
| `tools/golden_vectors.cpp` | dev machine | Emits vectors for cross-language pinning |
| `tests/cpp/`, `tests/host/` | dev machine | 63 tests total |

### 10.1 Flashing the Teensy 4.1

1. Arduino IDE + Teensyduino, board **Teensy 4.1**.
2. Install **FlexCAN_T4** (`tonton81/FlexCAN_T4`) and **WDT_T4**
   (`tonton81/WDT_T4`).
3. Open `firmware/rover_firmware.ino` and upload. Sources under `src/` are
   compiled automatically.

**Before powering anything:**

- Teensy 4.1 is **3.3 V and NOT 5 V tolerant.** A 5 V sensor wired straight to
  a pin destroys the board. Level-shift everything.
- Each CAN bus needs a **transceiver**; the MCU cannot drive a differential bus.
- The bus needs **120 Ω termination at both physical ends.** Exactly one
  resistor, or three, is the most common CAN bring-up failure.
