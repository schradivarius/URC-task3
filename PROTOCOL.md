# Rover Onboard Computer ↔ Embedded Controller Protocol (v0.5)

**Scope:** Onboard computer (Jetson Orin Nano, candidate) ↔ embedded controller
(**Teensy 4.1**, NXP i.MX RT1062, C++ / Teensyduino), over **CAN**.

> **v0.4 — structured command and telemetry packets.** Added a sequence number
> and an application-layer CRC-8 in both directions, plus the autonomy-abort,
> return-request and status-indicator fields, voltage, operating-mode echo,
> per-link health and low-level controller health. `fault_status` widened to
> `uint16`. Telemetry became **four frames** carrying one shared sequence
> number.
>
> **v0.5 — C2 and the status indicator, reconciled.** v0.4 was written against
> a `main` that predated two merged changes (PRs #13 and #15), so it left the
> C2 link as a placeholder and used its own indicator policy. v0.5 folds both
> in and nothing is a placeholder any more:
>
> * `c2_lost` is a real CONTROL field again, at **flags bit 6**, so the Jetson
>   forwards its verdict on a link the controller cannot see.
> * `FAULT_C2_LINK_LOST` claims the `0x0080` bit v0.4 had reserved for it.
> * `TELEM_STATE.c2_link` is populated from that bit, and goes
>   `LINK_NOT_REPORTED` when the value is stale. That replaces v0.3.1's
>   receiver-side `C2_UNKNOWN` convention: the staleness rule now lives in the
>   **sender**, so a receiver that forgets it cannot be misled (§3.5).
> * The indicator follows the controller's own mode, per v0.3.1's rule table,
>   with `INDICATOR_FAULT` added for the cases where nothing is driving the
>   rover (§3.5). v0.4 forced `FAULT` on *any* fault bit, which would have
>   taken the light away from red mid-autonomy over an undervoltage warning.
>
> Full field tables with types, units, ranges, meanings and update rates are in
> section 3.

---

## 1. Design goals

- **No hardware dependency for development.** The protocol, the safety logic
  and the full exchange run and are tested on a laptop with no board, no CAN
  interface and no drivers (section 7).
- **Fail-safe on silence**, and fail *closed* on anything unrecognized.
- **End-to-end protected**, not merely wire-protected (section 4).
- **One implementation of the safety rule**, with the codec pinned across the
  two languages that must both speak it (section 8).
- **Portable across CAN flavours.** Every frame is 8 bytes, so this runs on
  Classic CAN or CAN FD, alongside Classic-only motor controllers.

## 2. Link basics

| Parameter | Value |
|---|---|
| Physical layer | CAN 2.0B, differential twisted pair, **120 Ω termination at both ends** |
| Bit rate | 500 kbps (placeholder; 1 Mbps is fine at these frame sizes) |
| Bus | Teensy 4.1 **CAN3** (FD-capable) — CAN1/CAN2 work unchanged |
| Frame format | Standard, 11-bit identifiers, **every frame DLC 8** |
| CONTROL rate | 20 Hz (1 frame per cycle) |
| TELEMETRY rate | 20 Hz (4 frames per cycle = 80 frames/s) |
| Byte order | Little-endian, written byte-by-byte (not struct-punned) |

At 500 kbps an 8-byte frame is roughly 220 µs of airtime. Five frames per cycle
is ~1.1 ms of a 50 ms period — about **2% bus utilisation**. Throughput is
nowhere near the constraint; CAN was chosen for determinism and noise immunity,
not speed. Say so plainly when anyone asks.

### 2.1 Identifiers *are* priorities

CAN arbitration is bitwise and dominant-low, so the **numerically lowest id
wins the bus** and the loser backs off without its message being corrupted.
This table is a priority ordering, not just a set of names.

| ID | Message | Direction | DLC | Rate |
|---|---|---|---|---|
| `0x000`–`0x0FF` | *reserved* — headroom for a dedicated e-stop frame | — | — | — |
| **`0x100`** | `CONTROL` | Jetson → controller | 8 | 20 Hz |
| **`0x200`** | `TELEM_DRIVE_L` | controller → Jetson | 8 | 20 Hz |
| **`0x201`** | `TELEM_DRIVE_R` | controller → Jetson | 8 | 20 Hz |
| **`0x202`** | `TELEM_POWER` | controller → Jetson | 8 | 20 Hz |
| **`0x203`** | `TELEM_STATE` | controller → Jetson | 8 | 20 Hz |

Commands outrank telemetry because a late command can hurt the rover and late
telemetry only annoys an operator.

### 2.2 Frame envelope

Every frame, both directions, has the same shape:

```
 byte:  0    1    2    3    4    5      6      7
      +----+----+----+----+----+----+-------+-------+
      |     6-byte message payload     |  SEQ  | CRC8  |
      +----+----+----+----+----+----+-------+-------+
```

| Field | Type | Units | Range | Meaning |
|---|---|---|---|---|
| `seq` | uint8 | frames | 0–255, **wraps** | Increments once per transmitted cycle. Receiver uses the delta: 1 healthy, 0 duplicate, >1 frames lost. |
| `crc8` | uint8 | — | 0–255 | CRC-8/SAE-J1850 over the CAN id then bytes 0–6. See section 4. |

**Why four telemetry frames and not one CAN FD frame.** 6 payload bytes × 4
frames = 24 bytes, which is what the field list needs. One 64-byte FD frame
would hold it all atomically, but would require FD-capable transceivers on
every node of the bus including Classic-only motor controllers. The field
definitions would not change if you later move telemetry to FD.

**Snapshot tearing.** All four telemetry frames of one cycle carry the **same
`seq`**. The Jetson therefore knows whether the four frames it holds came from
one cycle or straddle two, and can discard or flag a torn snapshot instead of
silently mixing a fresh encoder reading with a stale fault word.
`host/jetson_test.py` counts torn snapshots; a clean link produces zero.

## 3. Message definitions

### 3.0 Requirement coverage

Every field the Task 3 brief enumerates, and where it lives. Nothing in either
direction is "to be added later".

**Jetson → MCU command packet** — one structured message, `CONTROL` (§3.1):

| Required field | Where | Type |
|---|---|---|
| sequence number | `CONTROL` byte 6 | uint8 |
| operating mode | `CONTROL` byte 4 | uint8 enum |
| desired drive command | `CONTROL` bytes 0–1 | int16 |
| desired steering command | `CONTROL` bytes 2–3 | int16 |
| stop command | `flags` bit 0 | bool |
| autonomy-abort flag | `flags` bit 1 | bool |
| return-request flag | `flags` bit 2 | bool |
| status-indicator request | `flags` bits 3–5 | uint8 enum |
| checksum / CRC | `CONTROL` byte 7 | uint8 CRC-8 |

**MCU → Jetson telemetry packet** — one logical packet, sent as four frames of
a shared cycle (§2.2, §3.2–3.5):

| Required field | Where | Type |
|---|---|---|
| sequence number | byte 6 of all four frames | uint8 |
| encoder values | `TELEM_DRIVE_L` / `_R` bytes 0–3 | int32 ×2 |
| steering feedback | `TELEM_DRIVE_L` bytes 4–5 | int16 |
| current status | `TELEM_POWER` bytes 0–1 | int16 |
| voltage status | `TELEM_POWER` bytes 2–3 | int16 |
| current operating mode | `TELEM_STATE` byte 0 | uint8 enum |
| fault flags | `TELEM_POWER` bytes 4–5 | uint16 bitfield |
| command age | `TELEM_DRIVE_R` bytes 4–5 | uint16 |
| Jetson-link health | `TELEM_STATE` byte 1 | uint8 enum |
| C2-link state (as forwarded) | `TELEM_STATE` byte 2 | uint8 enum |
| low-level controller health | `TELEM_STATE` byte 3 | uint8 enum |
| status-indicator state | `TELEM_STATE` byte 4 | uint8 enum |
| checksum / CRC | byte 7 of all four frames | uint8 CRC-8 |

**Every field in both directions updates at 20 Hz**, the rate of its frame. The
rate is per frame, not per field: there is no slow-changing field sent less
often, because at 8 bytes a frame the bus is 2% utilised (§2) and a second rate
would only add a staleness question nobody needs to ask.

### 3.1 `CONTROL` — `0x100`, DLC 8, 20 Hz, Jetson → controller

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–1 | `drive_cmd` | int16 | 0.1 % of full drive effort | −1000 … +1000 | Desired drive. Positive is forward. Values outside the range are **not** clamped by the protocol; the motor layer owns that. |
| 2–3 | `steer_cmd` | int16 | 0.1 % of full steering range | −1000 … +1000 | Desired steering. Positive is right. |
| 4 | `mode` | uint8 | enum | **0, 1, 2 only** | `0=DISABLED, 1=MANUAL, 2=AUTONOMOUS`. Any other value **rejects the frame** — see 3.1.1. |
| 5 | `flags` | uint8 | bitfield | see below | Five fields packed; see 3.1.2. |
| 6 | `seq` | uint8 | frames | 0–255 wraps | See 2.2. |
| 7 | `crc8` | uint8 | — | 0–255 | See 4. |

#### 3.1.1 `mode` validation

`mode` is a uint8 carrying 256 possible values where three are defined. Two
separate questions follow, and conflating them was a real bug:

| Question | Answer |
|---|---|
| Is this value *defined*? (`isKnownMode`) | `0`, `1`, `2`. **`DISABLED` is valid** — a legitimate command meaning "do not move". |
| May the rover *move*? (`modePermitsMotion`) | `MANUAL` or `AUTONOMOUS` only — a **whitelist**. |

An undefined mode **rejects the whole frame**, exactly like a bad CRC: the
sender disagrees with us about the protocol. Deliberately **not**
forward-compatible — a newer peer sending a mode this firmware does not
implement must stop this rover. Adding a fourth mode means updating every
controller on the bus *before* a host may send it.

#### 3.1.2 `flags` byte (byte 5)

| Bits | Field | Type | Range | Meaning |
|---|---|---|---|---|
| 0 | `stop` | bool | 0/1 | Stop now, regardless of `mode`. Takes effect on the cycle it arrives — no coast frame. Separate from `mode` so an e-stop can be asserted **and released** without a mode round trip. |
| 1 | `autonomy_abort` | bool | 0/1 | Abandon the current autonomous task. **Forces a stop while `mode == AUTONOMOUS`**; in `MANUAL` the operator is already driving, so it has no autonomous task to abort and does not stop. ⚠️ *This behaviour is a judgement call, not spelled out by the requirement — flagged for review.* |
| 2 | `return_request` | bool | 0/1 | Begin the return-to-base behaviour. Carried and reported; the controller takes no low-level action on it. |
| 3–5 | `indicator_request` | uint8 | **0–4** | Requested status-indicator state. `0=OFF, 1=TELEOP, 2=AUTONOMOUS, 3=ARRIVED, 4=FAULT`. 5–7 are reachable on the wire and **reject the frame**. The controller honours this only where it agrees with its own mode — see §3.5. |
| 6 | `c2_lost` | bool | 0/1 | `1` = the Jetson has lost the base-station (C2) link. The Jetson's report on a link the controller cannot see — see below. |
| 7 | *reserved* | — | **must be 0** | A set bit **rejects the frame**: a newer sender is using a field we cannot interpret, so the rest of the byte is untrustworthy. |

#### 3.1.3 `c2_lost` — the one field the controller cannot determine itself

The controller detects a silent Jetson on its own (the command watchdog), but
it has **no radio** and therefore no view of the base station ↔ Jetson link. So
the Jetson watches that link (`C2Monitor` in `host/c2_link.py`: no base-station
heartbeat within `C2_timeout_s`, default 1.0 s) and forwards the verdict in
every `CONTROL` frame.

**The two link failures are deliberately not equivalent.** C2 loss with a
healthy Jetson is *expected* on the autonomy course, where line of sight to the
base station drops while onboard autonomy keeps running.

| Mode | `c2_lost = 1` → | Why |
|---|---|---|
| `MANUAL` | **stop** | The operator's commands can no longer arrive, so nobody is driving |
| `AUTONOMOUS` | **keep driving** | The Jetson is driving, and is provably alive or the command watchdog would already have stopped the rover |
| `DISABLED` | stopped anyway | — |

`FAULT_C2_LINK_LOST` is raised in **every** mode: during an autonomous run it
is the only sign that the rover is out of contact, which an operator needs
whether or not it changes the driving. The per-mode *stop* policy is an initial
choice and may be revised; the reporting is not negotiable.

Two implementation details that exist to stop this field lying:

- **`encode_control()` has no default for `c2_lost`.** A caller that forgot it
  would silently report the link as healthy, which is the one direction of
  error that lets a rover keep driving in `MANUAL` with nobody able to reach
  it. A missing argument is a loud failure instead.
- **The controller boots with `c2_lost = 1`** and holds it until a valid frame
  says otherwise, matching every other boot default (assume the worst). It is
  only ever *visible* through `c2LinkState()`, which reports
  `LINK_NOT_REPORTED` until a real frame arrives, so the boot default cannot
  masquerade as forwarded state.

### 3.2 `TELEM_DRIVE_L` — `0x200`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–3 | `enc_left` | int32 | encoder ticks | full int32, **wraps** | Left-side cumulative count. **Relative only** — difference consecutive readings and handle rollover. |
| 4–5 | `steer_fb` | int16 | 0.1 % of full steering range | −1000 … +1000 | Measured steering position, same scale as `steer_cmd`. |

### 3.3 `TELEM_DRIVE_R` — `0x201`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–3 | `enc_right` | int32 | encoder ticks | full int32, **wraps** | Right-side cumulative count. |
| 4–5 | `cmd_age_ms` | uint16 | milliseconds | 0 … 65533, plus 2 sentinels | Time since the last **valid** CONTROL frame. See below. |

| `cmd_age_ms` value | Meaning |
|---|---|
| 0 … 65533 | A real measurement |
| `0xFFFE` (`CMD_AGE_MAX`) | Saturation ceiling — "at least 65.534 s" |
| `0xFFFF` (`CMD_AGE_UNKNOWN`) | **No valid CONTROL frame has ever arrived** |

Two distinct values because "you have never spoken to me" is a wiring or
bus-config problem while "you stopped speaking 65 s ago" is something that died
mid-mission. Different diagnoses.

### 3.4 `TELEM_POWER` — `0x202`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0–1 | `current_ca` | **int16** | centiamps (1 cA = 10 mA) | −327.68 … +327.67 A | Motor current. **Signed**: a braking motor genuinely produces negative current, and unsigned would wrap it to a huge positive value — the worst kind of bad telemetry, because it looks plausible. |
| 2–3 | `voltage_cv` | **int16** | centivolts (1 cV = 10 mV) | −327.68 … +327.67 V | Pack voltage. Signed for symmetry and to make a mis-wired sense line obvious rather than enormous. |
| 4–5 | `fault_status` | **uint16** | bitfield | see below | Active faults. |

**`fault_status`** — widened from `uint8` in v0.4, which had exactly one bit
left. Bits `0x0001`–`0x0040` keep their v0.3 values.

| Bit | Name | Meaning |
|---|---|---|
| `0x0001` | `COMM_TIMEOUT` | No valid CONTROL frame within the watchdog |
| `0x0002` | `OVER_CURRENT` | Current exceeded a threshold |
| `0x0004` | `ESTOP_ACTIVE` | Hardware or software e-stop engaged |
| `0x0008` | `ENCODER_FAULT` | Encoder reading invalid or stalled |
| `0x0010` | `UNDERVOLTAGE` | Supply voltage below threshold |
| `0x0020` | `FIRMWARE_FAULT` | This boot followed a watchdog reset (5.2) |
| `0x0040` | `PROTOCOL_ERROR` | A frame on our id was uninterpretable: wrong DLC, undefined mode, bad flags or bad indicator |
| `0x0080` | `C2_LINK_LOST` | The last `CONTROL` frame reported the base-station link lost (`c2_lost = 1`). Raised in every mode; stops the rover only in `MANUAL` (§3.1.3). Cleared, not latched, once the value goes stale — see §3.5 |
| `0x0100` | `SEQ_GAP` | CONTROL frames were lost, duplicated or reordered |
| `0x0200` | `CRC_ERROR` | Application-layer CRC mismatch on our id |
| `0x0400`–`0x8000` | *reserved* | |

`PROTOCOL_ERROR`, `CRC_ERROR` and `SEQ_GAP` all self-clear when a valid,
contiguous frame arrives, so fixing the sender clears the fault rather than
leaving it latched and misleading for the rest of the session. `C2_LINK_LOST`
clears the same way, and also clears when the value behind it goes stale (§3.5)
rather than asserting a link state the controller can no longer vouch for.

### 3.5 `TELEM_STATE` — `0x203`, DLC 8, 20 Hz

| Byte | Field | Type | Units | Valid range | Meaning |
|---|---|---|---|---|---|
| 0 | `mode` | uint8 | enum | 0–2 | **Echo** of the mode the controller believes it is in. Without it, a dropped or misread mode change is invisible from outside the rover. |
| 1 | `jetson_link` | uint8 | enum | 0–3 | Health of the **Jetson ↔ controller** link. |
| 2 | `c2_link` | uint8 | enum | 0–3 | Health of the **base-station C2** link, as *forwarded by the Jetson* (§3.1.3). Never derived from `jetson_link`. |
| 3 | `controller_health` | uint8 | enum | 0–3 | Aggregate low-level motor-driver / ESC health. |
| 4 | `indicator_state` | uint8 | enum | 0–4 | What the status indicator is **actually displaying**. |
| 5 | *reserved* | uint8 | — | transmit as 0 | |

**Link health enum** (`jetson_link`, `c2_link`):

| Value | Name | Meaning |
|---|---|---|
| 0 | `LINK_OK` | Healthy |
| 1 | `LINK_DEGRADED` | Intermittent: losses, gaps or CRC errors seen within the last 1000 ms |
| 2 | `LINK_LOST` | Down |
| 3 | `LINK_NOT_REPORTED` | **We do not know** — nobody has told us, or what we were told has gone stale |

**The two link fields are independent and must never be collapsed.** The 2027
autonomy course deliberately includes areas with no C2 line-of-sight while
onboard autonomy keeps working normally. Treating C2 loss as equivalent to
Jetson loss would stop the rover exactly where it is supposed to keep going.

`jetson_link` the controller measures for itself, from the command watchdog and
from sequence gaps or CRC errors seen within the last `LINK_DEGRADED_HOLD_MS`.

`c2_link` it cannot measure at all — it has no radio — so the field is only
ever the `c2_lost` bit the Jetson forwarded, mapped by `c2LinkState()`:

| Condition | `c2_link` | `FAULT_C2_LINK_LOST` |
|---|---|---|
| No valid `CONTROL` frame has ever arrived | `LINK_NOT_REPORTED` | clear |
| `JETSON_HEARTBEAT_LOST` — the forwarded value is **stale** | `LINK_NOT_REPORTED` | clear |
| Last frame said `c2_lost = 1` | `LINK_LOST` | **set** |
| Last frame said `c2_lost = 0` | `LINK_OK` | clear |

**The staleness rule lives in the sender, and that is the change from v0.3.1.**
The controller only knows C2 from the last `CONTROL` frame, so once the Jetson
goes silent that value is worthless: a clear bit from ten seconds ago does not
mean C2 is fine now, and a set bit does not mean it is still down. v0.3.1 put
that rule on the *receiver* — `describe_faults()` hid the stale bit and
substituted a `C2_UNKNOWN` label — which works only as long as every receiver
remembers to do it. A dashboard, a log parser or a second consumer that read
`fault_status` directly saw a stale claim as a live one. Now the controller
simply never transmits a value it cannot vouch for, so there is nothing for a
receiver to remember and nothing to get wrong. `LINK_NOT_REPORTED` is the
on-the-wire spelling of what `C2_UNKNOWN` used to mean.

**Controller health enum** (`controller_health`): `0=OK, 1=DEGRADED,
2=FAULT, 3=NOT_REPORTED`. Currently always `NOT_REPORTED` — no motor-driver
telemetry is wired up yet.

#### 3.5.1 The status indicator, and why request and state are two fields

URC requires a status light, and scores it on what the rover is **actually
doing**: **red = autonomous operation, blue = teleoperation, flashing green =
successful arrival**. The enum is named for the meanings rather than the
colours, because the colour is a wiring decision and the meaning is not:

| Value | Name | URC colour | Meaning |
|---|---|---|---|
| 0 | `INDICATOR_OFF` | off | Not operating. A legitimate idle state |
| 1 | `INDICATOR_TELEOP` | blue | Under operator control |
| 2 | `INDICATOR_AUTONOMOUS` | red | Driving itself |
| 3 | `INDICATOR_ARRIVED` | flashing green | Autonomous arrival at a target |
| 4 | `INDICATOR_FAULT` | *not red, blue or green* | Nothing is driving this rover |

`INDICATOR_FAULT` must be wired to something a judge cannot mistake for one of
the three scored colours — amber, or off with a separate annunciator. **Open
hardware question**, noted in §9.

**The two fields differ on purpose.** The controller knows its own mode, so the
Jetson's request is honoured only where it agrees with that mode; otherwise the
controller overrides it. Checked top to bottom, first match wins
(`RoverController::indicatorState()`):

| # | Condition | Shown | Request |
|---|---|---|---|
| 1 | Command watchdog tripped | `FAULT` | ignored |
| 2 | `MANUAL` with `c2_lost = 1` | `FAULT` | ignored |
| 3 | Mode `DISABLED` | `OFF` | ignored |
| 4 | Mode `MANUAL` | `TELEOP` | ignored — a human-driven rover never shows red or green |
| 5 | Mode `AUTONOMOUS` | `ARRIVED` if `ARRIVED` was requested, else `AUTONOMOUS` | only `ARRIVED` matters |

Rules 1 and 2 are the two ways nothing can be driving the rover, and they show
`FAULT` rather than `OFF` because `OFF` reads as a legitimate idle state.

**A fault that does not stop the rover does not touch the light.** An
undervoltage warning, an over-current trip or one lost frame leaves an
autonomous rover showing red, which is what URC requires it to show; those
conditions are reported in `fault_status`, which is where an operator reads
them. v0.4 forced `FAULT` on *any* set bit, so a single `SEQ_GAP` mid-course
would have blanked a correctly-operating autonomous rover's light.

`stop` also does not change the light: a paused autonomous rover is still under
autonomous operation, so it stays red.

Comparing `indicator_request` against `indicator_state` tells an operator
immediately whether the controller honoured the request, and rule 4 means a
disagreement is normal rather than alarming.

## 4. Integrity: two CRCs, covering different things

| Concern | Provided by |
|---|---|
| Message boundaries | CAN hardware |
| Wire corruption between the two CAN controllers | CAN hardware, 15-bit CRC |
| Corruption recovery | CAN hardware, automatic retransmit in µs |
| Priority / arbitration | CAN hardware, non-destructive |
| **Corruption in the software path** | **our CRC-8** |
| **Loss, duplication, reordering** | **our sequence number** |
| **Wrong-message-on-wrong-id** | **our CRC-8, via id seeding** |
| **Protocol version mismatch** | **our DLC, mode, flag and indicator validation** |

**Why an application-layer CRC when CAN already has one.** CAN's CRC is
computed by the transmitting controller and checked by the receiving
controller. It protects the **wire** between those two chips:

```
Jetson software → driver/DMA → [ CAN CRC covers this ] → driver/DMA → MCU software
   ^^^^^^^^^^^^^^^^^^^^^^^^^                              ^^^^^^^^^^^^^^^^^^^^^^^^
   covered only by our CRC-8                              covered only by our CRC-8
```

A bug assembling the struct, DMA corruption, or a driver copying into the wrong
buffer all produce a frame CAN considers perfectly valid and delivers with
wrong contents. Our CRC is computed by our code over our struct and checked by
our code on the far side, so it covers the whole path. This is the same
reasoning behind AUTOSAR's E2E protection, which also layers a CRC on top of
CAN's.

**Do not remove it as redundant.** It is not redundant; it covers a different
failure domain.

### 4.1 The algorithm

**CRC-8 / SAE-J1850** — what AUTOSAR E2E profiles 1 and 2 use.

| Parameter | Value |
|---|---|
| Polynomial | `0x1D` |
| Init | `0xFF` |
| Reflect in / out | No / No |
| Final XOR | `0xFF` |
| **Check value** (`"123456789"`) | **`0x4B`** |

Computed over **the CAN identifier (4 bytes, low byte first) followed by frame
bytes 0–6**, excluding the CRC byte itself.

**The id is in the seed on purpose** — AUTOSAR calls this a Data ID. Without
it, a correctly-CRC'd payload delivered on the wrong id would validate: a
telemetry frame mistakenly transmitted with `CONTROL`'s id would be accepted as
a command. With it, that fails.

**Porting note:** for a CRC-**8** the data byte is XORed into the whole
register (`crc ^= byte`); for a CRC-16 it goes into the high byte
(`crc ^= byte << 8`). Copying a CRC-16 loop and changing the width is the
classic way to produce a plausible-looking wrong answer. The check value is
pinned in `tests/cpp/test_protocol.cpp` and cross-checked against Python,
because this repo has already shipped a CRC documented as one variant and
implemented as another.

### 4.2 What a rejected frame does

A frame failing **any** validation — DLC, CRC, undefined `mode`, the reserved
flag bit set, or an out-of-range `indicator_request` — is discarded and **does
not refresh the command watchdog**. Otherwise a node
on a mismatched protocol version could keep the rover alive while sending
commands it never understood. The matching fault bit is raised so the
resulting stop is diagnosable rather than silent.

## 5. Safety

### 5.1 The fail-safe rule

Five triggers — **comm timeout**, **explicit `stop`**, **any mode that does not
positively permit motion**, **`autonomy_abort` while `AUTONOMOUS`**, and
**`c2_lost` while `MANUAL`** — all force a stop through a single function,
`RoverController::effectiveStop()`. There is exactly one place in the codebase
where "should this rover be moving?" is answered. `commandedOutputs()` then
zeroes drive and steer *before* they reach the motor layer, so a future edit to
`setMotorOutputs()` cannot accidentally act on a stale command.

The first and fifth are deliberately different. A silent Jetson stops the rover
in **every** mode, because nothing is driving it. C2 loss stops it only in
`MANUAL`, because an autonomous Jetson drives perfectly well without the base
station — and the autonomy course is laid out so that it has to (§3.1.3).

The mode test is a **whitelist**. An earlier version asked `mode ==
MODE_DISABLED` and stopped only then, so every undefined mode value read as
"not disabled" and permitted full throttle — a safety predicate that failed
*open*.

The Jetson separately considers the *link* down after 500 ms with no telemetry.
That is informational only; the rover's safety never depends on the Jetson
noticing anything.

### 5.2 Surviving a firmware fault

1. **Hardware watchdog** (`WDT_T4`, 1 s), fed once per loop. A hang resets the
   Teensy, driving outputs to a safe state.
2. **Boot-fault reporting.** The reset is otherwise invisible to the Jetson
   except as a telemetry gap — indistinguishable from a flaky bus. The next
   boot reads `SRC_SRSR` and, if the last reset was the watchdog, raises
   `FIRMWARE_FAULT` for that whole session and says so in the startup banner.
3. **Bounded counters.** Encoder accumulators pass through `wrapI32()`. In
   Python an unwrapped counter raised inside `struct.pack`; in C++ signed
   overflow is **undefined behaviour**, which is worse. CI runs the suite under
   UBSan to prove it.

> **Honest difference from the CircuitPython original:** there is no try/except
> guard, because Arduino builds run without exceptions. The hardware watchdog
> is the entire story for an unexpected fault, which is why arming it is not
> optional.

### 5.3 `millis()` wraps

Arduino's `millis()` is `uint32_t` and wraps every ~49.7 days. Unsigned
subtraction handles that correctly: `(now - then)` is computed modulo 2³².
Comparing timestamps directly (`now >= then + timeout`) does **not** — it
breaks the moment the counter wraps past the deadline. **Every time comparison
in `rover_controller.cpp` is written as an elapsed-time subtraction.** Sequence
numbers follow the same rule: `seqDelta()` subtracts, so `255 → 0` is a delta
of 1 rather than a 255-frame loss.

`test_controller.cpp` crosses both wrap boundaries explicitly, and the
`millis()` test was verified to *fail* against a naive timestamp-comparison
implementation. It is a real regression test, not decoration.

## 6. Timing constants

| Constant | Value | Notes |
|---|---|---|
| `CONTROL` rate | 20 Hz | 1 frame per cycle |
| `TELEMETRY` rate | 20 Hz | 4 frames per cycle |
| `WATCHDOG_TIMEOUT_MS` | 300 | 6× the control period |
| `TELEMETRY_PERIOD_MS` | 50 | |
| `LINK_DEGRADED_HOLD_MS` | 1000 | How long a gap or CRC error keeps the link `DEGRADED` |
| `HW_WATCHDOG_MS` | 1000 | Teensy reset if the loop stalls |
| `LINK_TIMEOUT_S` (Jetson) | 0.5 | Informational only |
| `C2_timeout_s` (Jetson, `C2Monitor`) | 1.0 | No base-station heartbeat for this long sets `c2_lost`. Longer than the 300 ms CAN watchdog because a radio drops packets far more often than a CAN bus. Tunable, and the base-station heartbeat rate is not yet defined |

All placeholders, chosen to sit comfortably above one period plus margin. None
are derived from measured actuator response yet.

## 7. Demonstration and tests — no hardware required

```
make test     # 68 C++ tests + 43 host tests
make demo     # the message-exchange demonstration
```

`make demo` runs a live exchange against `tools/rover_sim`, then exercises a
link cut, the stop flag, an autonomy abort, **C2 loss in each of the two modes
that differ**, a corrupted frame and a sequence gap. Captured output, trimmed
to one line per phase:

```
-- D2: C2 lost while AUTONOMOUS -- keeps driving, fault raised --
  t= 3.99s  seq= 80  enc=  +54580   +12.00A  23.80V  mode=AUTONOMOUS jet=OK   c2=LOST         ind=AUTONOMOUS age=20ms  faults=C2_LINK_LOST
-- D3: C2 lost while MANUAL -- stops, nobody can reach it --
  t= 4.50s  seq= 91  enc=  +63380    +0.00A  24.00V  mode=MANUAL     jet=OK   c2=LOST         ind=FAULT      age=20ms  faults=C2_LINK_LOST
-- E: corrupted payload (app-layer CRC must reject it) --
  t= 5.19s  seq=104  enc=  +63380    +0.00A  24.00V  mode=MANUAL     jet=LOST c2=NOT_REPORTED ind=FAULT      age=487ms faults=COMM_TIMEOUT,CRC_ERROR
-- F: sequence gap (frames 100 -> 140) --
  t= 5.38s  seq=108  enc=  +66420    +6.00A  23.90V  mode=MANUAL     jet=DEGRADED c2=OK       ind=TELEOP     age=98ms  faults=SEQ_GAP
```

Three things in that capture are the whole point of §3.1.3 and §3.5. In **D2**
an autonomous rover keeps drawing current with C2 lost, and the fault is
reported anyway. In **D3** the same loss in `MANUAL` zeroes the current. In
**E** the Jetson has gone silent, so `c2=NOT_REPORTED` and `C2_LINK_LOST` is
*absent* rather than latched: the controller stops asserting a C2 verdict it
can no longer vouch for.

Against real hardware the same host script runs unchanged:

```
python3 host/jetson_test.py --channel can0 --bitrate 500000
```

## 8. One implementation, pinned across two languages

The safety logic exists **once**, in `firmware/src/rover_controller.cpp`. The
simulator the host tests run against is *that same source compiled natively* —
not a Python mock. A hand-written mock would recreate the exact failure this
project already had: two copies of a safety rule, where the simulator passes
while the firmware misbehaves and the tests check the wrong copy.

The **codec** genuinely must exist twice, since the Teensy runs C++ and the
Jetson runs Python and neither can import the other. So it is pinned rather
than trusted. `tests/host/test_golden_vectors.py` compiles the C++ encoder,
runs it, and asserts Python produces identical output for:

- every encoded frame, byte for byte
- the CRC check value and the id-seeded frame CRC
- `seqDelta` across the wrap
- `isKnownMode` and `modePermitsMotion` for **all 256** values
- `isKnownIndicator` for **all 256** values
- `c2_lost` round-tripping in every mode, and bit 6 decoding as a field rather
  than as a reserved bit — the guard against a typo'd reserved mask quietly
  swallowing a live flag

Verified to fail on an injected endianness change.

## 9. Known placeholders

- **`drive_cmd`/`steer_cmd` scaling** is unitless ±1000; expect real units once
  motors and steering geometry are chosen.
- **Two encoders and one aggregate current reading.** A 6-wheel rover needs
  six, and one current figure cannot identify *which* wheel is stalling. Both
  change the payload when they grow — likely into per-corner frames, which CAN
  makes cheap.
- **`controller_health` is always `NOT_REPORTED`** — no motor-driver telemetry
  is wired up. On a CAN bus that arrives as the drivers' own messages.
- **`INDICATOR_FAULT` has no agreed colour.** It must not be red, blue or
  flashing green, since URC scores those three. Amber, or off plus a separate
  annunciator — a hardware decision nobody has made yet (§3.5.1).
- **`drive_cmd` / `steer_cmd` are not range-checked on decode.** The ±1000
  range is a contract, not an enforced one: the decoder accepts any int16 and
  the motor layer owns clamping. [PR #16](https://github.com/schradivarius/URC-task3/pull/16)
  adds command-range validation and is the right place to change that.
- **`return_request` has no controller-side behaviour.** Carried and reported
  only.
- **`autonomy_abort` stopping an autonomous rover is a judgement call** — see
  3.1.2. Flagged for review.
- **Motor and sensor I/O is stubbed** behind `readSensors()` and
  `setMotorOutputs()` in `rover_firmware.ino`.
- **`SRC_SRSR` watchdog bit mask is unverified on hardware.** It fails safe,
  but confirm it by deliberately hanging the loop once on the bench.
- **Not yet used:** CAN FD, the second and third CAN buses, and a dedicated
  high-priority e-stop frame in the reserved `0x000`–`0x0FF` block.
- **Mode names are being revised in parallel.**
  [PR #18](https://github.com/schradivarius/URC-task3/pull/18) renames
  `MODE_DISABLED`/`MODE_MANUAL` to `MODE_SAFE`/`MODE_TELEOP`, adds a
  non-commandable `MODE_FAULT = 3` and introduces `activeMode()`. No wire
  values change. Whichever lands second applies a mechanical rename; and
  `TELEM_STATE.mode` (§3.5) should then echo `activeMode()` rather than the
  commanded mode, which is strictly more informative and what #18 expects.

## 10. Repository layout and build

| Path | Runs on | Purpose |
|---|---|---|
| `firmware/rover_firmware.ino` | Teensy 4.1 | Hardware wiring only |
| `firmware/src/rover_protocol.*` | **Both** | Message definitions, CRC, codec |
| `firmware/src/rover_controller.*` | **Both** | Command/safety state machine |
| `host/rover_protocol.py` | Jetson | Python codec (pinned to the C++ one) |
| `host/can_link.py` | Jetson | Sim and python-can backends |
| `host/c2_link.py` | Jetson | C2 (base-station) link-loss detection, and a fake link for testing |
| `host/jetson_test.py` | Jetson | Live harness, snapshot reassembly |
| `host/demo.py` | Jetson | The demonstration in section 7 |
| `tools/rover_sim.cpp` | dev machine | Simulator: real controller, fake plant |
| `tools/golden_vectors.cpp` | dev machine | Emits vectors for cross-language pinning |
| `tests/cpp/`, `tests/host/` | dev machine | 111 tests total |

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
