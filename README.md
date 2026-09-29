# URC Rover — Jetson ↔ Teensy CAN Link (Task 3)

Communication link between the rover's **onboard computer** (Jetson Orin Nano,
candidate) and its **embedded controller** (**Teensy 4.1**, C++ / Teensyduino),
over **CAN**.

Framed, prioritised CAN messages with watchdog-based fail-safes. Full
specification: **[PROTOCOL.md](PROTOCOL.md)**.

## Status

| | |
|---|---|
| Protocol, safety logic, firmware | Implemented, **39 tests** passing |
| Cross-language codec | Pinned byte-for-byte between C++ and Python |
| **Validated on real hardware** | **Not yet — see [Known gaps](#known-gaps)** |

---

## Quick start

**Nothing to install.** Needs a C++17 compiler and Python 3.9+, both of which
you already have. The host tests compile the *real firmware sources* rather
than mocking them, so there is no dependency to manage.

```bash
git clone https://github.com/schradivarius/URC-task3.git
cd URC-task3

make test     # 28 C++ tests + 11 host tests
make demo     # no-hardware message-exchange demonstration
```

If both succeed, your environment is good.

---

## Layout

Two languages, because the two ends of the link are different machines:

```
firmware/                    C++ — runs on the Teensy 4.1
  rover_firmware.ino           hardware wiring ONLY (CAN, watchdog, pins)
  src/rover_protocol.*         message definitions + codec   ─┐ pure C++,
  src/rover_controller.*       command/safety state machine  ─┘ no Arduino

host/                        Python — runs on the Jetson
  rover_protocol.py            codec (pinned to the C++ one)
  can_link.py                  sim + python-can backends
  jetson_test.py               live harness
  demo.py                      the demonstration

tools/
  rover_sim.cpp                simulator: REAL controller, fake plant
  golden_vectors.cpp           emits vectors for cross-language pinning

tests/cpp/                   28 tests — firmware core
tests/host/                  11 tests — golden vectors + integration
```

The two files under `firmware/src/` have **no Arduino dependency and no
hardware calls**, which is what lets the safety-critical logic be unit-tested
natively with a fake clock. `rover_firmware.ino` is only the wiring around
them. **If you are reviewing safety behaviour, read `rover_controller.h`.**

---

## Running the tests

```bash
make test        # everything
make test-cpp    # firmware core only — no Python needed
make test-host   # host side only — compiles the C++ core
```

| Suite | Tests | Answers |
|---|---:|---|
| `tests/cpp/test_protocol.cpp` | 11 | Does the wire format encode and decode correctly? |
| `tests/cpp/test_controller.cpp` | 17 | **Does the rover stop when it should?** |
| `tests/host/test_golden_vectors.py` | 5 | Do C++ and Python agree byte-for-byte? |
| `tests/host/test_integration.py` | 6 | Do both ends actually talk to each other? |

Every test is named for the failure it prevents, so the reason it exists
outlives anyone's memory of writing it. **Start a safety review at
`test_controller.cpp`.**

Three of these were verified to *fail* against a deliberately broken
implementation, rather than merely assumed to work:

- `watchdog_is_correct_across_millis_wraparound` — fails against a naive
  timestamp comparison
- `test_python_encoder_matches_cpp_byte_for_byte` — fails against an injected
  endianness flip
- `wrong_dlc_does_not_refresh_the_watchdog` — the version-mismatch guard

CI additionally builds with **clang** as a second toolchain and runs the suite
under **UBSan + ASan**, because signed overflow in the encoder accumulators
would be undefined behaviour rather than a wrap.

---

## Running against hardware

### A virtual CAN bus (no hardware, real CAN stack)

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
pip install python-can
python3 host/jetson_test.py --channel vcan0
```

### The real Teensy

1. Arduino IDE + Teensyduino, board **Teensy 4.1**.
2. Install **FlexCAN_T4** (`tonton81/FlexCAN_T4`) and **WDT_T4**
   (`tonton81/WDT_T4`).
3. Open `firmware/rover_firmware.ino` and upload.
4. `python3 host/jetson_test.py --channel can0 --bitrate 500000`

> **Before powering anything.**
> - Teensy 4.1 is **3.3 V and NOT 5 V tolerant.** A 5 V sensor on a pin
>   destroys the board. Level-shift everything.
> - Each CAN bus needs a **transceiver** — the MCU cannot drive a differential
>   bus directly.
> - The bus needs **120 Ω termination at both physical ends.** Exactly one
>   resistor, or three, is the single most common CAN bring-up failure.

The firmware prints a banner over USB serial on boot:

```
rover firmware up: CAN3 @ 500000 bps, hw watchdog 1000 ms
```

`RECOVERED FROM WATCHDOG RESET` appended means the previous run hung or
crashed — see `PROTOCOL.md` §5.2.

---

## Known gaps

Stated plainly, so nobody builds on an unchecked assumption:

- **Never run on a real Teensy or a real CAN bus.** Every library call was
  verified against the `FlexCAN_T4` and `WDT_T4` headers, but the firmware has
  not been compiled by Teensyduino or flashed. Bench validation is owed.
- **`SRC_SRSR` watchdog bit mask is unverified.** It fails safe — an
  unrecognised reset cause reports nothing — but confirm it by deliberately
  hanging the loop once on the bench.
- **Motor and sensor I/O is stubbed** behind `readSensors()` and
  `setMotorOutputs()`.
- **Two encoders, one aggregate current reading.** Both need to grow, and both
  change the payload when they do.
- **CAN FD, the second and third CAN buses, and a dedicated e-stop frame** are
  all unused. The `0x000`–`0x0FF` id block is reserved for that e-stop.

## CI

Every push runs the C++ suite under g++ **and** clang **and** sanitizers, plus
the host suite on Python 3.9 / 3.11 / 3.12 —
[`.github/workflows/ci.yml`](.github/workflows/ci.yml).

## License

GPL-3.0 — see [LICENSE](LICENSE).
