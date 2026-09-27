# URC Rover — Jetson ↔ Embedded Controller Link (Task 3)

Communication link between the rover's **onboard computer** (Jetson Orin Nano,
candidate) and its **embedded controller** (Adafruit Feather RP2040 RFM9x,
candidate, running CircuitPython).

Provides framed, CRC-checked CONTROL and TELEMETRY messages with watchdog-based
fail-safes. Full specification: **[PROTOCOL.md](PROTOCOL.md)**.

## Status

| | |
|---|---|
| Protocol, parser, safety logic | Implemented, 67 tests passing |
| Firmware | Written and tested against stubbed hardware |
| **Validated on the real board** | **Not yet — see [Known gaps](#known-gaps)** |

---

## Quick start

**No installation required.** The tests and both demos use only the Python
standard library (Python 3.9+). `pip` is needed only to talk to real hardware.

```bash
git clone https://github.com/schradivarius/URC-task3.git
cd URC-task3

python3 -m unittest discover -s tests -v   # 67 tests, ~0.5s
python3 demo.py                            # the message-exchange demonstration
python3 jetson_test.py --mock --duration 5 # live host harness, no hardware
```

If all three succeed, your environment is good.

---

## Running the tests

```bash
python3 -m unittest discover -s tests -v   # verbose: names every test
python3 -m unittest discover -s tests      # quiet: just "Ran 67 tests ... OK"
```

Works from the repository root or from inside `tests/`.

### One file or one test at a time

```bash
python3 -m unittest tests.test_protocol -v      # wire format + safety logic (45)
python3 -m unittest tests.test_firmware -v      # the flashed firmware (17)
python3 -m unittest tests.test_integration -v   # end-to-end over a mock link (5)

python3 -m unittest tests.test_protocol.TestControllerSafety -v
python3 -m unittest tests.test_protocol.TestCRC.test_known_answer_vector -v
```

`pytest tests/` also works if you prefer it — these are plain
`unittest.TestCase` classes — but `unittest` is the supported path and needs
nothing installed.

### What the suite covers

| Group | Tests | Answers |
|---|---:|---|
| `TestCRC` | 4 | Is the checksum the algorithm the spec claims? |
| `TestRoundTrip` | 5 | Does what we encode decode back identically? |
| `TestParserRobustness` | 10 | Does the parser survive a hostile byte stream? |
| `TestRangeHandling` | 6 | Do values at their numeric limits behave? |
| `TestControllerSafety` | 13 | **Does the rover stop when it should?** |
| `TestNoDuplicatedProtocol` | 7 | Is there still only one copy of the protocol? |
| `TestFirmwareLoop` | 8 | Does the main loop behave while running? |
| `TestFirmwareSensorStubs` | 3 | Is the hardware boundary safe? |
| `TestFirmwareStartup` | 6 | Do the boot and fault paths work? |
| `TestEndToEnd` | 5 | Do both sides actually talk to each other? |

Every test is named for the failure it prevents, so the reason it exists
survives longer than anyone's memory of writing it. Start a safety review at
`TestControllerSafety` and `TestEndToEnd`.

`tests/fake_hardware.py` stubs CircuitPython's `board`, `busio`, `digitalio`,
`microcontroller` and `usb_cdc`, so `feather_main.py` — the exact file that
gets flashed — is imported and driven in CI rather than being the one file no
test touches.

---

## Running the demos

### `demo.py` — the Task 3 demonstration

```bash
python3 demo.py
```

Three self-asserting parts (non-zero exit if any fail):

- **Part A** — a corrupted payload byte is caught by the CRC, and the parser
  resynchronizes on the very next valid frame.
- **Part C** — a corrupted `LEN` byte costs **one byte** of resync instead of
  260. Prints the ~1182 ms blackout the naive version would have suffered,
  against the 300 ms watchdog.
- **Part B** — a live 20 Hz exchange with a deliberate 600 ms link cut. Watch
  `cmd_age` climb, `COMM_TIMEOUT` appear, current fall to `+0.00A` and the
  encoders freeze — then everything clear by itself when commands resume.

### `jetson_test.py --mock` — the live host harness

```bash
python3 jetson_test.py --mock --duration 5
```

The same script you point at real hardware, talking instead to `mcu_sim.py`
in-process over a software loopback. Prints a live status line:

```
[t=  2.5s] link=UP  sent=  51 rx=  51 enc=( +8700, +8700) steer_fb=-109
           current=  +4.50A cmd_age=46ms faults=none
```

`cmd_age=never` on the first line is expected — the controller had not yet
received a valid CONTROL frame when it sent that telemetry. Over a clean mock
link the closing stats should read `crc_errors: 0, bad_headers: 0`.

---

## Running against real hardware

Full instructions: **[PROTOCOL.md §10.2](PROTOCOL.md)**. In short:

1. Flash CircuitPython for the **Feather RP2040 RFM9x**
   ([download](https://circuitpython.org/board/adafruit_feather_rp2040_rfm9x/)).
2. Copy four files onto `CIRCUITPY`:
   `feather_main.py`→`code.py`, plus `framing.py`, `controller.py`, `boot.py`.
3. **Power-cycle the board.** `boot.py` only runs on hard reset; a soft reset
   will not create the USB data channel. This is the step people miss.
4. Then:

```bash
pip install -r requirements.txt            # pyserial
ls /dev/ttyACM*                            # expect ttyACM0 AND ttyACM1
python3 jetson_test.py --port /dev/ttyACM1 --duration 10
```

**Use `ttyACM1`, not `ttyACM0`.** On CircuitPython the first USB serial device
is the REPL console — frames written there are fed to the Python interpreter,
not to the firmware.

### Troubleshooting

| Symptom | Cause |
|---|---|
| Only `ttyACM0` exists | `boot.py` missing or no hard reset. Re-copy and power-cycle. |
| No telemetry, no errors | Talking to the console endpoint. Use `ttyACM1`. |
| `hw watchdog = UNAVAILABLE` in the banner | That CircuitPython build exposes no watchdog; the exception guard is your only backstop. Worth fixing before the rover moves under power. |
| `ModuleNotFoundError: framing` | `framing.py` and `controller.py` were not copied onto `CIRCUITPY`. |

The firmware prints a banner to the console endpoint on boot:

```
rover firmware up: link = usb_cdc.data (second CDC endpoint), hw watchdog = on
```

---

## Repository layout

| File | Runs on | Purpose |
|---|---|---|
| `PROTOCOL.md` | — | The specification. Source of truth for the wire format. |
| `framing.py` | **Both** | CRC, framing, encode/decode, stream parser |
| `controller.py` | **Both** | Command/safety state machine: watchdog, stop rule, pacing |
| `feather_main.py` | Feather | The firmware — flash as `code.py` |
| `boot.py` | Feather | Enables the USB CDC data channel |
| `jetson_test.py` | Host | Host-side test script — real serial or mock |
| `mock_link.py` | Host | In-memory duplex loopback with fault injection |
| `mcu_sim.py` | Host | Demo-only fake plant driving the real `controller.py` |
| `demo.py` | Host | The three-part demonstration |
| `tests/` | Host | 67 tests, no dependencies |

`framing.py` and `controller.py` run on **both** sides. Nothing
safety-critical is duplicated, and `TestNoDuplicatedProtocol` fails the build
if a second copy reappears.

---

## Known gaps

Honest status, so nobody builds on an assumption that has not been checked:

- **Never run on the real board.** The pinout, `usb_cdc` behaviour and the
  watchdog are correct per Adafruit's CircuitPython board definition, but
  bench validation is outstanding. The suite proves the logic and the
  fail-safe paths, not the enumeration or actuator timing.
- **The I/O budget does not close.** GPIO16–23 are consumed by the RFM9x
  module and are not broken out, leaving roughly 13 usable GPIO and 4 ADC
  channels against about 29 pins of need for a 6-wheel rover with 4-corner
  steering — before the arm and science payload. See `PROTOCOL.md` §9.1.
- **The RFM9x radio is unused.** No driver, and no transport profile. Note
  that 20 Hz is not achievable over LoRa (~40–60 ms airtime per packet), so
  the radio needs its own rate and watchdog settings.
- **Motor and sensor I/O is stubbed.** `read_sensors()` and
  `set_motor_outputs()` in `feather_main.py` are placeholders behind
  `SIMULATE_SENSORS`.
- **Two encoders, one aggregate current reading.** Both need to grow, and
  both change the TELEMETRY payload when they do.

## CI

Every push runs the suite, both demos, each test module directly, and
`compileall`, across Python 3.9 / 3.11 / 3.12 — see
[`.github/workflows/ci.yml`](.github/workflows/ci.yml).

## License

GPL-3.0 — see [LICENSE](LICENSE).
