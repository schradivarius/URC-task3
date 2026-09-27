"""
boot.py -- CircuitPython boot-time USB configuration for the rover controller.

Copy this onto the CIRCUITPY drive and POWER-CYCLE the board once (a soft
reset is not enough; boot.py only runs on hard reset).

Why this file exists
--------------------
By default CircuitPython's single USB serial device is the REPL console. If
the Jetson opens /dev/ttyACM0 and writes protocol frames there, the bytes go
to the Python interpreter, which tries to execute them. Enabling the data
channel adds a SECOND CDC endpoint -- usually /dev/ttyACM1 -- that carries
only our frames, while leaving the console available on the first one for
debugging. feather_main.py reads usb_cdc.data.

Verify after power-cycling:
    ls /dev/ttyACM*        # expect ttyACM0 (console) AND ttyACM1 (data)
    python3 jetson_test.py --port /dev/ttyACM1
"""

import usb_cdc

usb_cdc.enable(console=True, data=True)
