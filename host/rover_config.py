"""
rover_config.py -- Every protocol and timing constant, in one place.

Mirrors firmware/src/rover_config.h. tests/host/test_config_sync.py fails if
the two ever disagree, so change both together.

Payload sizes are NOT repeated here: rover_protocol.py derives them from its
struct format strings, and the sync test checks those against the C++ values.
"""

# --- protocol version: bump whenever a message's layout or meaning changes ---
PROTOCOL_VERSION = 1

# --- link ---
CAN_BITRATE_HZ = 500000       # 500 kbps; every node must match

# --- timing ---
WATCHDOG_TIMEOUT_MS = 300     # controller stops after this long without CONTROL
TELEMETRY_PERIOD_MS = 50      # 20 Hz
CONTROL_RATE_HZ = 20          # Jetson-side send rate
LINK_TIMEOUT_S = 0.5          # Jetson-side, informational only

# --- CAN identifiers (also the bus priority: lowest id wins arbitration) ---
CAN_ID_CONTROL      = 0x100
CAN_ID_TELEM_MOTION = 0x200
CAN_ID_TELEM_STATUS = 0x201

# --- operating modes ---
MODE_DISABLED   = 0
MODE_MANUAL     = 1
MODE_AUTONOMOUS = 2
MODE_MAX        = MODE_AUTONOMOUS

# --- valid command range for drive_cmd / steer_cmd (tenths of a percent) ---
CMD_MIN = -1000
CMD_MAX = 1000
