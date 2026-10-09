// rover_sim.cpp -- a simulated rover controller for host-side testing.
//
// WHY THIS IS C++ AND NOT PYTHON
//   The obvious thing would be a Python mock of the controller. That would
//   recreate the exact problem this project already fixed once: two
//   hand-maintained copies of a safety rule, where the simulator can pass
//   while the real firmware misbehaves, and the tests check the wrong copy.
//
//   So this links the REAL rover_controller.cpp -- the same source the Teensy
//   compiles. Only the plant (fake encoders, fake current and voltage) and the
//   transport (a pipe instead of a CAN bus) are simulated.
//
// PIPE PROTOCOL (stands in for the CAN bus)
//   stdin :  "<id_hex> <frame_hex>\n"   one received frame per line
//   stdout:  "<id_hex> <frame_hex>\n"   one transmitted frame per line
//   stdin closing ends the run.

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <chrono>
#include <string>
#include <poll.h>
#include <unistd.h>

#include "../firmware/src/rover_controller.h"

using namespace rover;

static std::chrono::steady_clock::time_point g_start;
static uint32_t realMillis() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now() - g_start).count());
}

static void emitFrame(uint32_t id, const uint8_t* buf, uint8_t len) {
    std::printf("%03x ", id);
    for (uint8_t i = 0; i < len; ++i) std::printf("%02x", buf[i]);
    std::printf("\n");
    std::fflush(stdout);        // unbuffered: the host reads these live
}

int main() {
    g_start = std::chrono::steady_clock::now();
    RoverController ctl(realMillis, DEFAULT_WATCHDOG_TIMEOUT_MS,
                        DEFAULT_TELEMETRY_PERIOD_MS);

    int32_t enc_left = 0, enc_right = 0;
    int16_t steer_fb = 0;
    std::string pending;
    char chunk[512];

    for (;;) {
        // Non-blocking stdin so the telemetry timer keeps running while the
        // host is quiet -- precisely the condition the command watchdog exists
        // to detect.
        struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
        if (poll(&pfd, 1, 2) > 0 && (pfd.revents & (POLLIN | POLLHUP))) {
            ssize_t n = read(STDIN_FILENO, chunk, sizeof(chunk));
            if (n <= 0) break;                       // host closed the pipe
            pending.append(chunk, static_cast<size_t>(n));
            size_t nl;
            while ((nl = pending.find('\n')) != std::string::npos) {
                std::string line = pending.substr(0, nl);
                pending.erase(0, nl + 1);
                unsigned id = 0; char hex[32] = {0};
                if (std::sscanf(line.c_str(), "%x %31s", &id, hex) == 2) {
                    uint8_t rx[8] = {0};
                    size_t len = std::strlen(hex) / 2;
                    if (len > 8) len = 8;
                    for (size_t i = 0; i < len; ++i) {
                        unsigned byte = 0;
                        std::sscanf(hex + i * 2, "%2x", &byte);
                        rx[i] = static_cast<uint8_t>(byte);
                    }
                    ctl.ingestFrame(id, rx, static_cast<uint8_t>(len));
                }
            }
        }

        int16_t drive, steer;
        ctl.commandedOutputs(drive, steer);          // zeroed under stop
        const bool stopped = ctl.effectiveStop();

        // Fake plant. The fail-safe has already been applied by
        // commandedOutputs(), so there is no second copy of the stop rule.
        enc_left  = wrapI32(static_cast<int64_t>(enc_left)  + drive / 10);
        enc_right = wrapI32(static_cast<int64_t>(enc_right) + drive / 10);
        if (steer_fb < steer) {
            steer_fb = static_cast<int16_t>((steer_fb + 25 < steer) ? steer_fb + 25 : steer);
        } else if (steer_fb > steer) {
            steer_fb = static_cast<int16_t>((steer_fb - 25 > steer) ? steer_fb - 25 : steer);
        }
        const int16_t current_ca = stopped ? 0 : static_cast<int16_t>(std::abs(drive) * 3 / 2);
        // Fake pack voltage: 24.00 V nominal, sagging a little under load.
        const int16_t voltage_cv = static_cast<int16_t>(2400 - std::abs(drive) / 40);

        if (ctl.telemetryDue()) {
            const uint8_t seq = ctl.telemetrySeq();   // one seq for all four
            uint8_t buf[8];

            TelemetryDriveL dl = ctl.buildDriveL(enc_left, steer_fb);
            emitFrame(CAN_ID_TELEM_DRIVE_L, buf, encodeTelemetryDriveL(dl, seq, buf));

            TelemetryDriveR dr = ctl.buildDriveR(enc_right);
            emitFrame(CAN_ID_TELEM_DRIVE_R, buf, encodeTelemetryDriveR(dr, seq, buf));

            TelemetryPower pw = ctl.buildPower(current_ca, voltage_cv);
            emitFrame(CAN_ID_TELEM_POWER, buf, encodeTelemetryPower(pw, seq, buf));

            // controller_health is NOT_REPORTED because no motor-driver
            // telemetry is wired up yet -- an honest "nobody told us" rather
            // than a misleading OK. c2_link the controller derives itself from
            // the c2_lost bit the Jetson forwards; see c2LinkState().
            TelemetryState st = ctl.buildState();
            emitFrame(CAN_ID_TELEM_STATE, buf, encodeTelemetryState(st, seq, buf));
        }
    }
    return 0;
}
