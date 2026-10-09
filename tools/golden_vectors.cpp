// golden_vectors.cpp -- emit encoded frames and predicate tables as text, for
// cross-language pinning.
//
// The codec is the one thing that must exist in both C++ (Teensy) and Python
// (Jetson), because neither can import the other. This program prints what the
// C++ side produces; tests/host/test_golden_vectors.py computes the same in
// Python and asserts they are identical.
//
// That turns "we were careful to keep them in sync" into a CI failure the
// moment someone changes a field width, order, endianness, CRC or predicate on
// one side only.
//
// Line formats:
//   CRC8|<ascii>|<hex>                   known-answer CRC
//   FRAMECRC|<can_id>|<payload hex>|<hex>
//   SEQDELTA|<prev>,<cur>|<delta>
//   MODE|<value>|<known><motion>         two 0/1 digits
//   INDICATOR|<value>|<known>
//   <NAME>|<args>|<frame hex>            encoded frames

#include <cstdio>
#include "../firmware/src/rover_protocol.h"

using namespace rover;

static void emitHex(const char* name, const char* args,
                    const uint8_t* buf, uint8_t len) {
    std::printf("%s|%s|", name, args);
    for (uint8_t i = 0; i < len; ++i) std::printf("%02x", buf[i]);
    std::printf("\n");
}

int main() {
    uint8_t buf[8];
    char args[160];

    // --- CRC known-answer and id seeding --------------------------------
    const char* kav = "123456789";
    std::printf("CRC8|%s|%02x\n", kav, crc8(reinterpret_cast<const uint8_t*>(kav), 9));

    const uint8_t sample[6] = {1, 2, 3, 4, 5, 6};
    const uint32_t ids[] = {CAN_ID_CONTROL, CAN_ID_TELEM_DRIVE_L,
                            CAN_ID_TELEM_DRIVE_R, CAN_ID_TELEM_POWER,
                            CAN_ID_TELEM_STATE, 0x7FF, 0x000};
    for (uint32_t id : ids) {
        std::printf("FRAMECRC|%u|010203040506|%02x\n",
                    static_cast<unsigned>(id), frameCrc8(id, sample, 6));
    }

    // --- sequence arithmetic across the wrap -----------------------------
    const uint8_t seq_pairs[][2] = {{0,1},{1,1},{254,255},{255,0},{254,1},{0,255},{200,10}};
    for (const auto& sp : seq_pairs) {
        std::printf("SEQDELTA|%u,%u|%u\n", sp[0], sp[1], seqDelta(sp[0], sp[1]));
    }

    // --- validation predicates, exhaustive over the byte -----------------
    for (int m = 0; m <= 255; ++m) {
        std::printf("MODE|%d|%d%d\n", m,
                    isKnownMode(static_cast<uint8_t>(m)) ? 1 : 0,
                    modePermitsMotion(static_cast<uint8_t>(m)) ? 1 : 0);
    }
    for (int i = 0; i <= 255; ++i) {
        std::printf("INDICATOR|%d|%d\n", i,
                    isKnownIndicator(static_cast<uint8_t>(i)) ? 1 : 0);
    }

    // --- CONTROL ---------------------------------------------------------
    struct Ctl { int16_t d, s; uint8_t m; bool stop, abort, ret, c2; uint8_t ind, seq; };
    const Ctl ctls[] = {
        {     0,     0, MODE_DISABLED,   true,  false, false, false, INDICATOR_OFF,        0},
        {   500,  -200, MODE_MANUAL,     false, false, false, false, INDICATOR_TELEOP,     1},
        {  1000, -1000, MODE_AUTONOMOUS, false, false, false, false, INDICATOR_AUTONOMOUS, 42},
        { -1000,  1000, MODE_MANUAL,     true,  true,  true,  true,  INDICATOR_ARRIVED,  255},
        {-32768, 32767, MODE_MANUAL,     false, false, true,  false, INDICATOR_FAULT,    128},
        {-21846,   170, MODE_AUTONOMOUS, false, true,  false, true,  INDICATOR_OFF,        7},
        {   250,   -50, MODE_AUTONOMOUS, false, false, false, true,  INDICATOR_AUTONOMOUS, 64},
        {     0,     0, MODE_MANUAL,     false, false, false, true,  INDICATOR_TELEOP,    65},
    };
    for (const Ctl& c : ctls) {
        ControlMsg m;
        m.drive_cmd = c.d; m.steer_cmd = c.s; m.mode = c.m;
        m.stop = c.stop; m.autonomy_abort = c.abort; m.return_request = c.ret;
        m.c2_lost = c.c2; m.indicator_request = c.ind; m.seq = c.seq;
        uint8_t n = encodeControl(m, buf);
        std::snprintf(args, sizeof(args), "%d,%d,%u,%d,%d,%d,%d,%u,%u",
                      c.d, c.s, c.m, c.stop ? 1 : 0, c.abort ? 1 : 0,
                      c.ret ? 1 : 0, c.c2 ? 1 : 0, c.ind, c.seq);
        emitHex("CONTROL", args, buf, n);
    }

    // --- TELEM_DRIVE_L ---------------------------------------------------
    struct DL { long long enc; int16_t fb; uint8_t seq; };
    const DL dls[] = {{0,0,0}, {123456,-1000,1}, {INT32_MAX_V,1000,250},
                      {INT32_MIN_V,-1,251}, {2147483648LL,0,9}};
    for (const DL& d : dls) {
        TelemetryDriveL m; m.enc_left = wrapI32(d.enc); m.steer_fb = d.fb;
        uint8_t n = encodeTelemetryDriveL(m, d.seq, buf);
        std::snprintf(args, sizeof(args), "%lld,%d,%u", d.enc, d.fb, d.seq);
        emitHex("DRIVE_L", args, buf, n);
    }

    // --- TELEM_DRIVE_R ---------------------------------------------------
    struct DR { long long enc; uint16_t age; uint8_t seq; };
    const DR drs[] = {{0,0,0}, {-123456,400,3}, {INT32_MAX_V,CMD_AGE_MAX,4},
                      {0,CMD_AGE_UNKNOWN,5}};
    for (const DR& d : drs) {
        TelemetryDriveR m; m.enc_right = wrapI32(d.enc); m.cmd_age_ms = d.age;
        uint8_t n = encodeTelemetryDriveR(m, d.seq, buf);
        std::snprintf(args, sizeof(args), "%lld,%u,%u", d.enc, d.age, d.seq);
        emitHex("DRIVE_R", args, buf, n);
    }

    // --- TELEM_POWER -----------------------------------------------------
    struct PW { int16_t ca, cv; uint16_t f; uint8_t seq; };
    const PW pws[] = {
        {0, 0, 0, 0},
        {-2500, 2400, FAULT_OVER_CURRENT, 6},
        {32767, -32768, 0xFFFF, 7},
        {-10000, 1200, static_cast<uint16_t>(FAULT_COMM_TIMEOUT | FAULT_SEQ_GAP
                                             | FAULT_CRC_ERROR), 8},
    };
    for (const PW& w : pws) {
        TelemetryPower m; m.current_ca = w.ca; m.voltage_cv = w.cv;
        m.fault_status = w.f;
        uint8_t n = encodeTelemetryPower(m, w.seq, buf);
        std::snprintf(args, sizeof(args), "%d,%d,%u,%u", w.ca, w.cv, w.f, w.seq);
        emitHex("POWER", args, buf, n);
    }

    // --- TELEM_STATE -----------------------------------------------------
    struct ST { uint8_t mode, jl, c2, h, ind, seq; };
    const ST sts[] = {
        {MODE_DISABLED, LINK_LOST, LINK_NOT_REPORTED, CTRL_HEALTH_NOT_REPORTED,
         INDICATOR_OFF, 0},
        {MODE_MANUAL, LINK_OK, LINK_OK, CTRL_HEALTH_OK, INDICATOR_TELEOP, 11},
        {MODE_AUTONOMOUS, LINK_DEGRADED, LINK_LOST, CTRL_HEALTH_DEGRADED,
         INDICATOR_AUTONOMOUS, 12},
        {MODE_AUTONOMOUS, LINK_OK, LINK_NOT_REPORTED, CTRL_HEALTH_FAULT,
         INDICATOR_FAULT, 13},
    };
    for (const ST& s : sts) {
        TelemetryState m; m.mode = s.mode; m.jetson_link = s.jl; m.c2_link = s.c2;
        m.controller_health = s.h; m.indicator_state = s.ind; m.reserved = 0;
        uint8_t n = encodeTelemetryState(m, s.seq, buf);
        std::snprintf(args, sizeof(args), "%u,%u,%u,%u,%u,%u",
                      s.mode, s.jl, s.c2, s.h, s.ind, s.seq);
        emitHex("STATE", args, buf, n);
    }
    return 0;
}
