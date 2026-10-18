#pragma once

#include "motor/types.hpp"

#include <cstdint>

namespace motor {

enum class CommandId : std::uint8_t {
    SetRpm = 0,
    GetStatus,
    Start,
    Stop,
    ClearFault,
    Ping,
};

struct Command {
    CommandId id = CommandId::Ping;
    std::int32_t rpm = 0;
};

// One decoded line. payload is NUL terminated when ok is true.
// error points at a static string, or nullptr when ok is true.
struct Frame {
    bool ok = false;
    const char* error = "bad_frame";
    char payload[64] = {};
    int length = 0;
};

// XOR of every payload byte. The wire form is "*HH" uppercase hex.
std::uint8_t checksum(const char* data, int length);

// Writes "<payload>*HH\n" and a trailing NUL. Returns the byte count
// excluding the NUL, or -1 when the frame does not fit or the payload
// contains '*' or a line break.
int format_frame(char* dst, int capacity, const char* payload);

// "STATUS <MODE> <FAULT> <target_rpm> <measured_rpm> <duty>"
// RPM has one decimal place, duty has three. Returns length, or -1.
int format_status(char* dst, int capacity, Mode mode, FaultCode fault, float target_rpm,
                  float measured_rpm, float duty);

// Turns a checksum-valid payload into a command. On failure, error is a
// static string: "bad_frame", "bad_arg", or "unknown_command".
bool decode_command(const char* payload, int length, Command& command, const char*& error);

// Byte parser. No heap: the line sits in a fixed buffer. Feed bytes until
// feed() returns true, then inspect the frame (accepted or rejected).
class Parser {
 public:
    static constexpr int kMaxPayload = 60;

    void reset();
    bool feed(char byte, Frame& out);

 private:
    enum class Phase : std::uint8_t { Payload, CsHi, CsLo, WaitNl, Drop };

    Phase phase_ = Phase::Payload;
    char buf_[kMaxPayload] = {};
    int length_ = 0;
    int cs_hi_ = 0;
    std::uint8_t received_cs_ = 0;
};

}  // namespace motor
