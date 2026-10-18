#include "motor/protocol.hpp"

#include <cstring>

namespace motor {
namespace {

int hex_nibble(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

char hex_digit(int value) {
    static constexpr char kDigits[] = "0123456789ABCDEF";
    return kDigits[value & 0xF];
}

bool append_char(char* dst, int capacity, int& length, char c) {
    if (length + 1 >= capacity) {
        return false;
    }
    dst[length++] = c;
    dst[length] = '\0';
    return true;
}

bool append_str(char* dst, int capacity, int& length, const char* text) {
    if (text == nullptr) {
        return false;
    }
    for (const char* p = text; *p != '\0'; ++p) {
        if (!append_char(dst, capacity, length, *p)) {
            return false;
        }
    }
    return true;
}

bool append_uint(char* dst, int capacity, int& length, int value) {
    char tmp[16];
    int n = 0;
    if (value <= 0) {
        tmp[n++] = '0';
    } else {
        while (value > 0 && n < static_cast<int>(sizeof(tmp))) {
            tmp[n++] = static_cast<char>('0' + (value % 10));
            value /= 10;
        }
    }
    while (n > 0) {
        if (!append_char(dst, capacity, length, tmp[--n])) {
            return false;
        }
    }
    return true;
}

bool append_fixed(char* dst, int capacity, int& length, float value, int decimals) {
    if (decimals < 0 || decimals > 6) {
        return false;
    }
    float magnitude = value < 0.f ? -value : value;
    int scale = 1;
    for (int i = 0; i < decimals; ++i) {
        scale *= 10;
    }
    const double scaled_d = static_cast<double>(magnitude) * static_cast<double>(scale) + 0.5;
    if (scaled_d > 2000000000.0) {
        return false;
    }
    int scaled = static_cast<int>(scaled_d);
    const bool negative = value < 0.f && scaled != 0;
    if (negative && !append_char(dst, capacity, length, '-')) {
        return false;
    }
    const int whole = decimals == 0 ? scaled : scaled / scale;
    if (!append_uint(dst, capacity, length, whole)) {
        return false;
    }
    if (decimals == 0) {
        return true;
    }
    if (!append_char(dst, capacity, length, '.')) {
        return false;
    }
    int frac = scaled % scale;
    int place = scale;
    for (int i = 0; i < decimals; ++i) {
        place /= 10;
        const int digit = place > 0 ? frac / place : 0;
        if (place > 0) {
            frac %= place;
        }
        if (!append_char(dst, capacity, length, static_cast<char>('0' + digit))) {
            return false;
        }
    }
    return true;
}

bool same_text(const char* payload, int length, const char* literal) {
    const int literal_len = static_cast<int>(std::strlen(literal));
    if (literal_len != length) {
        return false;
    }
    return std::memcmp(payload, literal, static_cast<std::size_t>(length)) == 0;
}

bool parse_i32(const char* text, int length, std::int32_t& out) {
    if (text == nullptr || length <= 0) {
        return false;
    }
    int index = 0;
    int sign = 1;
    if (text[0] == '+' || text[0] == '-') {
        sign = text[0] == '-' ? -1 : 1;
        index = 1;
    }
    if (index >= length) {
        return false;
    }
    int digits = 0;
    std::int64_t acc = 0;
    for (; index < length; ++index) {
        const char c = text[index];
        if (c < '0' || c > '9') {
            return false;
        }
        ++digits;
        if (digits > 10) {
            return false;
        }
        acc = acc * 10 + (c - '0');
    }
    if (sign < 0) {
        if (acc > 2147483648LL) {
            return false;
        }
        out = static_cast<std::int32_t>(-acc);
        return true;
    }
    if (acc > 2147483647LL) {
        return false;
    }
    out = static_cast<std::int32_t>(acc);
    return true;
}

}  // namespace

std::uint8_t checksum(const char* data, int length) {
    std::uint8_t sum = 0;
    if (data == nullptr || length <= 0) {
        return 0;
    }
    for (int i = 0; i < length; ++i) {
        sum = static_cast<std::uint8_t>(sum ^ static_cast<std::uint8_t>(data[i]));
    }
    return sum;
}

int format_frame(char* dst, int capacity, const char* payload) {
    if (dst == nullptr || payload == nullptr || capacity <= 0) {
        return -1;
    }
    const int length = static_cast<int>(std::strlen(payload));
    if (length <= 0 || length > Parser::kMaxPayload) {
        return -1;
    }
    for (int i = 0; i < length; ++i) {
        if (payload[i] == '*' || payload[i] == '\n' || payload[i] == '\r') {
            return -1;
        }
    }
    // payload + '*' + two hex digits + '\n' + NUL
    if (capacity < length + 5) {
        return -1;
    }
    std::memcpy(dst, payload, static_cast<std::size_t>(length));
    const std::uint8_t sum = checksum(payload, length);
    dst[length] = '*';
    dst[length + 1] = hex_digit(sum >> 4);
    dst[length + 2] = hex_digit(sum);
    dst[length + 3] = '\n';
    dst[length + 4] = '\0';
    return length + 4;
}

int format_status(char* dst, int capacity, Mode mode, FaultCode fault, float target_rpm,
                  float measured_rpm, float duty) {
    if (dst == nullptr || capacity <= 0) {
        return -1;
    }
    dst[0] = '\0';
    int length = 0;
    if (!append_str(dst, capacity, length, "STATUS ") ||
        !append_str(dst, capacity, length, mode_name(mode)) ||
        !append_char(dst, capacity, length, ' ') ||
        !append_str(dst, capacity, length, fault_name(fault)) ||
        !append_char(dst, capacity, length, ' ') ||
        !append_fixed(dst, capacity, length, target_rpm, 1) ||
        !append_char(dst, capacity, length, ' ') ||
        !append_fixed(dst, capacity, length, measured_rpm, 1) ||
        !append_char(dst, capacity, length, ' ') || !append_fixed(dst, capacity, length, duty, 3)) {
        return -1;
    }
    return length;
}

bool decode_command(const char* payload, int length, Command& command, const char*& error) {
    command = Command{};
    if (payload == nullptr || length <= 0) {
        error = "bad_frame";
        return false;
    }
    if (same_text(payload, length, "GET_STATUS")) {
        command.id = CommandId::GetStatus;
        error = nullptr;
        return true;
    }
    if (same_text(payload, length, "START")) {
        command.id = CommandId::Start;
        error = nullptr;
        return true;
    }
    if (same_text(payload, length, "STOP")) {
        command.id = CommandId::Stop;
        error = nullptr;
        return true;
    }
    if (same_text(payload, length, "CLEAR_FAULT")) {
        command.id = CommandId::ClearFault;
        error = nullptr;
        return true;
    }
    if (same_text(payload, length, "PING")) {
        command.id = CommandId::Ping;
        error = nullptr;
        return true;
    }

    static constexpr char kPrefix[] = "SET_RPM ";
    constexpr int kPrefixLen = 8;
    if (length > kPrefixLen && std::memcmp(payload, kPrefix, kPrefixLen) == 0) {
        std::int32_t rpm = 0;
        if (!parse_i32(payload + kPrefixLen, length - kPrefixLen, rpm)) {
            error = "bad_arg";
            return false;
        }
        command.id = CommandId::SetRpm;
        command.rpm = rpm;
        error = nullptr;
        return true;
    }

    error = "unknown_command";
    return false;
}

void Parser::reset() {
    phase_ = Phase::Payload;
    length_ = 0;
    cs_hi_ = 0;
    received_cs_ = 0;
    buf_[0] = '\0';
}

bool Parser::feed(char byte, Frame& out) {
    out = Frame{};
    if (byte == '\r') {
        return false;
    }

    if (phase_ == Phase::Drop) {
        if (byte != '\n') {
            return false;
        }
        reset();
        out.ok = false;
        out.error = "bad_frame";
        return true;
    }

    if (phase_ == Phase::Payload) {
        if (byte == '\n') {
            reset();
            out.ok = false;
            out.error = "bad_frame";
            return true;
        }
        if (byte == '*') {
            if (length_ == 0) {
                phase_ = Phase::Drop;
                return false;
            }
            phase_ = Phase::CsHi;
            return false;
        }
        if (length_ >= kMaxPayload) {
            phase_ = Phase::Drop;
            return false;
        }
        buf_[length_++] = byte;
        return false;
    }

    if (phase_ == Phase::CsHi) {
        const int nibble = hex_nibble(byte);
        if (nibble < 0) {
            phase_ = Phase::Drop;
            if (byte == '\n') {
                reset();
                out.ok = false;
                out.error = "bad_frame";
                return true;
            }
            return false;
        }
        cs_hi_ = nibble;
        phase_ = Phase::CsLo;
        return false;
    }

    if (phase_ == Phase::CsLo) {
        const int nibble = hex_nibble(byte);
        if (nibble < 0) {
            phase_ = Phase::Drop;
            if (byte == '\n') {
                reset();
                out.ok = false;
                out.error = "bad_frame";
                return true;
            }
            return false;
        }
        received_cs_ = static_cast<std::uint8_t>((cs_hi_ << 4) | nibble);
        phase_ = Phase::WaitNl;
        return false;
    }

    // Phase::WaitNl
    if (byte != '\n') {
        phase_ = Phase::Drop;
        return false;
    }

    Frame ready;
    ready.length = length_;
    for (int i = 0; i < length_; ++i) {
        ready.payload[i] = buf_[i];
    }
    ready.payload[length_] = '\0';
    if (received_cs_ == checksum(buf_, length_)) {
        ready.ok = true;
        ready.error = nullptr;
    } else {
        ready.ok = false;
        ready.error = "bad_checksum";
    }
    reset();
    out = ready;
    return true;
}

}  // namespace motor
