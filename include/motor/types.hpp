#pragma once

#include "motor/detail/exhaustive.hpp"

#include <cstdint>

namespace motor {

enum class Mode : std::uint8_t {
    Idle = 0,
    Run = 1,
    Fault = 2,
};

enum class FaultCode : std::uint8_t {
    None = 0,
    Overcurrent = 1,
    Stall = 2,
    Watchdog = 3,
};

inline const char* mode_name(Mode mode) {
    switch (mode) {
        case Mode::Idle:
            return "IDLE";
        case Mode::Run:
            return "RUN";
        case Mode::Fault:
            return "FAULT";
        default:
            detail::unhandled_enum(mode);
            return "";
    }
}

inline const char* fault_name(FaultCode fault) {
    switch (fault) {
        case FaultCode::None:
            return "NONE";
        case FaultCode::Overcurrent:
            return "OVERCURRENT";
        case FaultCode::Stall:
            return "STALL";
        case FaultCode::Watchdog:
            return "WATCHDOG";
        default:
            detail::unhandled_enum(fault);
            return "";
    }
}

}  // namespace motor
