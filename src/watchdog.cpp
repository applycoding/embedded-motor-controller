#include "motor/watchdog.hpp"

namespace motor {

Watchdog::Watchdog(std::uint32_t timeout_ticks) : timeout_ticks_(timeout_ticks) {}

void Watchdog::set_timeout(std::uint32_t timeout_ticks) { timeout_ticks_ = timeout_ticks; }

void Watchdog::reset() {
    ticks_since_kick_ = 0;
    expired_ = false;
}

void Watchdog::kick() { ticks_since_kick_ = 0; }

bool Watchdog::tick() {
    if (timeout_ticks_ == 0 || expired_) {
        return expired_;
    }
    ++ticks_since_kick_;
    if (ticks_since_kick_ >= timeout_ticks_) {
        expired_ = true;
    }
    return expired_;
}

}  // namespace motor
