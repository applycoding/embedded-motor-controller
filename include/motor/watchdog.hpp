#pragma once

#include <cstdint>

namespace motor {

// Software command watchdog. tick() once per control period. A timeout of 0
// disables it. Expiry latches until reset(); kick() does not clear a fault.
class Watchdog {
 public:
    explicit Watchdog(std::uint32_t timeout_ticks = 0);

    void set_timeout(std::uint32_t timeout_ticks);
    void reset();
    void kick();
    bool tick();
    bool expired() const { return expired_; }

 private:
    std::uint32_t timeout_ticks_ = 0;
    std::uint32_t ticks_since_kick_ = 0;
    bool expired_ = false;
};

}  // namespace motor
