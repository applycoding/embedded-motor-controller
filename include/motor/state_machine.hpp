#pragma once

#include "motor/types.hpp"

namespace motor {

// IDLE --start--> RUN --stop--> IDLE
// IDLE or RUN --trip--> FAULT --clear_fault--> IDLE
// The first fault code sticks until it is cleared. START is refused in FAULT.
class StateMachine {
 public:
    Mode mode() const { return mode_; }
    FaultCode fault() const { return fault_; }

    bool start();
    bool stop();
    void trip(FaultCode code);
    bool clear_fault();

 private:
    Mode mode_ = Mode::Idle;
    FaultCode fault_ = FaultCode::None;
};

}  // namespace motor
