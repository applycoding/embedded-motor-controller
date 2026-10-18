#include "motor/state_machine.hpp"

namespace motor {

bool StateMachine::start() {
    if (mode_ != Mode::Idle) {
        return false;
    }
    mode_ = Mode::Run;
    return true;
}

bool StateMachine::stop() {
    if (mode_ != Mode::Run) {
        return false;
    }
    mode_ = Mode::Idle;
    return true;
}

void StateMachine::trip(FaultCode code) {
    if (code == FaultCode::None || mode_ == Mode::Fault) {
        return;
    }
    mode_ = Mode::Fault;
    fault_ = code;
}

bool StateMachine::clear_fault() {
    if (mode_ != Mode::Fault) {
        return false;
    }
    mode_ = Mode::Idle;
    fault_ = FaultCode::None;
    return true;
}

}  // namespace motor
