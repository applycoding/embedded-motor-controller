#include "motor/faults.hpp"

namespace motor {
namespace {

float abs_f(float value) { return value < 0.f ? -value : value; }

}  // namespace

FaultDetector::FaultDetector(FaultConfig config) : config_(config) {}

void FaultDetector::reset() {
    fault_ = FaultCode::None;
    overcurrent_samples_ = 0;
    stall_samples_ = 0;
}

FaultCode FaultDetector::update(float rpm, float duty, float current_a, bool motion_enabled) {
    if (fault_ != FaultCode::None) {
        return fault_;
    }

    if (config_.overcurrent_count > 0 && abs_f(current_a) > config_.overcurrent_a) {
        ++overcurrent_samples_;
    } else {
        overcurrent_samples_ = 0;
    }
    if (config_.overcurrent_count > 0 && overcurrent_samples_ >= config_.overcurrent_count) {
        fault_ = FaultCode::Overcurrent;
        return fault_;
    }

    const bool stalled = motion_enabled && abs_f(duty) >= config_.stall_duty &&
                         abs_f(rpm) <= config_.stall_rpm;
    if (config_.stall_count > 0 && stalled) {
        ++stall_samples_;
    } else {
        stall_samples_ = 0;
    }
    if (config_.stall_count > 0 && stall_samples_ >= config_.stall_count) {
        fault_ = FaultCode::Stall;
    }
    return fault_;
}

}  // namespace motor
