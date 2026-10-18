#pragma once

#include "motor/types.hpp"

namespace motor {

struct FaultConfig {
    float overcurrent_a = 4.f;
    int overcurrent_count = 5;
    float stall_rpm = 40.f;
    float stall_duty = 0.7f;
    int stall_count = 200;
};

// Latches the first fault until reset(). A non-positive count disables that check.
// Stall is only armed when the caller says the axis is supposed to be moving.
class FaultDetector {
 public:
    explicit FaultDetector(FaultConfig config = {});

    void reset();
    FaultCode update(float rpm, float duty, float current_a, bool motion_enabled);
    FaultCode fault() const { return fault_; }

 private:
    FaultConfig config_{};
    FaultCode fault_ = FaultCode::None;
    int overcurrent_samples_ = 0;
    int stall_samples_ = 0;
};

}  // namespace motor
