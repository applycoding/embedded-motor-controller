#pragma once

namespace motor {

// Slews a speed command so a step setpoint cannot saturate the drive.
class RampLimiter {
 public:
    explicit RampLimiter(float max_rpm_per_s = 0.f);

    void set_rate(float max_rpm_per_s);
    // Places the limiter at `value` so the next step does not jump.
    void reset(float value);

    float step(float target, float dt_s);
    float value() const { return current_; }

 private:
    float max_rate_ = 0.f;
    float current_ = 0.f;
    bool initialized_ = false;
};

}  // namespace motor
