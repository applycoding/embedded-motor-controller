#include "motor/ramp.hpp"

namespace motor {

RampLimiter::RampLimiter(float max_rpm_per_s) : max_rate_(max_rpm_per_s) {}

void RampLimiter::set_rate(float max_rpm_per_s) { max_rate_ = max_rpm_per_s; }

void RampLimiter::reset(float value) {
    current_ = value;
    initialized_ = true;
}

float RampLimiter::step(float target, float dt_s) {
    if (!initialized_ || max_rate_ <= 0.f || dt_s <= 0.f) {
        current_ = target;
        initialized_ = true;
        return current_;
    }

    const float max_step = max_rate_ * dt_s;
    const float delta = target - current_;
    if (delta > max_step) {
        current_ += max_step;
    } else if (delta < -max_step) {
        current_ -= max_step;
    } else {
        current_ = target;
    }
    return current_;
}

}  // namespace motor
