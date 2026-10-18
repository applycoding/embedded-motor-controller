#include "motor/pid.hpp"

#include "motor/clamp.hpp"

namespace motor {

Pid::Pid(PidGains gains, PidLimits limits) : gains_(gains), limits_(limits) {}

void Pid::set_gains(PidGains gains) { gains_ = gains; }

void Pid::set_limits(PidLimits limits) { limits_ = limits; }

void Pid::reset() {
    integral_ = 0.f;
    prev_error_ = 0.f;
    last_output_ = 0.f;
    has_prev_ = false;
}

float Pid::step(float setpoint, float measured, float dt_s) {
    const float error = setpoint - measured;

    const bool saturated_high = last_output_ >= limits_.out_max && error > 0.f;
    const bool saturated_low = last_output_ <= limits_.out_min && error < 0.f;
    if (!saturated_high && !saturated_low && dt_s > 0.f) {
        integral_ += gains_.ki * error * dt_s;
        integral_ = clamp(integral_, limits_.i_min, limits_.i_max);
    }

    float derivative = 0.f;
    if (has_prev_ && dt_s > 0.f) {
        derivative = gains_.kd * (error - prev_error_) / dt_s;
    }

    const float output =
        clamp(gains_.kp * error + integral_ + derivative, limits_.out_min, limits_.out_max);

    prev_error_ = error;
    has_prev_ = true;
    last_output_ = output;
    return output;
}

}  // namespace motor
