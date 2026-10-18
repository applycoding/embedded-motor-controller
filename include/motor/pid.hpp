#pragma once

namespace motor {

struct PidGains {
    float kp = 0.f;
    float ki = 0.f;
    float kd = 0.f;
};

struct PidLimits {
    float out_min = -1.f;
    float out_max = 1.f;
    float i_min = -1.f;
    float i_max = 1.f;
};

// Speed-loop PID. The integrator freezes when the output is already hard
// against the rail and the error would push it further (clamping anti-windup).
// Error that pulls the output off the rail is still integrated.
class Pid {
 public:
    Pid() = default;
    Pid(PidGains gains, PidLimits limits);

    void set_gains(PidGains gains);
    void set_limits(PidLimits limits);
    void reset();

    // dt_s <= 0 updates the proportional term only.
    float step(float setpoint, float measured, float dt_s);

    float integral() const { return integral_; }
    float last_output() const { return last_output_; }

 private:
    PidGains gains_{};
    PidLimits limits_{};
    float integral_ = 0.f;
    float prev_error_ = 0.f;
    float last_output_ = 0.f;
    bool has_prev_ = false;
};

}  // namespace motor
