#pragma once

#include "motor/faults.hpp"
#include "motor/hal/encoder.hpp"
#include "motor/hal/pwm.hpp"
#include "motor/hal/uart.hpp"
#include "motor/pid.hpp"
#include "motor/protocol.hpp"
#include "motor/ramp.hpp"
#include "motor/speed_estimator.hpp"
#include "motor/state_machine.hpp"
#include "motor/watchdog.hpp"

#include <cstdint>

namespace motor {

struct ControllerConfig {
    PidGains gains{};
    PidLimits limits{};
    float max_rpm_per_s = 4000.f;
    float counts_per_rev = 8192.f;
    float speed_lpf_alpha = 0.45f;
    float max_abs_rpm = 5000.f;
    FaultConfig faults{};
    // Control periods without a SET_RPM or PING before a watchdog fault.
    // 0 disables the watchdog. GET_STATUS does not count as a kick.
    std::uint32_t watchdog_timeout_ticks = 250;
};

// 1 kHz-style speed controller. tick() uses only stack and fixed buffers:
// no new, no std::string, no std::vector.
class SpeedController {
 public:
    SpeedController(hal::Pwm& pwm, hal::Encoder& encoder, hal::Uart& uart, ControllerConfig config);

    SpeedController(const SpeedController&) = delete;
    SpeedController& operator=(const SpeedController&) = delete;

    void tick(float dt_s, float current_a);

    Mode mode() const { return state_.mode(); }
    FaultCode fault() const { return state_.fault(); }
    float commanded_rpm() const { return target_rpm_; }
    float measured_rpm() const { return measured_rpm_; }
    float duty() const { return pwm_.duty(); }

 private:
    void service_uart();
    void handle_frame(const Frame& frame);
    void handle_command(const Command& command);
    void reply(const char* payload);
    void reply_status();
    void enter_run();

    hal::Pwm& pwm_;
    hal::Encoder& encoder_;
    hal::Uart& uart_;
    ControllerConfig config_;
    Pid pid_;
    RampLimiter ramp_;
    SpeedEstimator estimator_;
    FaultDetector faults_;
    Watchdog watchdog_;
    StateMachine state_;
    Parser parser_;
    float target_rpm_ = 0.f;
    float measured_rpm_ = 0.f;
};

}  // namespace motor
