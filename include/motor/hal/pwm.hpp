#pragma once

namespace motor {
namespace hal {

// Signed duty cycle. +1 is full-scale forward, -1 is full-scale reverse.
// A real port drives a timer PWM channel and a direction pin (STM32 TIMx,
// or an RP2040 PWM slice).
class Pwm {
 public:
    virtual ~Pwm() = default;
    virtual void set_duty(float duty) = 0;
    virtual float duty() const = 0;
};

}  // namespace hal
}  // namespace motor
