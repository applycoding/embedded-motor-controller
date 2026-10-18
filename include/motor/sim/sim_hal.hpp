#pragma once

#include "motor/hal/encoder.hpp"
#include "motor/hal/pwm.hpp"
#include "motor/hal/uart.hpp"

#include <cstdint>

namespace motor {
namespace sim {

class SimPwm : public hal::Pwm {
 public:
    void set_duty(float duty) override;
    float duty() const override { return duty_; }

 private:
    float duty_ = 0.f;
};

class SimEncoder : public hal::Encoder {
 public:
    std::int32_t count() const override { return count_; }
    void set_count(std::int32_t count) { count_ = count; }
    void add_counts(std::int32_t delta) { count_ += delta; }

 private:
    std::int32_t count_ = 0;
};

// Two fixed rings: host -> device (RX) and device -> host (TX).
class SimUart : public hal::Uart {
 public:
    static constexpr int kCapacity = 256;

    int read(char* dst, int max_len) override;
    int write(const char* src, int len) override;

    // Host injects bytes the device will read.
    int inject_rx(const char* src, int len);
    // Host pulls bytes the device wrote.
    int read_tx(char* dst, int max_len);

 private:
    struct Ring {
        char data[kCapacity] = {};
        int head = 0;
        int tail = 0;
        int count = 0;
    };

    static int push(Ring& ring, const char* src, int len);
    static int pop(Ring& ring, char* dst, int max_len);

    Ring rx_{};
    Ring tx_{};
};

}  // namespace sim
}  // namespace motor
