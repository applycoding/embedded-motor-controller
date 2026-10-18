#pragma once

#include <cstdint>

namespace motor {
namespace hal {

// Quadrature count. Positive counts are forward. A real port reads a timer
// encoder interface (STM32 TIMx encoder mode) or a PIO / GPIO quadrature
// decoder on RP2040.
class Encoder {
 public:
    virtual ~Encoder() = default;
    virtual std::int32_t count() const = 0;
};

}  // namespace hal
}  // namespace motor
