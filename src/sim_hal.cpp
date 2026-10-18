#include "motor/sim/sim_hal.hpp"

#include "motor/clamp.hpp"

namespace motor {
namespace sim {

void SimPwm::set_duty(float duty) { duty_ = clamp(duty, -1.f, 1.f); }

int SimUart::push(Ring& ring, const char* src, int len) {
    if (src == nullptr || len <= 0) {
        return 0;
    }
    int stored = 0;
    while (stored < len && ring.count < kCapacity) {
        ring.data[ring.head] = src[stored];
        ring.head = (ring.head + 1) % kCapacity;
        ++ring.count;
        ++stored;
    }
    return stored;
}

int SimUart::pop(Ring& ring, char* dst, int max_len) {
    if (dst == nullptr || max_len <= 0) {
        return 0;
    }
    int taken = 0;
    while (taken < max_len && ring.count > 0) {
        dst[taken] = ring.data[ring.tail];
        ring.tail = (ring.tail + 1) % kCapacity;
        --ring.count;
        ++taken;
    }
    return taken;
}

int SimUart::read(char* dst, int max_len) { return pop(rx_, dst, max_len); }

int SimUart::write(const char* src, int len) { return push(tx_, src, len); }

int SimUart::inject_rx(const char* src, int len) { return push(rx_, src, len); }

int SimUart::read_tx(char* dst, int max_len) { return pop(tx_, dst, max_len); }

}  // namespace sim
}  // namespace motor
