#pragma once

namespace motor {

template <typename T>
constexpr T clamp(T value, T low, T high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

}  // namespace motor
