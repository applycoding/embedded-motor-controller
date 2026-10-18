#pragma once

#include <cstdint>

namespace motor {

// Converts a quadrature count into RPM. Count wrap uses unsigned subtraction
// so a single sample that crosses the int32 boundary stays defined.
class SpeedEstimator {
 public:
    SpeedEstimator(float counts_per_rev, float lpf_alpha);

    void reset(std::int32_t count);
    float step(std::int32_t count, float dt_s);
    float rpm() const { return rpm_; }

 private:
    float counts_per_rev_ = 1.f;
    float alpha_ = 1.f;
    std::int32_t prev_count_ = 0;
    float rpm_ = 0.f;
    bool initialized_ = false;
};

}  // namespace motor
