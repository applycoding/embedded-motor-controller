#include "motor/speed_estimator.hpp"

#include "motor/clamp.hpp"

namespace motor {

SpeedEstimator::SpeedEstimator(float counts_per_rev, float lpf_alpha)
    : counts_per_rev_(counts_per_rev), alpha_(clamp(lpf_alpha, 0.f, 1.f)) {}

void SpeedEstimator::reset(std::int32_t count) {
    prev_count_ = count;
    rpm_ = 0.f;
    initialized_ = true;
}

float SpeedEstimator::step(std::int32_t count, float dt_s) {
    if (!initialized_) {
        prev_count_ = count;
        initialized_ = true;
        rpm_ = 0.f;
        return rpm_;
    }
    if (dt_s <= 0.f || counts_per_rev_ <= 0.f) {
        prev_count_ = count;
        return rpm_;
    }

    const auto delta = static_cast<std::int32_t>(static_cast<std::uint32_t>(count) -
                                                 static_cast<std::uint32_t>(prev_count_));
    prev_count_ = count;

    const float raw = (static_cast<float>(delta) / counts_per_rev_) * (60.f / dt_s);
    rpm_ = alpha_ * raw + (1.f - alpha_) * rpm_;
    return rpm_;
}

}  // namespace motor
