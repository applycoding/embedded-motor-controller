#include "motor/sim/plant.hpp"

#include "motor/clamp.hpp"

#include <cstdint>

namespace motor {
namespace sim {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// Toward-zero truncation without libm, so this file can be cross-compiled
// with -ffreestanding. Shaft steps stay well inside int32.
std::int32_t trunc_i32(float value) { return static_cast<std::int32_t>(value); }

}  // namespace

DcMotorPlant::DcMotorPlant(PlantParams params) : params_(params) {}

void DcMotorPlant::reset() {
    current_a_ = 0.f;
    omega_rad_s_ = 0.f;
    count_accum_ = 0.f;
}

float DcMotorPlant::rpm() const { return omega_rad_s_ * (60.f / (2.f * kPi)); }

void DcMotorPlant::step(float duty, float dt_s, SimEncoder& encoder) {
    if (dt_s <= 0.f || params_.inductance_h <= 0.f || params_.inertia <= 0.f ||
        params_.counts_per_rev <= 0.f) {
        return;
    }

    const float voltage = clamp(duty, -1.f, 1.f) * params_.bus_voltage;
    constexpr float kSubstep = 50.0e-6f;
    int substeps = static_cast<int>(dt_s / kSubstep);
    if (substeps < 1) {
        substeps = 1;
    }
    if (substeps > 2000) {
        substeps = 2000;
    }
    const float h = dt_s / static_cast<float>(substeps);

    float delta_theta = 0.f;
    for (int i = 0; i < substeps; ++i) {
        const float di = (voltage - params_.resistance_ohm * current_a_ -
                          params_.back_emf * omega_rad_s_) /
                         params_.inductance_h;
        current_a_ += di * h;
        const float torque = params_.torque_const * current_a_ - params_.viscous * omega_rad_s_;
        omega_rad_s_ += (torque / params_.inertia) * h;
        delta_theta += omega_rad_s_ * h;
    }

    count_accum_ += delta_theta * (params_.counts_per_rev / (2.f * kPi));
    const std::int32_t whole = trunc_i32(count_accum_);
    count_accum_ -= static_cast<float>(whole);
    if (whole != 0) {
        encoder.add_counts(whole);
    }
}

}  // namespace sim
}  // namespace motor
