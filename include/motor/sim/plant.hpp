#pragma once

#include "motor/sim/sim_hal.hpp"

namespace motor {
namespace sim {

// Brushed DC motor: electrical RL plus a rigid inertia and viscous friction.
// Steady state at the default parameters is about V = 0.012 * omega, so
// 1200 rpm (125.7 rad/s) sits near 1.51 V, or duty 0.13 on a 12 V bus.
struct PlantParams {
    float resistance_ohm = 1.0f;
    float inductance_h = 0.0005f;
    float torque_const = 0.01f;
    float back_emf = 0.01f;
    float inertia = 1.0e-5f;
    float viscous = 2.0e-5f;
    float bus_voltage = 12.0f;
    float counts_per_rev = 8192.0f;
};

class DcMotorPlant {
 public:
    explicit DcMotorPlant(PlantParams params = {});

    void reset();
    // Advances the plant by dt_s and folds shaft angle into encoder counts.
    // The step is split into ~50 us substeps so the electrical pole stays stable.
    void step(float duty, float dt_s, SimEncoder& encoder);

    float current_a() const { return current_a_; }
    float omega_rad_s() const { return omega_rad_s_; }
    float rpm() const;
    const PlantParams& params() const { return params_; }

 private:
    PlantParams params_{};
    float current_a_ = 0.f;
    float omega_rad_s_ = 0.f;
    float count_accum_ = 0.f;
};

}  // namespace sim
}  // namespace motor
