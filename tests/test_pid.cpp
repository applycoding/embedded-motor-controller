#include "motor/pid.hpp"

#include <gtest/gtest.h>

namespace {

motor::PidLimits wide_limits() { return motor::PidLimits{-100.f, 100.f, -100.f, 100.f}; }

}  // namespace

TEST(Pid, ProportionalTerm) {
    motor::Pid pid({2.f, 0.f, 0.f}, wide_limits());
    EXPECT_NEAR(pid.step(10.f, 0.f, 0.01f), 20.f, 1e-5f);
}

TEST(Pid, OutputSaturation) {
    const motor::PidLimits limits{-5.f, 5.f, -5.f, 5.f};
    motor::Pid pid({10.f, 0.f, 0.f}, limits);
    EXPECT_NEAR(pid.step(10.f, 0.f, 0.01f), 5.f, 1e-5f);
    EXPECT_NEAR(pid.step(-10.f, 0.f, 0.01f), -5.f, 1e-5f);
}

TEST(Pid, IntegralAccumulates) {
    motor::Pid pid({0.f, 4.f, 0.f}, wide_limits());
    EXPECT_NEAR(pid.step(1.f, 0.f, 0.25f), 1.f, 1e-5f);
    EXPECT_NEAR(pid.integral(), 1.f, 1e-5f);
    EXPECT_NEAR(pid.step(1.f, 0.f, 0.25f), 2.f, 1e-5f);
    EXPECT_NEAR(pid.integral(), 2.f, 1e-5f);
}

TEST(Pid, IntegralIsClamped) {
    const motor::PidLimits limits{-100.f, 100.f, -0.5f, 0.5f};
    motor::Pid pid({0.f, 10.f, 0.f}, limits);
    EXPECT_NEAR(pid.step(1.f, 0.f, 1.f), 0.5f, 1e-5f);
    EXPECT_NEAR(pid.step(1.f, 0.f, 1.f), 0.5f, 1e-5f);
    EXPECT_NEAR(pid.integral(), 0.5f, 1e-5f);
}

TEST(Pid, AntiWindupFreezesThenUnwinds) {
    const motor::PidLimits limits{-1.f, 1.f, -10.f, 10.f};
    motor::Pid pid({0.f, 2.f, 0.f}, limits);

    EXPECT_NEAR(pid.step(1.f, 0.f, 1.f), 1.f, 1e-5f);
    EXPECT_NEAR(pid.integral(), 2.f, 1e-5f);
    EXPECT_NEAR(pid.step(1.f, 0.f, 1.f), 1.f, 1e-5f);
    EXPECT_NEAR(pid.step(1.f, 0.f, 1.f), 1.f, 1e-5f);
    EXPECT_NEAR(pid.integral(), 2.f, 1e-5f);

    EXPECT_NEAR(pid.step(0.f, 1.f, 1.f), 0.f, 1e-5f);
    EXPECT_NEAR(pid.integral(), 0.f, 1e-5f);
}

TEST(Pid, DerivativeUsesErrorChange) {
    motor::Pid pid({0.f, 0.f, 0.25f}, wide_limits());
    EXPECT_NEAR(pid.step(0.f, 0.f, 0.5f), 0.f, 1e-5f);
    EXPECT_NEAR(pid.step(4.f, 0.f, 0.5f), 2.f, 1e-5f);
}

TEST(Pid, NonPositiveDtSkipsIntegralAndDerivative) {
    motor::Pid pid({1.f, 5.f, 5.f}, wide_limits());
    EXPECT_NEAR(pid.step(2.f, 0.f, 0.f), 2.f, 1e-5f);
    EXPECT_NEAR(pid.integral(), 0.f, 1e-6f);
}

TEST(Pid, ResetClearsIntegrator) {
    motor::Pid pid({0.f, 4.f, 0.f}, wide_limits());
    EXPECT_NEAR(pid.step(1.f, 0.f, 0.25f), 1.f, 1e-5f);
    pid.reset();
    EXPECT_NEAR(pid.integral(), 0.f, 1e-6f);
    EXPECT_NEAR(pid.last_output(), 0.f, 1e-6f);
    EXPECT_NEAR(pid.step(0.f, 0.f, 0.25f), 0.f, 1e-5f);
}
