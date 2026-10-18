#include "motor/state_machine.hpp"

#include <gtest/gtest.h>

TEST(StateMachine, StartsInIdle) {
    const motor::StateMachine sm;
    EXPECT_EQ(sm.mode(), motor::Mode::Idle);
    EXPECT_EQ(sm.fault(), motor::FaultCode::None);
}

TEST(StateMachine, StartAndStop) {
    motor::StateMachine sm;
    EXPECT_FALSE(sm.stop());
    EXPECT_TRUE(sm.start());
    EXPECT_EQ(sm.mode(), motor::Mode::Run);
    EXPECT_FALSE(sm.start());
    EXPECT_EQ(sm.mode(), motor::Mode::Run);
    EXPECT_TRUE(sm.stop());
    EXPECT_EQ(sm.mode(), motor::Mode::Idle);
    EXPECT_FALSE(sm.clear_fault());
}

TEST(StateMachine, FaultLatchesUntilCleared) {
    motor::StateMachine sm;
    ASSERT_TRUE(sm.start());
    sm.trip(motor::FaultCode::Overcurrent);
    EXPECT_EQ(sm.mode(), motor::Mode::Fault);
    EXPECT_EQ(sm.fault(), motor::FaultCode::Overcurrent);

    sm.trip(motor::FaultCode::Stall);
    EXPECT_EQ(sm.fault(), motor::FaultCode::Overcurrent);

    EXPECT_FALSE(sm.start());
    EXPECT_FALSE(sm.stop());
    EXPECT_EQ(sm.mode(), motor::Mode::Fault);

    EXPECT_TRUE(sm.clear_fault());
    EXPECT_EQ(sm.mode(), motor::Mode::Idle);
    EXPECT_EQ(sm.fault(), motor::FaultCode::None);
    EXPECT_FALSE(sm.clear_fault());
    EXPECT_TRUE(sm.start());
    EXPECT_EQ(sm.mode(), motor::Mode::Run);
}

TEST(StateMachine, TripNoneIsIgnored) {
    motor::StateMachine sm;
    sm.trip(motor::FaultCode::None);
    EXPECT_EQ(sm.mode(), motor::Mode::Idle);
    EXPECT_EQ(sm.fault(), motor::FaultCode::None);
}

TEST(StateMachine, WatchdogFaultFromIdle) {
    motor::StateMachine sm;
    sm.trip(motor::FaultCode::Watchdog);
    EXPECT_EQ(sm.mode(), motor::Mode::Fault);
    EXPECT_EQ(sm.fault(), motor::FaultCode::Watchdog);
}
