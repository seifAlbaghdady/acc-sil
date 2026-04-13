#include <gtest/gtest.h>
#include "acc.h"

class AccTest : public ::testing::Test
{
protected:
    AdaptiveCruiseControl acc;
    void SetUp() override { acc.reset(); }

    AccOutput run(int cycles, const AccInput &in)
    {
        AccOutput out{};
        for (int i = 0; i < cycles; ++i)
            out = acc.update(in);
        return out;
    }
};

TEST_F(AccTest, InitialStateIsFreeCruise)
{
    EXPECT_EQ(acc.getState(), AccState::FREE_CRUISE);
}

TEST_F(AccTest, FreeCruise_ThrottleIncreasesWhenBelowSetSpeed)
{
    AccInput in{10.0f, 0.0f, 30.0f, false};
    AccOutput out = acc.update(in);
    EXPECT_GT(out.throttle_cmd, 0.0f);
    EXPECT_EQ(out.brake_cmd, 0.0f);
}

TEST_F(AccTest, FreeCruise_NoBrakeWhenAtSetSpeed)
{
    AccInput in{30.0f, 0.0f, 30.0f, false};
    AccOutput out = run(200, in);
    EXPECT_EQ(out.brake_cmd, 0.0f);
}

TEST_F(AccTest, FreeCruise_OutputsClamped)
{
    AccInput in{0.0f, 0.0f, 50.0f, false};
    for (int i = 0; i < 500; ++i)
    {
        AccOutput out = acc.update(in);
        EXPECT_GE(out.throttle_cmd, 0.0f);
        EXPECT_LE(out.throttle_cmd, 1.0f);
        EXPECT_GE(out.brake_cmd, 0.0f);
        EXPECT_LE(out.brake_cmd, 1.0f);
    }
}

TEST_F(AccTest, FreeCruise_StaysFreeCruiseWithNoVehicle)
{
    AccInput in{20.0f, 0.0f, 30.0f, false};
    run(100, in);
    EXPECT_EQ(acc.getState(), AccState::FREE_CRUISE);
}

TEST_F(AccTest, Transition_FreeCruiseToFollowing_WhenVehicleWithinThreshold)
{
    AccInput in{20.0f, 40.0f, 30.0f, true};
    acc.update(in);
    EXPECT_EQ(acc.getState(), AccState::FOLLOWING);
}

TEST_F(AccTest, NoTransition_VehicleBeyondThreshold)
{
    AccInput in{20.0f, 60.0f, 30.0f, true};
    acc.update(in);
    EXPECT_EQ(acc.getState(), AccState::FREE_CRUISE);
}

TEST_F(AccTest, Transition_FollowingToEmergency_WhenTooClose)
{

    acc.update({20.0f, 30.0f, 30.0f, true});
    ASSERT_EQ(acc.getState(), AccState::FOLLOWING);

    acc.update({20.0f, 5.0f, 30.0f, true});
    EXPECT_EQ(acc.getState(), AccState::EMERGENCY);
}

TEST_F(AccTest, Transition_FollowingToFreeCruise_WhenVehicleClears)
{

    acc.update({20.0f, 30.0f, 30.0f, true});
    ASSERT_EQ(acc.getState(), AccState::FOLLOWING);

    acc.update({20.0f, 70.0f, 30.0f, false});
    EXPECT_EQ(acc.getState(), AccState::FREE_CRUISE);
}

TEST_F(AccTest, Hysteresis_StaysFollowingBelowHysteresisBand)
{

    acc.update({20.0f, 30.0f, 30.0f, true});
    ASSERT_EQ(acc.getState(), AccState::FOLLOWING);

    acc.update({20.0f, 55.0f, 30.0f, true});
    EXPECT_EQ(acc.getState(), AccState::FOLLOWING);
}

TEST_F(AccTest, Following_BrakeWhenCloserThanSafeDistance)
{
    acc.update({20.0f, 30.0f, 30.0f, true});
    // Safe distance at 20 m/s = 2*20 + 5 = 45 m; 15 m is too close
    AccOutput out = acc.update({20.0f, 15.0f, 30.0f, true});
    EXPECT_GT(out.brake_cmd, 0.0f);
    EXPECT_EQ(out.throttle_cmd, 0.0f);
}

TEST_F(AccTest, Following_ThrottleWhenFartherThanSafeDistance)
{
    acc.update({20.0f, 45.0f, 30.0f, true});
    AccOutput out = acc.update({20.0f, 48.0f, 30.0f, true});
    EXPECT_GT(out.throttle_cmd, 0.0f);
}

TEST_F(AccTest, Emergency_FullBrakeZeroThrottle)
{
    acc.update({20.0f, 30.0f, 30.0f, true});
    AccOutput out = acc.update({20.0f, 5.0f, 30.0f, true});
    EXPECT_EQ(acc.getState(), AccState::EMERGENCY);
    EXPECT_EQ(out.brake_cmd, 1.0f);
    EXPECT_EQ(out.throttle_cmd, 0.0f);
}

TEST_F(AccTest, Emergency_PersistsUntilSpeedAndGapRecover)
{
    acc.update({20.0f, 30.0f, 30.0f, true});
    acc.update({20.0f, 5.0f, 30.0f, true});
    ASSERT_EQ(acc.getState(), AccState::EMERGENCY);

    acc.update({15.0f, 10.0f, 30.0f, true});
    EXPECT_EQ(acc.getState(), AccState::EMERGENCY);
}

TEST_F(AccTest, Reset_ClearsStateAndIntegrators)
{
    acc.update({20.0f, 30.0f, 30.0f, true});
    acc.reset();
    EXPECT_EQ(acc.getState(), AccState::FREE_CRUISE);
    AccOutput out = acc.update({10.0f, 0.0f, 30.0f, false});
    EXPECT_GT(out.throttle_cmd, 0.0f);
}
