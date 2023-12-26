#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class BangBangTrajectory1DTest : public testing::Test
{
   protected:
    void verifyVelocityAndAccelerationLimits(double max_vel, double max_accel,
                                             double max_decel)
    {
        for (unsigned int i = 0; i < traj.getNumTrajectoryParts(); i++)
        {
            const auto& part = traj.getTrajectoryPart(i);
            EXPECT_LE(std::abs(part.velocity), max_vel)
                << "Trajectory part " << i << " has a velocity of " << part.velocity
                << " which has a greater magnitude than the limit of " << max_vel;

            double accel_limit;
            // if velocity and acceleration have opposite signs, then the velocity is
            // going towards 0, so use max_decel.
            if (part.velocity * part.acceleration < 0)
            {
                // Velocity is going towards 0, so use max_decel
                accel_limit = max_decel;
            }
            else
            {
                accel_limit = max_accel;
            }
            EXPECT_LE(std::abs(part.acceleration), std::abs(accel_limit))
                << "Trajectory part " << i << " has an acceleration of "
                << part.acceleration
                << " which has a greater magnitude than the limit of " << accel_limit;
        }
    }

    void verifyChronologicalTime()
    {
        double prev_time = 0.0;
        for (unsigned int i = 0; i < traj.getNumTrajectoryParts(); i++)
        {
            const auto& part = traj.getTrajectoryPart(i);
            EXPECT_GE(part.end_time_sec, prev_time)
                << "Trajectory part " << i << " ends at " << part.end_time_sec
                << " which is greater than the end time of previous part: " << prev_time;
            prev_time = part.end_time_sec;
        }
    }

    void verifyFinalState(double final_pos, double final_accel)
    {
        EXPECT_TRUE(TestUtil::equalWithinTolerance(
            final_pos, traj.getPosition(traj.getTotalTime()), 0.001));
        // The final velocity should always be 0
        EXPECT_TRUE(TestUtil::equalWithinTolerance(
            0, traj.getVelocity(traj.getTotalTime()), 0.001));
        EXPECT_TRUE(TestUtil::equalWithinTolerance(
            final_accel, traj.getAcceleration(traj.getTotalTime()), 0.001));
    }

    void verifyPartState(size_t part_index, double end_time, double position,
                         double velocity, double acceleration)
    {
        const auto& part = traj.getTrajectoryPart(part_index);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(end_time, part.end_time_sec, 0.001))
            << "Trajectory part " << part_index << " ends at " << part.end_time_sec
            << " instead of the expected end time of " << end_time;
        EXPECT_TRUE(TestUtil::equalWithinTolerance(position, part.position, 0.001))
            << "Trajectory part " << part_index << " has a position of " << part.position
            << " instead of the expected position of " << position;
        EXPECT_TRUE(TestUtil::equalWithinTolerance(velocity, part.velocity, 0.001))
            << "Trajectory part " << part_index << " has a velocity of " << part.velocity
            << " instead of the expected velocity of " << velocity;
        EXPECT_TRUE(
            TestUtil::equalWithinTolerance(acceleration, part.acceleration, 0.001))
            << "Trajectory part " << part_index << " has an acceleration of "
            << part.acceleration << " instead of the expected acceleration of "
            << acceleration;
    }

    BangBangTrajectory1D traj;
};

TEST_F(BangBangTrajectory1DTest, positive_symmetrical_trapezoidal_profile)
{
    double initial_pos = 0;
    double destination = 3;
    double initial_vel = 0;
    double max_vel     = 1;
    double max_accel   = 1;
    double max_decel   = -1;

    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 3);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // 1.0 sec to accelerate from 0 to 1 m/s (travels 0.5m)
    // 1.0 sec to decelerate from 1 to 0 m/s (travels 0.5m)
    // Remaining 2m travelled at 1 m/s takes 2 sec
    // Total time = 4 sec
    EXPECT_TRUE(TestUtil::equalWithinTolerance(4.0, traj.getTotalTime(), 0.001));

    verifyPartState(0, 1.0, initial_pos, initial_vel, max_accel);
    verifyPartState(1, 3.0, 0.5, max_vel, 0.0);
    verifyPartState(2, 4.0, 2.5, max_vel, max_decel);
    verifyFinalState(destination, max_decel);
}

TEST_F(BangBangTrajectory1DTest, positive_non_symmetrical_trapezoidal_profile)
{
    double initial_pos = 2;
    double destination = 8.5;
    double initial_vel = 1;
    double max_vel     = 2;
    double max_accel   = 1;
    double max_decel   = -2;

    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 3);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // 1.0 sec to accelerate from 1 to 2 m/s (travels 1.5m)
    // 1.0 sec to decelerate from 2 to 0 m/s (travels 1m)
    // Remaining 4m travelled at 2 m/s takes 2 sec
    // Total time = 4 sec
    EXPECT_TRUE(TestUtil::equalWithinTolerance(4.0, traj.getTotalTime(), 0.001));

    verifyPartState(0, 1.0, initial_pos, initial_vel, max_accel);
    verifyPartState(1, 3.0, 3.5, max_vel, 0.0);
    verifyPartState(2, 4.0, 7.5, max_vel, max_decel);
    verifyFinalState(destination, max_decel);
}

TEST_F(BangBangTrajectory1DTest,
       positive_non_symmetrical_trapezoidal_profile_with_negative_initial_velocity)
{
    double initial_pos = 5;
    double destination = 27;
    // Initially moving away from the destination. So we must stop, and then accelerate
    // towards the destination.
    double initial_vel = -2;
    double max_vel     = 4;
    double max_accel   = 2;
    double max_decel   = -1;

    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 4);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // 2.0 sec to decelerate from -2 to 0 m/s (travels -2m)
    // 2.0 sec to accelerate from 0  to 4 m/s (travels 4m)
    // 4.0 sec to decelerate from 4  to 0 m/s (travels 8m)
    // Remaining 12m travelled at 4 m/s takes 3 sec
    // Total time = 11 sec
    EXPECT_TRUE(TestUtil::equalWithinTolerance(11.0, traj.getTotalTime(), 0.001));

    // We're decelerating in the positive direction (i.e. velocity is increasing,
    // from -2 to 0).
    verifyPartState(0, 2.0, initial_pos, initial_vel, -max_decel);
    verifyPartState(1, 4.0, 3.0, 0, max_accel);
    verifyPartState(2, 7.0, 7.0, max_vel, 0.0);
    verifyPartState(3, 11.0, 19.0, max_vel, max_decel);
    verifyFinalState(destination, max_decel);
}

TEST_F(BangBangTrajectory1DTest, negative_non_symmetrical_trapezoidal_profile)
{
    double initial_pos = -2;
    double destination = -8.5;
    double initial_vel = -1;
    double max_vel     = 2;
    double max_accel   = 1;
    double max_decel   = -2;

    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 3);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // 1.0 sec to accelerate from 1 to 2 m/s (travels 1.5m)
    // 1.0 sec to decelerate from 2 to 0 m/s (travels 1m)
    // Remaining 4m travelled at 2 m/s takes 2 sec
    // Total time = 4 sec
    EXPECT_TRUE(TestUtil::equalWithinTolerance(4.0, traj.getTotalTime(), 0.001));

    verifyPartState(0, 1.0, initial_pos, initial_vel, -max_accel);
    verifyPartState(1, 3.0, -3.5, -max_vel, 0.0);
    verifyPartState(2, 4.0, -7.5, -max_vel, -max_decel);
    verifyFinalState(destination, -max_decel);
}

TEST_F(BangBangTrajectory1DTest, positive_non_symmetrical_triangular_profile)
{
    double initial_pos = 2;
    double destination = 10.5;
    double initial_vel = 1;
    // Pick a high max velocity to ensure that the profile is triangular
    double max_vel   = 10;
    double max_accel = 1;
    double max_decel = -2;

    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 2);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // Expected values are determined through Desmos
    // https://www.desmos.com/calculator/lgpc5g8ewr
    EXPECT_TRUE(TestUtil::equalWithinTolerance(4.19615, traj.getTotalTime(), 0.001));

    verifyPartState(0, 2.46410, initial_pos, initial_vel, max_accel);
    verifyPartState(1, 4.19615, 7.5, 3.46410, max_decel);
    verifyFinalState(destination, max_decel);
}

TEST_F(BangBangTrajectory1DTest, negative_non_symmetrical_triangular_profile)
{
    double initial_pos = -2;
    double destination = -10.5;
    double initial_vel = -1;
    // Pick a high max velocity to ensure that the profile is triangular
    double max_vel   = 10;
    double max_accel = 1;
    double max_decel = -2;

    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 2);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // Expected values are determined through Desmos
    // https://www.desmos.com/calculator/lgpc5g8ewr
    EXPECT_TRUE(TestUtil::equalWithinTolerance(4.19615, traj.getTotalTime(), 0.001));

    verifyPartState(0, 2.46410, initial_pos, initial_vel, -max_accel);
    verifyPartState(1, 4.19615, -7.5, -3.46410, -max_decel);
    verifyFinalState(destination, -max_decel);
}

TEST_F(BangBangTrajectory1DTest,
       positive_non_symmetrical_triangular_profile_with_negative_initial_velocity)
{
    double initial_pos = 5;
    double destination = 15;
    // Initially moving away from the destination. So we must stop, and then accelerate
    // towards the destination.
    double initial_vel = -2;
    // Pick a high max velocity to ensure that the profile is triangular
    double max_vel   = 20;
    double max_accel = 2;
    double max_decel = -1;

    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 3);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // 2.0 sec to decelerate from -2 to 0 m/s (travels -2m)
    // 2.0 sec to accelerate from 0  to 4 m/s (travels 4m)
    // 4.0 sec to decelerate from 4  to 0 m/s (travels 8m)
    // Total time = 8 sec
    EXPECT_TRUE(TestUtil::equalWithinTolerance(8.0, traj.getTotalTime(), 0.001));

    // We're decelerating in the positive direction (i.e. velocity is increasing,
    // from -2 to 0).
    verifyPartState(0, 2.0, initial_pos, initial_vel, -max_decel);
    verifyPartState(1, 4.0, 3.0, 0, max_accel);
    verifyPartState(2, 8.0, 7.0, 4.0, max_decel);
    verifyFinalState(destination, max_decel);
}

TEST_F(BangBangTrajectory1DTest,
       positive_non_symmetrical_triangular_profile_overshooting_destination)
{
    // We're near the destination, but our initial velocity is too high, resulting
    // in us overshooting the destination.
    double initial_pos = 0;
    double destination = 5;
    double initial_vel = 4;
    // Pick a high max velocity to ensure that the profile is triangular
    double max_vel   = 20;
    double max_accel = -2;
    double max_decel = 1;
    traj.generate(initial_pos, destination, initial_vel, max_vel, max_accel, max_decel);
    EXPECT_EQ(traj.getNumTrajectoryParts(), 3);
    verifyVelocityAndAccelerationLimits(max_vel, max_accel, max_decel);
    verifyChronologicalTime();

    // 4.0 sec to decelerate from 4  to  0 m/s (travels 8m -> overshooting destination by
    // 3m) 1.0 sec to accelerate from 0  to -2 m/s (travels -1m) 2.0 sec to decelerate
    // from -2 to 0 m/s (travels -2m) Total time = 7 sec
    EXPECT_TRUE(TestUtil::equalWithinTolerance(7.0, traj.getTotalTime(), 0.001));

    verifyPartState(0, 4.0, initial_pos, initial_vel, -max_decel);
    verifyPartState(1, 5.0, 8.0, 0, max_accel);
    verifyPartState(2, 7.0, 7.0, -2.0, max_decel);
    verifyFinalState(destination, max_decel);
}

TEST_F(BangBangTrajectory1DTest, already_at_destination)
{
    double initial_pos = 0;
    double destination = 0;
    double initial_vel = 0;

    traj.generate(initial_pos, destination, initial_vel, 1, 1, 1);
    EXPECT_TRUE(TestUtil::equalWithinTolerance(0.0, traj.getTotalTime(), 0.001));
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(initial_pos, traj.getDestination(), 0.001));
}

TEST_F(BangBangTrajectory1DTest, test_trajectory_min_max_range)
{
    // Trajectory from 0 to 1
    traj.generate(0.0, 1.0, 0.0, 1.0, 1.0, 1.0);
    auto min_max = traj.getMinMaxPositions();
    EXPECT_DOUBLE_EQ(min_max.first, 0.0);
    EXPECT_DOUBLE_EQ(min_max.second, 1.0);
}
