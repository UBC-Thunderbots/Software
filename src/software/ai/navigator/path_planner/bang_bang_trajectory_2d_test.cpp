#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

#include <gtest/gtest.h>

#include <random>

#include "software/test_util/test_util.h"

class BangBangTrajectory2DTest : public testing::Test
{
   public:
    // Use a fixed seed for the random number generator so that the tests are
    // deterministic
    BangBangTrajectory2DTest()
        : rng(0), pos_uniform_dist(-10, 10), vel_uniform_dist(-2, 2)
    {
    }

   protected:
    static const int NUM_SUB_POINTS   = 30;
    static const int NUM_RANDOM_TESTS = 1000;

    void verifyKinematicConstraints(double max_vel, double max_accel, double max_decel)
    {
        const double sub_point_length_sec =
            traj.getTotalTime().toSeconds() / NUM_SUB_POINTS;
        const double max_accel_decel = std::max(max_accel, std::abs(max_decel));

        for (int i = 0; i <= NUM_SUB_POINTS; i++)
        {
            Duration t = Duration::fromSeconds(i * sub_point_length_sec);

            // Note that the velocity may slightly exceed the max velocity if one of
            // the components gets a small fraction of the overall max velocity, and
            // has an initial velocity which exceeds it. By the time it decelerates
            // to something reasonable, the other component may be accelerating,
            // resulting in the velocity vector exceeding the max.
            Vector vel = traj.getVelocity(t);
            ASSERT_LE(vel.length(), max_vel + 0.5)
                << "Velocity constraint violated at t=" << t << " by "
                << vel.length() - max_vel;

            Vector acc = traj.getAcceleration(t);
            ASSERT_LE(acc.length(), max_accel_decel + 1e-3)
                << "Acceleration constraint violated at t=" << t << " by +"
                << acc.length() - max_accel_decel;
        }
    }

    void verifyPosition(const Point& expected_pos, Duration t)
    {
        Point actual_pos = traj.getPosition(t);
        EXPECT_TRUE(TestUtil::equalWithinTolerance((expected_pos - actual_pos).length(),
                                                   0.0, 0.01))
            << "Trajectory is not at the expected position at t=" << t
            << ". expected_pos=" << expected_pos << " actual_pos=" << actual_pos;
    }

    Vector getRandomVector()
    {
        return Vector(vel_uniform_dist(rng), vel_uniform_dist(rng));
    }

    Point getRandomPoint()
    {
        return Point(pos_uniform_dist(rng), pos_uniform_dist(rng));
    }

    std::mt19937 rng;
    std::uniform_real_distribution<> pos_uniform_dist;
    std::uniform_real_distribution<> vel_uniform_dist;
    BangBangTrajectory2D traj;
};

TEST_F(BangBangTrajectory2DTest, test_already_at_destination)
{
    traj.generate(Point(0, 0), Point(0, 0), Vector(0.0, 0.0), 1, 1, 1);
    verifyPosition(Point(0, 0), Duration::fromSeconds(0));
}

TEST_F(BangBangTrajectory2DTest, test_random_start_and_final_position_sampling)
{
    // Using Monte Carlo method to randomly sample start and final positions
    // and initial velocities and verify that the trajectory satisfies the
    // basic constraints.
    const double max_vel   = 4;
    const double max_accel = 3;
    const double max_decel = 5;
    for (int i = 0; i < NUM_RANDOM_TESTS; ++i)
    {
        std::cout << "Random test " << i << std::endl;

        Point start_pos  = getRandomPoint();
        Point final_pos  = getRandomPoint();
        Vector start_vel = getRandomVector();
        traj.generate(start_pos, final_pos, start_vel, max_vel, max_accel, max_decel);

        verifyKinematicConstraints(max_vel, max_accel, max_decel);
        verifyPosition(start_pos, Duration::fromSeconds(0));
        verifyPosition(final_pos, traj.getTotalTime());
        Vector final_vel = traj.getVelocity(traj.getTotalTime());
        EXPECT_LE(final_vel.length(), 0.001)
            << "Final velocity is " << final_vel << " instead of 0";
    }
}

TEST_F(BangBangTrajectory2DTest, test1)
{
    traj.generate(Point(0, 0), Point(0, 0), Vector(0.0, 0.0), 1, 1, 1);

    std::cout << "Positions " << std::endl;
    const int num_points = 20;
    for (int i = 0; i <= num_points; ++i)
    {
        Point pos = traj.getPosition(
            Duration::fromSeconds(i * traj.getTotalTime().toSeconds() / num_points));
        std::cout << pos << ",";
    }
    std::cout << std::endl;


    std::cout << "XY vel" << std::endl;
    for (int i = 0; i <= num_points; ++i)
    {
        double time = i * traj.getTotalTime().toSeconds() / num_points;
        Vector vel  = traj.getVelocity(Duration::fromSeconds(time));
        std::cout << Point(time, vel.length()) << ",";
    }
    std::cout << std::endl;

    std::cout << "XY accel" << std::endl;
    for (int i = 0; i <= num_points; ++i)
    {
        double time = i * traj.getTotalTime().toSeconds() / num_points;
        Vector acc  = traj.getAcceleration(Duration::fromSeconds(time));
        std::cout << Point(time, acc.length()) << ",";
    }
    std::cout << std::endl;

    std::cout << "Total time: " << traj.getTotalTime().toSeconds() << std::endl;
}
