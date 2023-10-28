#include "proto/message_translation/er_force_world.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"


TEST(ErForceWorldTest, test_create_ball)
{
    auto sim_ball = std::make_unique<world::SimBall>();
    sim_ball->set_p_x(1.0);
    sim_ball->set_p_y(2.0);
    sim_ball->set_p_z(3.0);
    sim_ball->set_v_x(4.0);
    sim_ball->set_v_y(5.0);
    sim_ball->set_v_z(6.0);
    const Ball test_ball = createBallProto(*sim_ball, Timestamp::fromSeconds(0));
    const Point pos(sim_ball->p_x(), sim_ball->p_y());
    const Vector vel(sim_ball->v_x(), sim_ball->v_y());
    BallState expected_state(pos, vel);
    EXPECT_TRUE(
        TestUtil::equalWithinTolerance(test_ball.currentState(), expected_state, 1e-6));
}

TEST(ErForceWorldTest, test_create_robot)
{
    auto quater = std::make_unique<world::Quaternion>();
    quater->set_real(1.0);
    quater->set_i(0);
    quater->set_j(0);
    quater->set_k(0);

    auto sim_robot = std::make_unique<world::SimRobot>();
    sim_robot->set_id(0);
    sim_robot->set_p_x(2.0);
    sim_robot->set_p_y(4.0);
    sim_robot->set_p_z(6.0);
    *(sim_robot->mutable_rotation()) = *quater;
    sim_robot->set_v_x(8.0);
    sim_robot->set_v_y(10.0);
    sim_robot->set_v_z(12.0);
    sim_robot->set_r_x(1.0);
    sim_robot->set_r_y(1.0);
    sim_robot->set_r_z(5.0);

    const Robot test_robot = createRobotProto(*sim_robot, Timestamp::fromSeconds(0));
    const Point expected_pos(sim_robot->p_x(), sim_robot->p_y());
    const Vector expected_vel(sim_robot->v_x(), sim_robot->v_y());
    RobotState expected_state(expected_pos, expected_vel, Angle::zero(),
                              Angle::fromRadians(5.0));
    EXPECT_EQ(0, test_robot.id());
    EXPECT_TRUE(TestUtil::equalWithinTolerance(test_robot.currentState(), expected_state,
                                               1e-6, Angle::fromDegrees(0)));
}
