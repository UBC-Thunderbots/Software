#include "software/world/robot_state.h"
#include "software/world/ball_state.h"
#include "proto/message_translation/er_force_world.h"

Ball createBall(world::SimBall sim_ball, Timestamp timestamp){
    Point position(sim_ball.p_x(), sim_ball.p_y());
    Vector velocity(sim_ball.v_x(), sim_ball.v_y());
    BallState state(position, velocity);
    Ball ball(state, timestamp);
    return ball;
}
Robot createRobot(world::SimRobot sim_robot, Timestamp timestamp){
    RobotId id(sim_robot.id());
    Point position(sim_robot.p_x(), sim_robot.p_y());
    Vector velocity(sim_robot.v_x(), sim_robot.v_y());
    double angular_velocity = atan(sim_robot.r_y()/sim_robot.r_x());
    double angle = 2 * acos(sim_robot.rotation().real());
    RobotState state(position, velocity, Angle::fromRadians(angle), Angle::fromRadians(angular_velocity));
    Robot robot(id, state, timestamp);
    return robot;
}