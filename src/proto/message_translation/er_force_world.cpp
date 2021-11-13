#include <btBulletinDynamicsCommon.h>

#include "software/time/timestamp.h"
#include "software/world/robot_state.h"
#include "software/world/ball_state.h"
#include "proto/message_translation/er_force_world.h"
#include "extlibs/er_force_sim/src/protobuf/robot.pb.h"

Ball createBall(world::SimBall sim_ball, TimeStamp timestamp){
    Point position(sim_ball.p_x(), sim_ball.p_y());
    Vector velocity(sim_ball.v_x(), sim_ball.v_y());
    BallState state(position, velocity);
    return Ball(state, timestamp);
}

Robot createRobot(world::SimRobot sim_robot, TimeStamp timestamp){
    RobotId id(sim_robot.id());
    Point position(sim_robot.p_x(), sim_robot.p_y());
    Vector velocity(sim_robot.v_x(), sim_robot.v_y());
    AngularVelocity omega(sim_robot.r_x(), sim_robot.r_y());
    Angle angle(2 * acos(sim_robot.rotation().real()));
    RobotState state(position, velocity, angle, omega);
    return Robot(id, state, timestamp);
}