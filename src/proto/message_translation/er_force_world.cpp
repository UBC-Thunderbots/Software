#include <btBulletinDynamicsCommon.h>

#include "software/world/ball_state.h"
#include "software/world/robot_state.h"
#include "software/time/timestamp.h"
#include "proto/message_translation/er_force_world.h"

Ball createBall(world::SimBall sim_ball){
    const btVector3 position = sim_ball.position();
    const btVector3 velocity = sim_ball.speed();
    return Ball(
        {position[0], position[1]}, 
        {velocity[0], velocity[1]}, 
        TimeStamp::fromSeconds(0)
    );
}

Robot createRobot(world::SimRobot sim_robot){

}