#include "backend/radio_backend.h"

#include "util/constants.h"
#include "util/ros_messages.h"

RadioBackend::RadioBackend(ros::NodeHandle node_handle)
    : network_input(Util::Constants::SSL_VISION_MULTICAST_ADDRESS,
                    Util::Constants::SSL_VISION_MULTICAST_PORT,
                    boost::bind(&RadioBackend::receiveWorld, this, _1)),
      radio_output(DEFAULT_RADIO_CONFIG,
                   node_handle),
      radio_output_thread(
          boost::bind(&RadioBackend::continuouslySendPrimitivesInBuffer, this)),
      in_destructor(false)
{
}

void RadioBackend::continuouslySendPrimitivesInBuffer()
{
    do
    {
        in_destructor_mutex.unlock();

        auto primitives_ptr = Observer<PrimitiveVecPtr>::getMostRecentValueFromBuffer();
        radio_output.sendPrimitives(*primitives_ptr);

        in_destructor_mutex.lock();
    } while (!in_destructor);
}

void RadioBackend::receiveWorld(Backend::World world)
{
    // TODO: We shouldn't have to do a lot of this work once we have a "real" world
    //       as we can just have a "world.getAllRobots()" function
    // TODO: put this hack in a function for now?
    std::vector<Robot> robots;
    for (auto& team : {world.friendly_team, world.enemy_team}){
        for (auto& robot_msg : team.robots){
            auto robot = Util::ROSMessages::createRobotFromROSMessage(robot_msg);
            robots.emplace_back(robot);
        }
    }
    Ball ball = Util::ROSMessages::createBallFromROSMessage(world.ball);
    radio_output.send_vision_packet(robots, ball);

    sendValueToObservers(world);
}
