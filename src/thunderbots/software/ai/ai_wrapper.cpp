#include "ai/ai_wrapper.h"

#include <boost/bind.hpp>

#include "util/canvas_messenger/canvas_messenger.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/ros_messages.h"

AIWrapper::AIWrapper(ros::NodeHandle node_handle)
{
    play_info_publisher = node_handle.advertise<thunderbots_msgs::PlayInfo>(
        Util::Constants::PLAY_INFO_TOPIC, 1);
}

// TODO: rename to `onValueReceived` here and in ALL other instances (DON'T TRUST CLION)
void AIWrapper::newValueCallback(World world)
{
    currently_known_world = world;
    runAIAndSendPrimitives();
    publishPlayInfo();
    drawWorld();
}

void AIWrapper::publishPlayInfo()
{
    auto play_info_msg =
        Util::ROSMessages::convertPlayPlayInfoToROSMessage(ai.getPlayInfo());
    play_info_publisher.publish(play_info_msg);
}

void AIWrapper::runAIAndSendPrimitives()
{
    if (Util::DynamicParameters::AI::run_ai.value())
    {
        std::vector<std::unique_ptr<Primitive>> new_primitives =
            ai.getPrimitives(currently_known_world);

        auto new_primitives_ptr =
            std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                std::move(new_primitives));
        sendValueToObservers(new_primitives_ptr);
    }
}

void AIWrapper::drawWorld()
{
    std::shared_ptr<Util::CanvasMessenger> canvas_messenger =
        Util::CanvasMessenger::getInstance();
    canvas_messenger->drawWorld(currently_known_world);
}
