#include "ai/ai_wrapper.h"

#include <boost/bind.hpp>

#include "thunderbots_msgs/PlayInfo.h"
#include "util/canvas_messenger/canvas_messenger.h"
#include "util/parameter/dynamic_parameters.h"

AIWrapper::AIWrapper(ros::NodeHandle node_handle)
{
    play_info_publisher = node_handle.advertise<thunderbots_msgs::PlayInfo>(
        Util::Constants::PLAY_INFO_TOPIC, 1);
}

void AIWrapper::onValueReceived(World world)
{
    currently_known_world = world;
    runAIAndSendPrimitives();
    publishPlayInfo();
    drawWorld();
}

void AIWrapper::publishPlayInfo()
{
    auto play_info_msg = convertPlayInfoToROSMessage(ai.getPlayInfo());
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
thunderbots_msgs::PlayInfo AIWrapper::convertPlayInfoToROSMessage(
    const PlayInfo& play_info)
{
    thunderbots_msgs::PlayInfo msg;

    msg.play_name = play_info.play_name;
    msg.play_type = play_info.play_type;

    for (const auto& tactic : play_info.robot_tactic_assignment)
    {
        msg.robot_tactic_assignment.emplace_back(tactic);
    }

    return msg;
}
