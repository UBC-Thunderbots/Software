#include "ai/ai_wrapper.h"

#include <boost/bind.hpp>

#include "thunderbots_msgs/PlayInfo.h"
#include "util/canvas_messenger/canvas_messenger.h"
#include "util/parameter/dynamic_parameters.h"
#include "gui/drawing/navigator.h"

AIWrapper::AIWrapper(ros::NodeHandle node_handle)
{
    play_info_publisher = node_handle.advertise<thunderbots_msgs::PlayInfo>(
        Util::Constants::PLAY_INFO_TOPIC, PLAY_INFO_QUEUE_SIZE);
}

void AIWrapper::onValueReceived(World world)
{
    most_recent_world = world;
    runAIAndSendPrimitives();
    drawAI();
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
            ai.getPrimitives(most_recent_world);

        auto new_primitives_ptr =
            std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                std::move(new_primitives));
        Subject<ConstPrimitiveVectorPtr>::sendValueToObservers(new_primitives_ptr);
    }
}

void AIWrapper::drawAI() {
    auto paths = ai.getNavigator()->getPlannedPaths();
    auto scene = std::make_shared<QGraphicsScene>();
    drawRobotPaths(&(*scene), paths);
    std::cout << "sending paths" << std::endl;
    Subject<std::shared_ptr<QGraphicsScene>>::sendValueToObservers(scene);
}

void AIWrapper::drawWorld()
{
    std::shared_ptr<Util::CanvasMessenger> canvas_messenger =
        Util::CanvasMessenger::getInstance();
    canvas_messenger->drawWorld(most_recent_world);
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
