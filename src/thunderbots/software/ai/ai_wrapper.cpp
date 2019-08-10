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

void AIWrapper::newValueCallback(thunderbots_msgs::World world)
{
    updateKnownWorld(world);
    runAIAndSendPrimitives();
    publishPlayInfo();
    drawWorld();
}

void AIWrapper::updateKnownWorld(thunderbots_msgs::World world_msg)
{
    World new_world = Util::ROSMessages::createWorldFromROSMessage(world_msg);
    currently_known_world.updateBallState(new_world.ball());
    currently_known_world.updateFieldGeometry(new_world.field());
    currently_known_world.updateEnemyTeamState(new_world.enemyTeam());
    currently_known_world.updateFriendlyTeamState(new_world.friendlyTeam());
    currently_known_world.updateTimestamp(new_world.getMostRecentTimestamp());
    RefboxGameState new_game_state =
        Util::ROSMessages::createGameStateFromROSMessage(world_msg.refbox_data.command);
    currently_known_world.updateRefboxGameState(new_game_state);
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
