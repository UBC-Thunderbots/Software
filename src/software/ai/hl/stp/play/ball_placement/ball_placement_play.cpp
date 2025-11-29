#include "software/ai/hl/stp/play/ball_placement/ball_placement_play.h"

#include "proto/message_translation/tbots_geometry.h"
#include "software/util/generic_factory/generic_factory.h"


BallPlacementPlay::BallPlacementPlay(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayBase<BallPlacementPlayFSM>(ai_config_ptr, true)
{
}

void BallPlacementPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                       const WorldPtr &world_ptr)
{
    // This function doesn't get called so it does nothing, will be removed once
    // coroutines are phased out
}

void BallPlacementPlay::updateTactics(const PlayUpdate &play_update)
{
    auto event = BallPlacementPlayFSM::Update(control_params, play_update);
    fsm.process_event(event);

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();
    if (placement_point.has_value())
    {
        TbotsProto::BallPlacementVisualization ball_placement_vis_msg;
        *(ball_placement_vis_msg.mutable_ball_placement_point()) =
            *createPointProto(placement_point.value());

        LOG(VISUALIZE) << ball_placement_vis_msg;
    }
}

std::vector<std::string> BallPlacementPlay::getState()
{
    std::vector<std::string> state;
    state.emplace_back(objectTypeName(*this) + " - " + getCurrentFullStateName(fsm));
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, BallPlacementPlay,
                       std::shared_ptr<const TbotsProto::AiConfig>>
    factory;
