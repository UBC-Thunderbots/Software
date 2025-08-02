#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_base.h"

/**
 * A Play that performs ball placement, i.e. placing the ball in a defined location
 * determined by the referee. This is used to obey the referee "Ball Placement Us" command
 */
class BallPlacementPlay : public PlayBase<BallPlacementPlayFSM>
{
   public:
    /**
     * Constructor for BallPlacementPlay
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    BallPlacementPlay(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;
};
