#pragma once

#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play_fsm.h"
#include "software/ai/hl/stp/play/play_base.hpp"

/**
 * A play to set up robots during enemy ball placement
 */
class EnemyBallPlacementPlay : public PlayBase<EnemyBallPlacementPlayFSM>
{
   public:
    /**
     * Constructor for EnemyBallPlacementPlay
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    EnemyBallPlacementPlay(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;
};
