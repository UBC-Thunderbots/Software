#pragma once

#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play_fsm.h"

/**
 * A play to set up robots during enemy ball placement
 */
class EnemyBallPlacementPlay : public Play
{
   public:
    EnemyBallPlacementPlay(std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;
    std::vector<std::string> getState() override;

   private:
    FSM<EnemyBallPlacementPlayFSM> fsm;
    EnemyBallPlacementPlayFSM::ControlParams control_params;
};
