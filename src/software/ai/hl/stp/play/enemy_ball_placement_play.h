#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"

/**
 * A play to set up robots during enemy ball placement
 */
class EnemyBallPlacementPlay : public Play
{
   public:
    EnemyBallPlacementPlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;

   private:
    /**
     * Ball Placement tactic shadowing the defense enemy.
     *
     *
     * @param world world
     * @param yield yield
     * @param crease_defenders list of crease defenders
     * @param move_tactics list of move tactics
     * @param placement_point ball placement point
     */
    void ballPlacementWithShadow(
        TacticCoroutine::push_type &yield, const WorldPtr &world_ptr,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 3> crease_defenders,
        std::array<std::shared_ptr<MoveTactic>, 2> move_tactics, Point placement_point);
};
