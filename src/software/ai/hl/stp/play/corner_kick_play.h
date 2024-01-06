#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/passing/pass.h"

/**
 * A Play for Corner Kicks
 */
class CornerKickPlay : public Play
{
   public:
    CornerKickPlay(const TbotsProto::AiConfig &config, std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

    // The maximum distance from the corner that the ball can be for it to be
    // considered a corner kick
    static constexpr double BALL_IN_CORNER_RADIUS = 0.5;

   private:
    /**
     * Update the tactic that aligns the robot to the ball in preparation to pass
     *
     * @param align_to_ball_tactic
     * @param world The current state of the world
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic,
                                 const World &world);

    /**
     * Sets up the pass for the corner kick: aligns the passer and positions the cherry
     * pickers
     *
     * @param yield The coroutine to yield
     * @param world The current state of the world
     *
     * @return the pass that was committed to
     */
    Pass setupPass(TacticCoroutine::push_type &yield, const World &world);
};
