#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/passing/pass.h"

/**
 * Play for scoring with static defenders play hardware challenge
 * https://robocup-ssl.github.io/ssl-hardware-challenge-rules/rules.html
 */
class ScoringWithStaticDefendersPlay : public Play
{
   public:
    ScoringWithStaticDefendersPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    // 3 robots for this hardware challenge
    const unsigned int NUM_ROBOTS = 3;

    /**
     * Update the tactic that aligns the robot to the ball in preparation to pass
     *
     * @param align_to_ball_tactic
     * @param world The current state of the world
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic,
                                 Pass pass, const World &world);
};
