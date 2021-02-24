#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/passing/pass_generator.h"

/**
 * A Play for Direct Free kicks
 */
class FreeKickPlay : public Play
{
   public:
    FreeKickPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    // The maximum time that we will wait before committing to a pass
    const Duration MAX_TIME_TO_COMMIT_TO_PASS;

    // The minimum pass score we will attempt
    static constexpr double MIN_ACCEPTABLE_PASS_SCORE = 0.05;

    std::shared_ptr<const PlayConfig> play_config;

    /**
     * Finds a place to chip the ball near the net and chips there.
     *
     * @param yield The coroutine to yield to
     * @param crease_defender_tactics The crease defender tactics to use
     * @param goalie_tactic The goalie tactic to use
     * @param world The current state of the world
     */
    void chipAtGoalStage(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        std::shared_ptr<GoalieTactic> goalie_tactic, const World &world);

    /**
     * Given a pass, coordinates and executes the pass with a Passer and Receiver
     *
     * @param yield The coroutine to yield
     * @param crease_defender_tactics The crease defender tactics to use
     * @param goalie_tactic The goalie tactic to use
     * @param best_pass_and_score_so_far The Pass to execute
     * @param world The current state of the world
     */
    void performPassStage(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        std::shared_ptr<GoalieTactic> goalie_tactic,
        PassWithRating best_pass_and_score_so_far, const World &world);

    /**
     * Tries to find a good pass on the field. This function starts with a high threshold
     * for what it considers a good pass, and slowly reduces this threshold over time if
     * we aren't finding passes that are good enough.
     *
     * @param yield The coroutine to yield to
     * @param crease_defender_tactics The crease defender tactics
     * @param goalie_tactic The goalie tactic
     * @param world The current state of the world
     *
     * @return the pass that was found
     */
    PassWithRating shootOrFindPassStage(
        TacticCoroutine::push_type &yield, std::shared_ptr<ShootGoalTactic> shoot_tactic,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        std::shared_ptr<GoalieTactic> goalie_tactic, const World &world);

    /**
     * Update the tactic that aligns the robot to the ball in preparation to pass
     *
     * @param align_to_ball_tactic
     * @param world The current state of the world
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic,
                                 const World &world);

    /**
     * Updates the pass generator
     *
     * @param pass_generator
     * @param world The current state of the world
     */
    void updatePassGenerator(PassGenerator &pass_generator, const World &world);
};
