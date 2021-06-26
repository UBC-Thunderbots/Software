#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
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

    std::vector<CircleWithColor> getCirclesWithColorToDraw() override;


   private:
    std::shared_ptr<ReceiverTactic> receiver;
    // The maximum time that we will wait before committing to a pass
    const Duration MAX_TIME_TO_COMMIT_TO_PASS;
    std::optional<Pass> best_pass;
    bool committed_to_pass = false;

    // The minimum pass score we will attempt
    static constexpr double MIN_ACCEPTABLE_PASS_SCORE = 0.05;

    /**
     * Finds a place to chip the ball near the net and chips there.
     *
     * @param yield The coroutine to yield to
     * @param crease_defender_tactics The crease defender tactics to use
     * @param world The current state of the world
     */
    static void lastResortChipStage(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        const World &world);

    /**
     * Given a pass, coordinates and executes the pass with a Passer and Receiver
     *
     * @param yield The coroutine to yield
     * @param crease_defender_tactics The crease defender tactics to use
     * @param best_pass_and_score_so_far The Pass to execute
     * @param world The current state of the world
     */
    void performPassStage(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        const PassWithRating &best_pass_and_score_so_far, const World &world);

    /**
     * Tries to find a good pass on the field. This function starts with a high threshold
     * for what it considers a good pass, and slowly reduces this threshold over time if
     * we aren't finding passes that are good enough.
     *
     * @param yield The coroutine to yield to
     * @param crease_defender_tactics The crease defender tactics
     * @param shoot_tactic The AttackerTactic to shoot with
     * @param world The current state of the world
     *
     * @return the pass that was found
     */
    PassWithRating shootOrFindPassStage(
        TacticCoroutine::push_type &yield, std::shared_ptr<AttackerTactic> shoot_tactic,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        const World &world);

    /**
     * Update the tactic that aligns the robot to the ball in preparation to pass
     *
     * @param align_to_ball_tactic
     * @param world The current state of the world
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic,
                                 const World &world);
};
