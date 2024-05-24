#pragma once

#include <memory>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/passing/receiver_position_generator.hpp"
#include "software/ai/passing/sampling_pass_generator.h"
#include "software/ai/strategy.h"

/**
 * A Play for Direct Free kicks
 */
class FreeKickPlay : public Play
{
   public:
    FreeKickPlay(std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;

   private:
    // The maximum time that we will wait before committing to a pass
    const Duration MAX_TIME_TO_COMMIT_TO_PASS;

    // The minimum pass score we will attempt
    static constexpr double MIN_ACCEPTABLE_PASS_SCORE = 0.05;

    /**
     * Finds a place to chip the ball near the net and chips there.
     *
     * @param yield The coroutine to yield to
     * @param crease_defender_tactics The crease defender tactics to use
     * @param world The current state of the world
     */
    void chipAtGoalStage(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        const WorldPtr &world_ptr);

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
        PassWithRating best_pass_and_score_so_far, const WorldPtr &world_ptr);

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
        const WorldPtr &world_ptr);

    /**
     * Update the tactic that aligns the robot to the ball in preparation to pass
     *
     * @param align_to_ball_tactic
     * @param world The current state of the world
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic,
                                 const WorldPtr &world_ptr);

    SamplingPassGenerator pass_generator;
    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator;
};
