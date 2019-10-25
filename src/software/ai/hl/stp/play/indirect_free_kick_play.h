#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/ai/passing/pass_generator.h"

/**
 * A Play for Indirect Free kicks
 */
class IndirectFreeKickPlay : public Play
{
   public:
    static const std::string name;

    IndirectFreeKickPlay();

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;

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
     * @param goalie_tactic The goalie tactic to use
     */
    void chipAtGoalStage(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        std::shared_ptr<GoalieTactic> goalie_tactic);

    /**
     * Given a pass, coordinates and executes the pass with a Passer and Receiver
     *
     * @param yield The coroutine to yield
     * @param crease_defender_tactics The crease defender tactics to use
     * @param goalie_tactic The goalie tactic to use
     * @param best_pass_and_score_so_far The Pass to execute
     */
    void performPassStage(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        std::shared_ptr<GoalieTactic> goalie_tactic,
        PassWithRating best_pass_and_score_so_far);

    /**
     * Tries to find a good pass on the field. This function starts with a high threshold
     * for what it considers a good pass, and slowly reduces this threshold over time if
     * we aren't finding passes that are good enough.
     *
     * @param yield The coroutine to yield to
     * @param align_to_ball_tactic The align to ball tactic
     * @param cherry_pick_tactic_1 A cherry pick tactic
     * @param cherry_pick_tactic_2 A cherry pick tactic
     * @param crease_defender_tactics The crease defender tactics
     * @param goalie_tactic The goalie tactic
     * @param pass_generator The pass generator that will generate passes
     * @param best_pass_and_score_so_far The best pass and score so far
     */
    void findPassStage(
        TacticCoroutine::push_type &yield,
        std::shared_ptr<MoveTactic> align_to_ball_tactic,
        std::shared_ptr<CherryPickTactic> cherry_pick_tactic_1,
        std::shared_ptr<CherryPickTactic> cherry_pick_tactic_2,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        std::shared_ptr<GoalieTactic> goalie_tactic, PassGenerator &pass_generator,
        PassWithRating &best_pass_and_score_so_far);

    /**
     * Updates the given cherry-pick tactics
     * @param tactics
     */
    void updateCherryPickTactics(std::vector<std::shared_ptr<CherryPickTactic>> tactics);

    /**
     * Update the tactic that aligns the robot to the ball in preperation to pass
     *
     * @param align_to_ball_tactic
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic);

    /**
     * Updates the pass generator
     *
     * @param pass_generator
     */
    void updatePassGenerator(Passing::PassGenerator &pass_generator);

    /**
     * Updates the given crease defender tactics
     *
     * @param crease_defenders The defenders to update
     */
    void updateCreaseDefenderTactics(
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defenders);
};
