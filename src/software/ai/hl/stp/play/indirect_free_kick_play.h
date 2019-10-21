#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"

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
    void updateCreaseDefenderTactics(std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defenders);
};
