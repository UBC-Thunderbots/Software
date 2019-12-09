#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/passing/pass_generator.h"

/**
 * Play that tries to find a shot on net, passes if it couldn't.
 */
class ShootOrPassPlay : public Play
{
   public:
    static const std::string name;

    ShootOrPassPlay();

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;

   private:
    // How close each of our patrol tactics must be to each of the points in their
    // route before progressing on the next one
    static constexpr double AT_PATROL_POINT_TOLERANCE = 0.3;

    // The speed each patrolling robot should be moving through its control point
    static constexpr double SPEED_AT_PATROL_POINTS = 0.0;

    /**
     * Updates the given cherry-pick tactics
     * @param tactics
     */
    void updateCherryPickTactics(
        std::array<std::shared_ptr<CherryPickTactic>, 2> tactics);

    /**
     * Update the given shoot goal tactic
     *
     * @param shoot_tactic
     */
    void updateShootGoalTactic(std::shared_ptr<ShootGoalTactic> shoot_tactic);

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

    /**
     * Updates the given goalie tactics
     *
     * @param goalie
     */
    void updateGoalie(std::shared_ptr<GoalieTactic> goalie);
};
