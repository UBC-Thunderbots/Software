#pragma once

#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/patrol_tactic.h"
#include "ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "ai/passing/pass_generator.h"

/**
 * A Play for Corner Kicks
 */
class FreeKickPlay : public Play
{
   public:
    static const std::string name;

    FreeKickPlay();

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

    // The maximum time that we will wait before committing to a pass
    const Duration MAX_TIME_TO_COMMIT_TO_PASS;

    // The minimum open net percentage we will try to shoot at
    const Angle MIN_NET_OPEN_ANGLE_FOR_SHOT;

    // The absolute minimum pass quality we're willing to accept
    static constexpr double ABS_MIN_PASS_QUALITY = 0.05;

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
};
