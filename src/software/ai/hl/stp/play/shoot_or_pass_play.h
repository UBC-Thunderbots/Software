#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.hpp"

/**
 * Play that tries to find a shot on net, passes if it couldn't.
 */
class ShootOrPassPlay : public Play
{
   public:
    ShootOrPassPlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    // How close each of our patrol tactics must be to each of the points in their
    // route before progressing on the next one
    static constexpr double AT_PATROL_POINT_TOLERANCE = 0.3;

    // The speed each patrolling robot should be moving through its control point
    static constexpr double SPEED_AT_PATROL_POINTS = 0.0;

    /**
     * Sets up the pass for the corner kick: aligns the passer and positions the cherry
     * pickers
     *
     * @param yield The coroutine to yield
     * @param crease_defender_tactics The crease defender tactics
     * @param attacker_tactic The attacker tactic
     * @param world The current state of the world
     *
     * @return the best pass found
     */
    PassWithRating attemptToShootWhileLookingForAPass(
        TacticCoroutine::push_type &yield,
        std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics,
        std::shared_ptr<AttackerTactic> attacker_tactic, const World &world);
};
