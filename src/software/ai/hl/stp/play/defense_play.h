#pragma once

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"

/**
 * The Defense Play tries to grab the ball from the enemy that has it, and all other
 * robots shadow the enemy robots in order of how threatening they are.
 */
class DefensePlay : public Play
{
   public:
    DefensePlay(std::shared_ptr<const PlayConfig> config) ;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

   private:
    /**
     * Moves up to 2 robots to block the enemy closest from the ball from being
     * able to shoot at the friendly net
     *
     * @param move_tactics The move tactics to use
     * @param world The current state of the world
     *
     * @return Updated move tactics that will block the closest enemy to the ball
     * from being able to shoot at the friendly net
     */
    std::vector<std::shared_ptr<MoveTactic>> moveRobotsToSwarmEnemyWithBall(
        std::vector<std::shared_ptr<MoveTactic>> move_tactics, const World &world);

    std::shared_ptr<const PlayConfig> play_config;
};
