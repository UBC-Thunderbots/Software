#pragma once

#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

/**
 * OffensePlay is a DynamicPlay that is run when we have possession of the ball.
 * It is a Play that sets up tactics to make passes and score goals.
 */
class OffensePlay : public DynamicPlay
{
   public:
    /**
     * Creates an OffensePlay.
     *
     * @param strategy the Strategy shared by all of AI
     */
    explicit OffensePlay(std::shared_ptr<Strategy> strategy);

    void terminate(const WorldPtr& world_ptr) override;

   protected:
    void updateTactics(const PlayUpdate& play_update) override;

    /**
     * Splits the given number of tactics to assign into the number of
     * defender tactics and the number of supporter tactics to assign.
     *
     * @param num_tactics the total number of defense/support tactics to assign
     *
     * @return [num_defenders, num_supporters] tuple
     */
    std::tuple<unsigned int, unsigned int> assignNumOfDefendersAndSupporters(
        unsigned int num_tactics);

   private:
    std::shared_ptr<AttackerTactic> attacker_tactic_;
    std::unique_ptr<DefensePlay> defense_play_;
};
