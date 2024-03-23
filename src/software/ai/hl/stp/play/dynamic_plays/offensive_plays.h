#pragma once

#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorers/offensive_enemy_third_feasibility_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorers/offensive_friendly_third_feasibility_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorers/offensive_middle_third_feasibility_scorer.h"
#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

/**
 * OffensivePlays are DynamicPlays that are run when we have possession of the ball.
 */
class OffensivePlay : public DynamicPlay
{
   public:
    void evaluate(double score) override;

   protected:
    /**
     * Base constructor for OffensivePlay
     *
     * @param strategy the Strategy
     * @param feasibility_scorer the feasibility scorer for the play
     */
    explicit OffensivePlay(std::shared_ptr<Strategy> strategy,
                           std::unique_ptr<FeasibilityScorer> feasibility_scorer);

    void updateTactics(const PlayUpdate &play_update) override;

   private:
    std::shared_ptr<AttackerTactic> attacker_tactic_;
    std::unique_ptr<DefensePlay> defense_play_;
};

/**
 * OffensiveFriendlyThirdPlay is an OffensivePlay initiated when we gain possession
 * within the friendly third of the field. It prioritizes moving the ball up the field
 * and creating passing opportunities.
 */
class OffensiveFriendlyThirdPlay : public OffensivePlay
{
   public:
    explicit OffensiveFriendlyThirdPlay(std::shared_ptr<Strategy> strategy)
        : OffensivePlay(strategy,
                        std::make_unique<OffensiveFriendlyThirdFeasibilityScorer>())
    {
    }
};

/**
 * OffensiveMiddleThirdPlay is an OffensivePlay initiated when we gain possession
 * within the middle third of the field. It prioritizes gaining entry into the enemy third
 * and setting up scoring opportunities.
 */
class OffensiveMiddleThirdPlay : public OffensivePlay
{
   public:
    explicit OffensiveMiddleThirdPlay(std::shared_ptr<Strategy> strategy)
        : OffensivePlay(strategy,
                        std::make_unique<OffensiveMiddleThirdFeasibilityScorer>())
    {
    }
};

/**
 * OffensiveEnemyThirdPlay is an OffensivePlay initiated when we gain possession
 * within the enemy third of the field. It prioritizes scoring and creating offensive
 * pressure in the enemy zone.
 */
class OffensiveEnemyThirdPlay : public OffensivePlay
{
   public:
    explicit OffensiveEnemyThirdPlay(std::shared_ptr<Strategy> strategy)
        : OffensivePlay(strategy,
                        std::make_unique<OffensiveEnemyThirdFeasibilityScorer>())
    {
    }
};
