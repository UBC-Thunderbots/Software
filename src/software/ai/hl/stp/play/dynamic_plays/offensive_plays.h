#pragma once

#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"
#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"

class OffensivePlay : public DynamicPlay
{
   public:
    void evaluate() override;
    
   protected:
    explicit OffensivePlay(std::shared_ptr<Strategy> strategy,
                           std::unique_ptr<FeasibilityScorer> feasibility_scorer)
        : DynamicPlay(strategy, feasibility_scorer),
          attacker_tactic_(std::make_shared<AttackerTactic>(strategy)),
          defense_play_(std::make_unique<DefensePlay>(strategy))
    {
    }

    void updateTactics(const PlayUpdate &play_update) override;

   private:
    std::shared_ptr<AttackerTactic> attacker_tactic_;
    std::unique_ptr<DefensePlay> defense_play_;
};

/**
 * OffensiveFriendlyThirdPlay is a DynamicPlay initiated when we gain possession
 * within the friendly third of the field. It prioritizes moving the ball up the field
 * and creating passing opportunities.
 */
class OffensiveFriendlyThirdPlay : public OffensivePlay
{
    explicit OffensiveFriendlyThirdPlay(std::shared_ptr<Strategy> strategy)
        : OffensivePlay(strategy,
                        std::make_unique<OffensiveFriendlyThirdFeasibilityScorer>())
    {
    }
};

/**
 * OffensiveMiddleThirdPlay is a DynamicPlay initiated when we gain possession
 * within the middle third of the field. It prioritizes gaining entry into the enemy third
 * and setting up scoring opportunities.
 */
class OffensiveMiddleThirdPlay : public OffensivePlay
{
    explicit OffensiveMiddleThirdPlay(std::shared_ptr<Strategy> strategy)
        : OffensivePlay(strategy,
                        std::make_unique<OffensiveMiddleThirdFeasibilityScorer>())
    {
    }
};

/**
 * OffensiveEnemyThirdPlay is a DynamicPlay initiated when we gain possession
 * within the enemy third of the field. It prioritizes scoring and creating offensive
 * pressure in the enemy zone.
 */
class OffensiveEnemyThirdPlay : public OffensivePlay
{
    explicit OffensiveEnemyThirdPlay(std::shared_ptr<Strategy> strategy)
        : OffensivePlay(strategy,
                        std::make_unique<OffensiveEnemyThirdFeasibilityScorer>())
    {
    }
};
