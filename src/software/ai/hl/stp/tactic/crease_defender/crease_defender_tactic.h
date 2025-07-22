#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/segment.h"


/**
 * A crease defender moves around the exterior of our defense box to help shadow
 * shots against the enemy
 *
 *                        XX   <-- Enemy
 *                        XX
 *                          O  <-- Ball
 *
 *
 *
 *                           ++   ++   <--- Two crease defenders
 *                           ++   ++
 *
 *                       +--------------------+
 *                       |                    |
 *                       |         ++         |
 *                       |         ++    <-- Goalie
 *+----------------------+---------++---------+------------------+
 */
class CreaseDefenderTactic : public Tactic<CreaseDefenderFSM>
{
   public:
    /**
     * Creates a new CreaseDefenderTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit CreaseDefenderTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    CreaseDefenderTactic() = delete;

    void accept(TacticVisitor &visitor) const override;

   private:
    std::unique_ptr<FSM<CreaseDefenderFSM>> fsm_init() override;
};
