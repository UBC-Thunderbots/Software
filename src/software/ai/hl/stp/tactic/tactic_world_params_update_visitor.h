#pragma once

#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/mutable_tactic_visitor.h"

class TacticWorldParamsUpdateVisitor : public MutableTacticVisitor
{
   public:
    /**
     * Creates a new TacticUpdateVisitor
     *
     * @param world The World we would like to update the Tactics with
     */
    explicit TacticWorldParamsUpdateVisitor(const World &world);

    TacticWorldParamsUpdateVisitor() = delete;

    /**
     * Calls the updateWorldParam in tactic to update params
     *
     * @param tactic The Tactic to update
     */
    void visit(CherryPickTactic &tactic) override;
    void visit(ShadowFreekickerTactic &tactic) override;
    void visit(GoalieTactic &tactic) override;
    void visit(CreaseDefenderTactic &tactic) override;
    void visit(ShadowEnemyTactic &tactic) override;
    void visit(MoveTactic &tactic) override;
    void visit(ChipTactic &tactic) override;
    void visit(KickoffChipTactic &tactic) override;
    void visit(StopTactic &tactic) override;
    void visit(PenaltyKickTactic &tactic) override;
    void visit(PenaltySetupTactic &tactic) override;
    void visit(ReceiverTactic &tactic) override;
    void visit(PatrolTactic &tactic) override;
    void visit(ShootGoalTactic &tactic) override;
    void visit(PasserTactic &tactic) override;
    void visit(DefenseShadowEnemyTactic &tactic) override;
    void visit(MoveTestTactic &tactic) override;
    void visit(StopTestTactic &tactic) override;
    void visit(GoalieTestTactic &tactic) override;

   private:
    World world;
};
