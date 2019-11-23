#pragma once

#include "software/ai/hl/stp/tactic/tactic_visitor.h"
<<<<<<< HEAD
#include "software/ai/hl/stp/tactic/all_tactics.h"
=======
#include "software/world/world.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/grab_ball_tactic.h"
>>>>>>> 464ae18961cae3d2d6b65d74df2fb4d5a2878f5f

class TacticUpdateVisitor : public TacticVisitor
{
public:
    /**
     * Creates a new TacticUpdateVisitor
     *
     * @param world The World we would like to update the Tactics with
     */
    explicit TacticUpdateVisitor(const World &world);

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
     void visit(BlockShotPathTactic &tactic) override;
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
     void visit(GrabBallTactic &tactic) override;
     void visit(MoveTestTactic &tactic) override;
     void visit(StopTestTactic &tactic) override;
     void visit(DefenseShadowEnemyTactic &tactic) override;

    World world;
};
