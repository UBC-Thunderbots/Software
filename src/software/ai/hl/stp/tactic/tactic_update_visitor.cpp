#include "software/ai/hl/stp/tactic/tactic_update_visitor.h"


class TacticUpdateVisitor : public TacticVisitor
{
public:
    explicit TacticUpdateVisitor(const World& world)
    {
        this->world = world;
    }

    void visit(CherryPickTactic& tactic) override
    {
        tactic.updateWorldParams(world);
    }

<<<<<<< HEAD
    void visit(ShadowFreekickerTactic &tactic) override
    {
        tactic.updateWorldParams(world.enemyTeam(),world.ball());
    }

    void visit(GoalieTactic &tactic) override
    {
        tactic.updateWorldParams(world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    }

    void visit(CreaseDefenderTactic &tactic) override
    {
        tactic.updateWorldParams(world.ball(),world.field(),world.friendlyTeam(),world.enemyTeam());
    }

    void visit(ShadowEnemyTactic &tactic) override
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(BlockShotPathTactic &tactic) override {}
    void visit(MoveTactic &tactic) override {}
    void visit(ChipTactic &tactic) override

    {
        tactic.updateWorldParams(world.ball());
    }

    void visit(KickoffChipTactic &tactic) override {}
    void visit(StopTactic &tactic) override {}
    void visit(PenaltyKickTactic &tactic) override
    {
        tactic.updateWorldParams(world.ball(), world.enemyTeam().goalie(),world.field());
    }


    void visit(PenaltySetupTactic &tactic) override {}
    void visit(ReceiverTactic &tactic) override
    {
        tactic.updateWorldParams(world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(PatrolTactic &tactic) override {}
    void visit(ShootGoalTactic &tactic) override
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(PasserTactic &tactic) override
    {
        tactic.updateWorldParams(world.ball());
    }

    void visit(GrabBallTactic &tactic) override
    {
        tactic.updateParams(world.field(), world.ball(), world.enemyTeam());
    }

    void visit(MoveTestTactic &tactic) override {}
    void visit(StopTestTactic &tactic) override {}
    void visit(DefenseShadowEnemyTactic &tactic) override {}

    World world;

};

