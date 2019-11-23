#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/world/world.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/grab_ball_tactic.h"

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

    void visit(ChipTactic& tactic) override
    {
        tactic.updateWorldParams(world.ball());
    }

    void visit(CreaseDefenderTactic& tactic) override
    {
        tactic.updateWorldParams(world.ball(),world.field(),world.friendlyTeam(),world.enemyTeam());
    }

    void visit(DefenseShadowEnemyTactic& tactic) override
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(GoalieTactic& tactic) override
    {
        tactic.updateWorldParams(world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    }

    void visit(GrabBallTactic& tactic) override
    {
        tactic.updateParams(world.field(), world.ball(), world.enemyTeam());
    }

    void visit(PasserTactic& tactic) override
    {
        tactic.updateWorldParams(world.ball());
    }

    void visit(PenaltyKickTactic& tactic) override
    {
        tactic.updateWorldParams(world.ball(), world.enemyTeam().goalie(),world.field());
    }

    void visit(ReceiverTactic& tactic) override
    {
        tactic.updateWorldParams(world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(ShadowEnemyTactic& tactic) override
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(ShadowFreekickerTactic& tactic) override
    {
        tactic.updateWorldParams(world.enemyTeam(),world.ball());
    }

    void visit(ShootGoalTactic& tactic) override
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    World world;

};

