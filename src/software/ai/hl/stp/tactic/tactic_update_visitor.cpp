#pragma once

#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/world/world.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/grab_ball_tactic.h"

class TacticUpdateVisitor : public TacticVisitor
{
public:
    explicit TacticUpdateVisitor(const World &world)
    {
        this->world = world;
    }

    void visit(CherryPickTactic& tactic)
    {
        tactic.updateWorldParams(world);
    }

    void visit(ChipTactic& tactic)
    {
        tactic.updateWorldParams(world.ball());
    }

    void visit(CreaseDefenderTactic& tactic)
    {
        tactic.updateWorldParams((world.ball(),world.field(),world.friendlyTeam(),world.enemyTeam()));
    }

    void visit(DefenseShadowEnemyTactic& tactic)
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(GoalieTactic& tactic)
    {
        tactic.updateWorldParams(world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    }

    void visit(GrabBallTactic& tactic)
    {
        tactic.updateParams(world.field(), world.ball(), world.enemyTeam());
    }

    void visit(PasserTactic& tactic)
    {
        tactic.updateWorldParams(world.ball());
    }

    void visit(PenaltyKickTactic& tactic)
    {
        tactic.updateWorldParams(world.ball(), world.enemyTeam().goalie(),world.field());
    }

    void visit(ReceiverTactic& tactic)
    {
        tactic.updateWorldParams(world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(ShadowEnemyTactic& tactic)
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    void visit(ShadowFreekickerTactic& tactic)
    {
        tactic.updateWorldParams(world.enemyTeam(),world.ball());
    }

    void visit(ShootGoalTactic& tactic)
    {
        tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
    }

    World world;

};

