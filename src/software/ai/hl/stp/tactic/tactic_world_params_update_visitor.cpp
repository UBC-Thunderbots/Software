#include "software/ai/hl/stp/tactic/tactic_world_params_update_visitor.h"

TacticWorldParamsUpdateVisitor::TacticWorldParamsUpdateVisitor(const World &world)
{
    this->world = world;
}

void TacticWorldParamsUpdateVisitor::visit(CherryPickTactic &tactic)
{
    tactic.updateWorldParams(world);
}

void TacticWorldParamsUpdateVisitor::visit(ShadowFreekickerTactic &tactic)
{
    tactic.updateWorldParams(world.enemyTeam(), world.ball());
}

void TacticWorldParamsUpdateVisitor::visit(GoalieTactic &tactic)
{
    tactic.updateWorldParams(world.ball(), world.field(), world.friendlyTeam(),
                             world.enemyTeam());
}

void TacticWorldParamsUpdateVisitor::visit(CreaseDefenderTactic &tactic)
{
    tactic.updateWorldParams(world.ball(), world.field(), world.friendlyTeam(),
                             world.enemyTeam());
}

void TacticWorldParamsUpdateVisitor::visit(ShadowEnemyTactic &tactic)
{
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());
}

void TacticWorldParamsUpdateVisitor::visit(MoveTactic &tactic) {}
void TacticWorldParamsUpdateVisitor::visit(ChipTactic &tactic)
{
    tactic.updateWorldParams(world.ball());
}

void TacticWorldParamsUpdateVisitor::visit(KickoffChipTactic &tactic) {}
void TacticWorldParamsUpdateVisitor::visit(StopTactic &tactic) {}
void TacticWorldParamsUpdateVisitor::visit(PenaltyKickTactic &tactic)
{
    tactic.updateWorldParams(world.ball(), world.enemyTeam().goalie(), world.field());
}

void TacticWorldParamsUpdateVisitor::visit(PenaltySetupTactic &tactic) {}
void TacticWorldParamsUpdateVisitor::visit(ReceiverTactic &tactic)
{
    tactic.updateWorldParams(world.friendlyTeam(), world.enemyTeam(), world.ball());
}

void TacticWorldParamsUpdateVisitor::visit(PatrolTactic &tactic) {}
void TacticWorldParamsUpdateVisitor::visit(ShootGoalTactic &tactic)
{
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());
}

void TacticWorldParamsUpdateVisitor::visit(PasserTactic &tactic)
{
    tactic.updateWorldParams(world.ball());
}

void TacticWorldParamsUpdateVisitor::visit(DefenseShadowEnemyTactic &tactic)
{
    tactic.updateWorldParams(world.field(), world.friendlyTeam(), world.enemyTeam(),
                             world.ball());
}
void TacticWorldParamsUpdateVisitor::visit(MoveTestTactic &tactic) {}
void TacticWorldParamsUpdateVisitor::visit(StopTestTactic &tactic) {}
void TacticWorldParamsUpdateVisitor::visit(GoalieTestTactic &tactic) {}
