#include "software/ai/hl/stp/tactic/tactic_update_visitor.h"


TacticUpdateVisitor::TacticUpdateVisitor(const World& world)
{
    this->world = world;
}

void TacticUpdateVisitor::visit(CherryPickTactic& tactic)
{
    tactic.updateWorldParams(world);
}

void TacticUpdateVisitor::visit(ShadowFreekickerTactic &tactic)
{
    tactic.updateWorldParams(world.enemyTeam(),world.ball());
}

void TacticUpdateVisitor::visit(GoalieTactic &tactic)
{
    tactic.updateWorldParams(world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
}

void TacticUpdateVisitor::visit(CreaseDefenderTactic &tactic)
{
    tactic.updateWorldParams(world.ball(),world.field(),world.friendlyTeam(),world.enemyTeam());
}

void TacticUpdateVisitor::visit(ShadowEnemyTactic &tactic)
{
    tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
}

void TacticUpdateVisitor::visit(MoveTactic &tactic){}
void TacticUpdateVisitor::visit(ChipTactic &tactic)
{
    tactic.updateWorldParams(world.ball());
}

void TacticUpdateVisitor::visit(KickoffChipTactic &tactic) {}
void TacticUpdateVisitor::visit(StopTactic &tactic){}
void TacticUpdateVisitor::visit(PenaltyKickTactic &tactic)
{
    tactic.updateWorldParams(world.ball(), world.enemyTeam().goalie(),world.field());
}

void TacticUpdateVisitor::visit(PenaltySetupTactic &tactic) {}
void TacticUpdateVisitor::visit(ReceiverTactic &tactic)
{
    tactic.updateWorldParams(world.friendlyTeam(),world.enemyTeam(),world.ball());
}

void TacticUpdateVisitor::visit(PatrolTactic &tactic) {}
void TacticUpdateVisitor::visit(ShootGoalTactic &tactic)
{
    tactic.updateWorldParams(world.field(),world.friendlyTeam(),world.enemyTeam(),world.ball());
}

void TacticUpdateVisitor::visit(PasserTactic &tactic)
{
    tactic.updateWorldParams(world.ball());
}

void TacticUpdateVisitor::visit(MoveTestTactic &tactic) {}
void TacticUpdateVisitor::visit(StopTestTactic &tactic) {}
void TacticUpdateVisitor::visit(DefenseShadowEnemyTactic &tactic) {}



