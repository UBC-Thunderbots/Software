#include "software/ai/hl/stp/action/action_world_params_update_visitor.h"

ActionWorldParamsUpdateVisitor::ActionWorldParamsUpdateVisitor(const World& world)
    : world(world)
{
}

void ActionWorldParamsUpdateVisitor::visit(ChipAction& action)
{
    action.updateWorldParams(world.ball());
}

void ActionWorldParamsUpdateVisitor::visit(DribbleAction& action) {}

void ActionWorldParamsUpdateVisitor::visit(InterceptBallAction& action)
{
    action.updateWorldParams(world.field(), world.ball());
}

void ActionWorldParamsUpdateVisitor::visit(KickAction& action)
{
    action.updateWorldParams(world.ball());
}

void ActionWorldParamsUpdateVisitor::visit(MoveAction& action) {}

void ActionWorldParamsUpdateVisitor::visit(MoveSpinAction& action) {}

void ActionWorldParamsUpdateVisitor::visit(PivotAction& action) {}

void ActionWorldParamsUpdateVisitor::visit(StopAction& action) {}
