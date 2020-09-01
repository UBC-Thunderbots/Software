#pragma once

#include "software/ai/hl/stp/action/all_actions.h"
#include "software/ai/hl/stp/action/mutable_action_visitor.h"
#include "software/world/world.h"

/**
 * This class can be used to generically update the world parameters of any action,
 * without knowing the type of the action
 */
class ActionWorldParamsUpdateVisitor : public MutableActionVisitor
{
   public:
    /**
     * Creates a new ActionWorldParamsUpdateVisitor
     *
     * @param world The world that will be used to update any visited actions
     */
    ActionWorldParamsUpdateVisitor(const World& world);

    /**
     * Visit and update the world parameters for a given action
     */
    void visit(ChipAction& action) override;
    void visit(InterceptBallAction& action) override;
    void visit(KickAction& action) override;
    void visit(MoveAction& action) override;
    void visit(SpinningMoveAction& action) override;
    void visit(StopAction& action) override;

   private:
    const World world;
};
