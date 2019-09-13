#include "software/ai/navigator/placeholder_navigator/placeholder_navigator.h"

#include "software/ai/primitive/catch_primitive.h"
#include "software/ai/primitive/chip_primitive.h"
#include "software/ai/primitive/direct_velocity_primitive.h"
#include "software/ai/primitive/direct_wheels_primitive.h"
#include "software/ai/primitive/dribble_primitive.h"
#include "software/ai/primitive/kick_primitive.h"
#include "software/ai/primitive/move_primitive.h"
#include "software/ai/primitive/movespin_primitive.h"
#include "software/ai/primitive/pivot_primitive.h"
#include "software/ai/primitive/stop_primitive.h"

#include "software/ai/intent/catch_intent.h"
#include "software/ai/intent/chip_intent.h"
#include "software/ai/intent/direct_velocity_intent.h"
#include "software/ai/intent/direct_wheels_intent.h"
#include "software/ai/intent/dribble_intent.h"
#include "software/ai/intent/kick_intent.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/intent/movespin_intent.h"
#include "software/ai/intent/pivot_intent.h"
#include "software/ai/intent/stop_intent.h"

std::vector<std::unique_ptr<Primitive>> PlaceholderNavigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents)
{
    this->world = world;

    auto assigned_primitives = std::vector<std::unique_ptr<Primitive>>();
    for (const auto &intent : assignedIntents)
    {
        intent->accept(*this);
        assigned_primitives.emplace_back(std::move(current_primitive));
    }

    return assigned_primitives;
}

void PlaceholderNavigator::visit(const CatchIntent &catch_intent)
{
    auto p            = std::make_unique<CatchPrimitive>(catch_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const ChipIntent &chip_intent)
{
    auto p            = std::make_unique<ChipPrimitive>(chip_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const DirectVelocityIntent &direct_velocity_intent)
{
    auto p            = std::make_unique<DirectVelocityPrimitive>(direct_velocity_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const DirectWheelsIntent &direct_wheels_intent)
{
    auto p            = std::make_unique<DirectWheelsPrimitive>(direct_wheels_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const DribbleIntent &dribble_intent)
{
    auto p            = std::make_unique<DribblePrimitive>(dribble_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const KickIntent &kick_intent)
{
    auto p            = std::make_unique<KickPrimitive>(kick_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const MoveIntent &move_intent)
{
    auto p            = std::make_unique<MovePrimitive>(move_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const MoveSpinIntent &move_spin_intent)
{
    auto p            = std::make_unique<MoveSpinPrimitive>(move_spin_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const PivotIntent &pivot_intent)
{
    auto p            = std::make_unique<PivotPrimitive>(pivot_intent);
    current_primitive = std::move(p);
}

void PlaceholderNavigator::visit(const StopIntent &stop_intent)
{
    auto p            = std::make_unique<StopPrimitive>(stop_intent);
    current_primitive = std::move(p);
}
