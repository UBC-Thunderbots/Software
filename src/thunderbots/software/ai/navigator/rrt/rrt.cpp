#include "rrt.h"
#include "ai/intent/move_intent.h"
#include "ai/primitive/move_primitive.h"

RRTNav::RRTNav()
{
}

std::vector<std::unique_ptr<Primitive>> RRTNav::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents) const
{
    std::vector<std::unique_ptr<Primitive>> assigned_primitives =
        std::vector<std::unique_ptr<Primitive>>();

    // Hand the different types of Intents here
    for (const auto &intent : assignedIntents)
    {
        if (intent->getIntentName() == MOVE_INTENT_NAME)
        {
            // TODO: Implement this
            // Cast down to the MoveIntent class so we can access its members
            MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent);

            std::unique_ptr<Primitive> move_prim =
                std::unique_ptr<Primitive>(new MovePrimitive(
                    move_intent.getRobotId(), move_intent.getDestination(),
                    move_intent.getFinalAngle(), move_intent.getFinalSpeed()));

            assigned_primitives.emplace_back(std::move(move_prim));
        }
        else
        {
            // TODO: Throw a proper exception here
            std::cerr << "Error: Unrecognized Intent given to navigator" << std::endl;
            exit(1);
        }
    }

    return assigned_primitives;
}
