#pragma once

#include "ai/intent/move_intent.h"
#include "ai/navigator/RobotObstacle.h"
#include "ai/primitive/move_primitive.h"
#include "ai/navigator/navigator.h"

class RRTNav : public Navigator
{
   public:
    /**
     * Creates a new Navigator that uses RRT (Rapidly Expanding Random Trees)
     * to generate paths
     */
    RRTNav();

    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
        const World &world,
        const std::vector<std::unique_ptr<Intent>> &assignedIntents) const override;

   private:
	std::optional<Point> findFreePoint(Point initPoint, double angleToDest, std::vector<RobotObstacle> obsts) const;
	double stepSize = 0.1; //10 cm
	double angleStep = 0.26; //15deg in rad
};
