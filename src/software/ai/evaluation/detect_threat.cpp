/**
 * This file contains independent Evaluation functions to evaluate threats to the friendly
 * team on the playing field
 */

#include "software/ai/evaluation/detect_threat.h"

#include <optional>

#include "shared/constants.h"
#include "software/new_geom/util/intersection.h"
#include "software/world/field.h"

namespace Evaluation
{
    std::optional<Point> calcBallVelIntersectFriendlyNet(Ball ball, Field field)
    {
        Ray ballRay = Ray(ball.position(), ball.velocity());
        Segment friendlyNetSegment =
            Segment(field.friendlyGoalpostPos(), field.friendlyGoalpostNeg());

        std::vector<Point> intersections = intersection(ballRay, friendlyNetSegment);
        if (intersections.empty())
        {
            return std::nullopt;
        }
        return intersections[0];
    }

    std::optional<Point> calcBallVelIntersectEnemyNet(Ball ball, Field field)
    {
        Ray ballRay = Ray(ball.position(), ball.velocity());
        Segment enemyNetSegment =
            Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

        std::vector<Point> intersections = intersection(ballRay, enemyNetSegment);
        if (intersections.empty())
        {
            return std::nullopt;
        }
        return intersections[0];
    }

};  // namespace Evaluation
