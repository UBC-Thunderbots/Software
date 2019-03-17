/**
 * This file contains independent Evaluation functions to evaluate threats to the friendly
 * team on the playing field
 */

#include "detect_threat.h"

#include <optional>

#include "ai/world/field.h"
#include "geom/util.h"
#include "shared/constants.h"

namespace Evaluation
{
    std::optional<Point> calcBallVelIntersectFriendlyNet(Ball ball, Field field)
    {
        Ray ballRay = Ray(ball.position(), ball.velocity());
        Segment friendlyNetSegment =
            Segment(field.friendlyGoalpostPos(), field.friendlyGoalpostNeg());

        auto [intersection1, intersection2] =
            raySegmentIntersection(ballRay, friendlyNetSegment);

        return intersection1;
    }

    std::optional<Point> calcBallVelIntersectEnemyNet(Ball ball, Field field)
    {
        Ray ballRay = Ray(ball.position(), ball.velocity());
        Segment enemyNetSegment =
            Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

        auto [intersection1, intersection2] =
            raySegmentIntersection(ballRay, enemyNetSegment);
        return intersection1;
    }

};  // namespace Evaluation
