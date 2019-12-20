/**
 * This file contains independent Evaluation functions to evaluate threats to the friendly
 * team on the playing field
 */

#include "software/ai/evaluation/detect_threat.h"

#include <optional>

#include "shared/constants.h"
#include "software/geom/util.h"
#include "software/world/field.h"

namespace Evaluation
{
    std::optional<Point> calcBallVelIntersectFriendlyNet(Ball ball, Field field)
    {
        Ray ballRay = Ray(ball.position(), ball.velocity());
        Segment friendlyNetSegment =
            Segment(field.friendlyGoalpostPos(), field.friendlyGoalpostNeg());

        std::optional<Point> intersection1 =
            raySegmentIntersection(ballRay, friendlyNetSegment).first;

        return intersection1;
    }

    std::optional<Point> calcBallVelIntersectEnemyNet(Ball ball, Field field)
    {
        Ray ballRay = Ray(ball.position(), ball.velocity());
        Segment enemyNetSegment =
            Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

        std::optional<Point> intersection1 =
            raySegmentIntersection(ballRay, enemyNetSegment).first;

        return intersection1;
    }

};  // namespace Evaluation
