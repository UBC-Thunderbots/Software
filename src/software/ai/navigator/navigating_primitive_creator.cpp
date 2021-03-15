#include "software/ai/navigator/navigating_primitive_creator.h"

#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/proto/message_translation/tbots_protobuf.h"

NavigatingPrimitiveCreator::NavigatingPrimitiveCreator(
    std::shared_ptr<const NavigatorConfig> config)
    : config(config)
{
}

TbotsProto::Primitive NavigatingPrimitiveCreator::createNavigatingPrimitive(
    const NavigatingIntent &intent, const Path &path,
    const std::vector<ObstaclePtr> &enemy_robot_obstacles)
{
    current_primitive = std::nullopt;
    auto result       = calculateDestinationAndFinalSpeed(intent.getFinalSpeed(), path,
                                                    enemy_robot_obstacles);
    new_destination   = result.first;
    new_final_speed   = result.second;
    intent.accept(*this);
    return *current_primitive;
}

void NavigatingPrimitiveCreator::visit(const MoveIntent &intent)
{
    current_primitive =
        *createMovePrimitive(new_destination, new_final_speed, intent.getFinalAngle(),
                             intent.getDribblerMode(), intent.getAutoChipOrKick(),
                             intent.getMaxAllowedSpeedMode(), intent.getTargetSpinRps());
}

std::pair<Point, double> NavigatingPrimitiveCreator::calculateDestinationAndFinalSpeed(
    double final_speed, Path path,
    const std::vector<ObstaclePtr> &enemy_robot_obstacles) const
{
    double desired_final_speed;
    Point final_dest;
    std::vector<Point> path_points = path.getKnots();

    if (path_points.size() <= 2)
    {
        // we are going to destination
        desired_final_speed = final_speed;
        final_dest          = path.getEndPoint();
    }
    else
    {
        // we are going to some intermediate point so we transition smoothly
        double transition_final_speed = ROBOT_MAX_SPEED_METERS_PER_SECOND *
                                        config->getTransitionSpeedFactor()->value();

        desired_final_speed = calculateTransitionSpeedBetweenSegments(
            path_points[0], path_points[1], path_points[2], transition_final_speed);

        final_dest = path_points[1];
    }

    return std::make_pair<>(
        Point(final_dest),
        // slow down around enemy robots
        desired_final_speed *
            getEnemyObstacleProximityFactor(path_points[1], enemy_robot_obstacles));
}

double NavigatingPrimitiveCreator::getEnemyObstacleProximityFactor(
    const Point &p, const std::vector<ObstaclePtr> &enemy_robot_obstacles) const
{
    double robot_proximity_limit = config->getEnemyRobotProximityLimit()->value();

    // find min dist between p and any robot
    double closest_dist = std::numeric_limits<double>::max();
    for (const auto &obstacle : enemy_robot_obstacles)
    {
        double current_dist = obstacle->distance(p);
        if (current_dist < closest_dist)
        {
            closest_dist = current_dist;
        }
    }

    // clamp ratio between 0 and 1
    return std::clamp(closest_dist / robot_proximity_limit, 0.0, 1.0);
}

double NavigatingPrimitiveCreator::calculateTransitionSpeedBetweenSegments(
    const Point &p1, const Point &p2, const Point &p3, double final_speed)
{
    return final_speed * (p2 - p1).normalize().project((p3 - p2).normalize()).length();
}
