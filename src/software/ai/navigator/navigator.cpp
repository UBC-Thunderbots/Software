#include "software/ai/navigator/navigator.h"

#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/proto/message_translation/tbots_protobuf.h"

Navigator::Navigator(std::unique_ptr<PathManager> path_manager,
                     RobotNavigationObstacleFactory robot_navigation_obstacle_factory,
                     std::shared_ptr<const NavigatorConfig> config)
    : config(config),
      robot_navigation_obstacle_factory(std::move(robot_navigation_obstacle_factory)),
      path_manager(std::move(path_manager))
{
}

void Navigator::visit(const DirectPrimitiveIntent &intent)
{
    current_primitive = intent.getDirectPrimitive();
}

void Navigator::visit(const MoveIntent &intent)
{
    unsigned int robot_id      = intent.getRobotId();
    auto robot_id_to_path_iter = robot_id_to_path.find(robot_id);
    if (robot_id_to_path_iter != robot_id_to_path.end() && robot_id_to_path_iter->second)
    {
        auto [new_destination, new_final_speed] = calculateDestinationAndFinalSpeed(
            intent.getFinalSpeed(), *(robot_id_to_path_iter->second));

        PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
        params->set_parameter1(
            static_cast<float>(new_destination.x() * MILLIMETERS_PER_METER));
        params->set_parameter2(
            static_cast<float>(new_destination.y() * MILLIMETERS_PER_METER));
        params->set_parameter3(static_cast<float>(intent.getFinalAngle().toRadians() *
                                                  CENTIRADIANS_PER_RADIAN));
        params->set_parameter4(
            static_cast<float>(new_final_speed * MILLIMETERS_PER_METER));
        uint32_t extra_bits = 0;
        extra_bits |= (intent.getAutochickType() == AutochickType::AUTOKICK) * 0x01;
        extra_bits |= (intent.getDribblerEnable() == DribblerEnable::ON) * 0x02;
        extra_bits |= (intent.getAutochickType() == AutochickType::AUTOCHIP) * 0x04;
        params->set_extra_bits(extra_bits);
        params->set_slow(intent.getMoveType() == MoveType::SLOW);

        current_primitive = PrimitiveMsg();
        current_primitive->set_allocated_move(params);
    }
    else
    {
        LOG(WARNING) << "Navigator's path manager could not find a path for RobotId = "
                     << robot_id;
        current_primitive = ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
            StopPrimitive(robot_id, false));
    }
}

std::unique_ptr<PrimitiveSetMsg> Navigator::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &intents)
{
    planned_paths.clear();
    robot_id_to_path.clear();
    Rectangle navigable_area = world.field().fieldBoundary();
    auto path_objectives     = createPathObjectives(intents, world);
    robot_id_to_path = path_manager->getManagedPaths(path_objectives, navigable_area);

    auto primitive_set_msg                    = std::make_unique<PrimitiveSetMsg>();
    *(primitive_set_msg->mutable_time_sent()) = *createCurrentTimestampMsg();
    auto &robot_primitives_map = *primitive_set_msg->mutable_robot_primitives();

    for (const auto &intent : intents)
    {
        current_primitive = std::nullopt;
        intent->accept(*this);
        robot_primitives_map[intent->getRobotId()] = *current_primitive;
    }
    return primitive_set_msg;
}

std::vector<PathObjective> Navigator::createPathObjectives(
    const std::vector<std::unique_ptr<Intent>> &intents, const World &world)
{
    std::vector<ObstaclePtr> friendly_non_navigating_robot_obstacles;
    std::vector<PathObjective> path_objectives;
    const auto ball_obstacle =
        robot_navigation_obstacle_factory.createFromBallPosition(world.ball().position());

    for (const auto &intent : intents)
    {
        unsigned int robot_id = intent->getRobotId();
        auto robot            = world.friendlyTeam().getRobotById(robot_id);
        if (robot)
        {
            std::vector<ObstaclePtr> obstacles =
                robot_navigation_obstacle_factory.createFromMotionConstraints(
                    intent->getMotionConstraints(), world);

            if (intent->getBallCollisionType() == BallCollisionType::AVOID)
            {
                obstacles.push_back(ball_obstacle);
            }

            Point path_obj_end;
            if (intent->getNavigationDestination())
            {
                path_obj_end = *intent->getNavigationDestination();
            }
            else
            {
                path_obj_end = robot->position();
            }
            path_objectives.emplace_back(
                PathObjective{.robot_id      = robot_id,
                              .start         = robot->position(),
                              .end           = path_obj_end,
                              .current_speed = robot->velocity().length(),
                              .obstacles     = obstacles});
        }
        else
        {
            LOG(WARNING) << "Failed to find robot associated with robot id = "
                         << robot_id;
            continue;
        }
    }
    return path_objectives;
}

std::pair<Point, double> Navigator::calculateDestinationAndFinalSpeed(double final_speed,
                                                                      Path path)
{
    double desired_final_speed;
    Point final_dest;
    std::vector<Point> path_points = path.getKnots();
    planned_paths.emplace_back(path_points);

    if (path_points.size() <= 2)
    {
        // we are going to destination
        desired_final_speed = final_speed;
        final_dest          = path.getEndPoint();
    }
    else
    {
        // we are going to some intermediate point so we transition smoothly
        double transition_final_speed =
            ROBOT_MAX_SPEED_METERS_PER_SECOND * config->TransitionSpeedFactor()->value();

        desired_final_speed = calculateTransitionSpeedBetweenSegments(
            path_points[0], path_points[1], path_points[2], transition_final_speed);

        final_dest = path_points[1];
    }

    return std::make_pair<>(Point(final_dest), desired_final_speed);
}

double Navigator::calculateTransitionSpeedBetweenSegments(const Point &p1,
                                                          const Point &p2,
                                                          const Point &p3,
                                                          double final_speed)
{
    return final_speed * (p2 - p1).normalize().project((p3 - p2).normalize()).length();
}

std::vector<std::vector<Point>> Navigator::getPlannedPathPoints()
{
    return planned_paths;
}

std::vector<ObstaclePtr> Navigator::getObstacles()
{
    return path_manager->getObstacles();
}
