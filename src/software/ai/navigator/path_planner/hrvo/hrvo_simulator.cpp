#include "hrvo_simulator.h"

HRVOSimulator::HRVOSimulator(double time_step, const RobotConstants_t &robot_constants,
                             TeamColour friendly_team_colour)
    : robot_constants(robot_constants),
      robots(),
      world(std::nullopt),
      primitive_set(),
      friendly_team_colour(friendly_team_colour),
      time_step(time_step)
{
}

void HRVOSimulator::updateWorld(const World &world)
{
    LOG(INFO) << time_step;
    this->world = world;
    // TODO (#2498): Update implementation to correctly support adding and removing agents
    //               to represent the newly added and removed friendly/enemy robots in the
    //               World.
    // TODO update agents instead of recreate tracked agents.
    robots.clear();
    for (const Robot &friendly_robot : world.friendlyTeam().getAllRobots())
    {
        configureHRVORobot(friendly_robot);
    }

    for (const Robot &enemy_robot : world.enemyTeam().getAllRobots())
    {
        configureLVRobot(enemy_robot);
    }
}


void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set)
{
    primitive_set = new_primitive_set;

    // Update all friendly agent's primitives
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        auto friendly_robot = robots.find(robot_id);
        if (friendly_robot == robots.end())
        {
            return;
        }
        if (world.has_value())
        {
            friendly_robot->second->updatePrimitive(primitive, world.value(), time_step);
        }
    }
}

void HRVOSimulator::configureHRVORobot(const Robot &robot)
{
    double max_accel = 1e-4;
    double max_speed = 1e-4;
    // TODO move this logic to constructor for agent
    const std::set<RobotCapability> &unavailable_capabilities =
        robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) ==
                    unavailable_capabilities.end();
    if (can_move)
    {
        max_accel = robot_constants.robot_max_acceleration_m_per_s_2;
        max_speed = robot_constants.robot_max_speed_m_per_s;
    }

    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Point destination_point      = robot.position();
    double speed_at_goal         = 0.0;
    const auto &robot_primitives = primitive_set.robot_primitives();
    auto primitive_iter          = robot_primitives.find(robot.id());
    if (primitive_iter != robot_primitives.end())
    {
        TbotsProto::Primitive primitive = primitive_iter->second;
        TbotsProto::Point destination_point_proto;

        if (primitive.has_move())
        {
            const auto &move_primitive = primitive.move();
            // TODO (#2418): Update implementation of Primitive to support
            // multiple path points and remove this check
            CHECK(move_primitive.motion_control().path().points().size() >= 2)
                << "Empty path: "
                << move_primitive.motion_control().path().points().size() << std::endl;
            destination_point_proto =
                move_primitive.motion_control().path().points().at(1);
            destination_point = Point(destination_point_proto.x_meters(),
                                      destination_point_proto.y_meters());
            speed_at_goal     = move_primitive.final_speed_m_per_s();
            max_speed         = move_primitive.max_speed_m_per_s();
        }
    }

    // Max distance which the robot can travel in one time step + scaling
    double max_radius = (max_speed * time_step) / 2;

    RobotPath path = RobotPath({PathPoint(destination_point, speed_at_goal)}, max_radius);

    std::shared_ptr<HRVOAgent> agent =
        std::make_shared<HRVOAgent>(robot.id(), robot.currentState(), TeamSide::FRIENDLY,
                                    path, ROBOT_MAX_RADIUS_METERS, max_speed, max_accel,
                                    FRIENDLY_ROBOT_RADIUS_MAX_INFLATION);
    robots[robot.id()] = std::static_pointer_cast<Agent>(agent);
}

void HRVOSimulator::configureLVRobot(const Robot &robot)
{
    // Set goal of enemy robot to be the farthest point, when moving in the
    // current direction
    Segment segment(robot.position(), robot.position() + robot.velocity() * 100);

    // Enemy robot should not enter the friendly defense area
    std::unordered_set<Point> intersection_point_set =
        intersection(world->field().friendlyDefenseArea(), segment);
    if (intersection_point_set.empty() &&
        contains(world->field().fieldLines(), robot.position()))
    {
        // If the robot is in the field, then move in the current direction
        // towards the field edge
        intersection_point_set = intersection(world->field().fieldLines(), segment);
    }

    if (intersection_point_set.empty())
    {
        // If there is no intersection point (robot is outside the field),
        // continue moving in the current direction
        intersection_point_set.insert(robot.position() + robot.velocity() * 5);
    }

    Point destination = *intersection_point_set.begin();
    double max_speed  = robot_constants.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    double path_radius = (robot.velocity().length() * time_step) / 2;

    RobotPath path = RobotPath({PathPoint(destination, 0.0)}, path_radius);

    // robot_constants.robot_max_speed_m_per_s

    std::shared_ptr<LVAgent> agent = std::make_shared<LVAgent>(
        robot.id(), robot.currentState(), TeamSide::ENEMY, path, ROBOT_MAX_RADIUS_METERS,
        max_speed, 0.0, ENEMY_ROBOT_RADIUS_MAX_INFLATION);

    robots[robot.id() + ENEMY_LV_ROBOT_OFFSET] = std::static_pointer_cast<Agent>(agent);
}

void HRVOSimulator::doStep()
{
    if (time_step == 0.0)
    {
        throw std::runtime_error("Time step is zero");
    }


    if (robots.empty())
    {
        return;
    }

    // Update all the hrvo robots velocities radii based on their current velocity
    for (auto &robot : robots)
    {
        // Linearly increase radius based on the current agent velocity
        robot.second->updateRadiusFromVelocity();
    }

    // Compute what velocity each agent will take next
    for (auto &robot : robots)
    {
        robot.second->computeNewVelocity(robots, time_step);
    }

    // Update the positions of all agents given their velocity
    for (auto &robot : robots)
    {
        robot.second->update(time_step);
    }
}

void HRVOSimulator::visualize(unsigned int robot_id)
{
    TbotsProto::HRVOVisualization hrvo_visualization;
    // Visualize all agents
    for (const auto &robot : robots)
    {
        Point position(robot.second->robot_state.position());
        *(hrvo_visualization.add_robots()) =
            *createCircleProto(Circle(position, robot.second->radius));
    }

    auto friendly_robot = robots.find(robot_id);
    if (friendly_robot == robots.end())
    {
        LOG(WARNING) << "Attempt to visualize untracked robot";
        return;
    }

    std::shared_ptr<HRVOAgent> robot =
        std::static_pointer_cast<HRVOAgent>(friendly_robot->second);
    std::vector<TbotsProto::VelocityObstacle> vo_protos;

    for (const VelocityObstacle &vo : robot->getVelocityObstacles())
    {
        vo_protos.emplace_back(
            *createVelocityObstacleProto(vo, robot->robot_state.position().toVector()));
    }
    hrvo_visualization.set_robot_id(robot_id);

    *(hrvo_visualization.mutable_velocity_obstacles()) = {vo_protos.begin(),
                                                          vo_protos.end()};


    // Visualize the ball obstacle
    if (robot->getBallObstacle().has_value())
    {
        TbotsProto::Circle ball_circle =
            robot->getBallObstacle().value()->createObstacleProto().circle()[0];
        *(hrvo_visualization.add_robots()) = ball_circle;
    }
    if (friendly_team_colour == TeamColour::YELLOW)
    {
        LOG(VISUALIZE, YELLOW_HRVO_PATH) << hrvo_visualization;
    }
    else
    {
        LOG(VISUALIZE, BLUE_HRVO_PATH) << hrvo_visualization;
    }
}

void HRVOSimulator::updateRobotVelocity(RobotId robot_id, const Vector &new_velocity)
{
    auto hrvo_agent = robots.find(robot_id);
    if (hrvo_agent != robots.end())
    {
        hrvo_agent->second->robot_state.velocity() = new_velocity;
    }
}

Vector HRVOSimulator::getRobotVelocity(unsigned int robot_id) const
{
    auto hrvo_agent = robots.find(robot_id);
    if (hrvo_agent != robots.end())
    {
        return hrvo_agent->second->robot_state.velocity();
    }
    LOG(WARNING) << "Velocity for robot " << robot_id
                 << " can not be found since it does not exist in HRVO Simulator"
                 << std::endl;
    return Vector();
}

std::size_t HRVOSimulator::getRobotCount()
{
    return robots.size();
}

std::map<RobotId, std::shared_ptr<Agent>> HRVOSimulator::getRobots()
{
    return robots;
}
