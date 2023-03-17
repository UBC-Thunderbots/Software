#include "hrvo_simulator.h"

HRVOSimulator::HRVOSimulator() : robots(), world(std::nullopt), primitive_set() {}

void HRVOSimulator::updateWorld(const World &world,
                                const RobotConstants_t &robot_constants,
                                Duration time_step)
{
    this->world = world;
    // TODO (#2498): Update implementation to correctly support adding and removing agents
    //               to represent the newly added and removed friendly/enemy robots in the
    //               World.
    robots.clear();
    for (const Robot &friendly_robot : world.friendlyTeam().getAllRobots())
    {
        configureHRVORobot(friendly_robot, robot_constants, time_step);
    }

    for (const Robot &enemy_robot : world.enemyTeam().getAllRobots())
    {
        configureLVRobot(enemy_robot, robot_constants, time_step);
    }
}


void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set,
                                       Duration time_step)
{
    // Update all friendly agent's primitives
    if (world.has_value())
    {
        primitive_set = new_primitive_set;
        for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
        {
            auto friendly_robot = robots.find(robot_id);
            if (friendly_robot == robots.end())
            {
                continue;
            }

            friendly_robot->second->updatePrimitive(primitive, world.value(), time_step);
        }
    }
}

void HRVOSimulator::configureHRVORobot(const Robot &robot,
                                       const RobotConstants_t &robot_constants,
                                       Duration time_step)
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
    double max_radius = (max_speed * time_step.toSeconds()) / 2;

    RobotPath path = RobotPath({PathPoint(destination_point, speed_at_goal)}, max_radius);

    std::shared_ptr<HRVOAgent> agent = std::make_shared<HRVOAgent>(
        robot.id(), robot.currentState(), path, ROBOT_MAX_RADIUS_METERS, max_speed,
        max_accel, FRIENDLY_ROBOT_RADIUS_MAX_INFLATION);
    robots[robot.id()] = std::static_pointer_cast<Agent>(agent);
}

void HRVOSimulator::configureLVRobot(const Robot &robot,
                                     const RobotConstants_t &robot_constants,
                                     Duration time_step)
{
    // Assume that enemy robots will continue to move in their current direction.
    Point destination = robot.position() + 2 * robot.velocity();
    double max_speed  = robot_constants.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    double path_radius = (robot.velocity().length() * time_step.toSeconds()) / 2;

    RobotPath path = RobotPath({PathPoint(destination, 0.0)}, path_radius);

    std::shared_ptr<LVAgent> agent = std::make_shared<LVAgent>(
        robot.id(), robot.currentState(), path, ROBOT_MAX_RADIUS_METERS, max_speed, 0.0,
        ENEMY_ROBOT_RADIUS_MAX_INFLATION);

    robots[robot.id() + ENEMY_LV_ROBOT_OFFSET] = std::static_pointer_cast<Agent>(agent);
}

void HRVOSimulator::doStep(Duration time_step)
{
    if (time_step.toSeconds() == 0.0)
    {
        LOG(WARNING) << "Simulator time step is zero";
        return;
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

void HRVOSimulator::visualize(unsigned int robot_id, TeamColour friendly_team_colour)
{
    auto friendly_robot = robots.find(robot_id);
    if (friendly_robot == robots.end())
    {
        LOG(WARNING) << "Attempt to visualize untracked robot";
        return;
    }

    std::shared_ptr<HRVOAgent> robot =
        std::static_pointer_cast<HRVOAgent>(friendly_robot->second);

    robot->visualize(friendly_team_colour);
}

void HRVOSimulator::updateRobotVelocity(RobotId robot_id, const Vector &new_velocity)
{
    auto hrvo_agent = robots.find(robot_id);
    if (hrvo_agent != robots.end())
    {
        hrvo_agent->second->setVelocity(new_velocity);
    }
}

Vector HRVOSimulator::getRobotVelocity(unsigned int robot_id) const
{
    auto hrvo_agent = robots.find(robot_id);
    if (hrvo_agent != robots.end())
    {
        return hrvo_agent->second->getVelocity();
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
