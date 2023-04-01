#include "hrvo_simulator.h"

HRVOSimulator::HRVOSimulator() : robots(), world(std::nullopt), primitive_set() {}

void HRVOSimulator::updateWorld(const World &world,
                                const RobotConstants_t &robot_constants,
                                Duration time_step)
{
    this->world = world;
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
    double max_accel         = 1e-4;
    double max_speed         = 1e-4;
    double max_angular_speed = 1e-4;
    double max_angular_accel = 1e-4;
    const std::set<RobotCapability> &unavailable_capabilities =
        robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) ==
                    unavailable_capabilities.end();
    if (can_move)
    {
        max_speed = robot_constants.robot_max_speed_m_per_s;
        max_accel = robot_constants.robot_max_acceleration_m_per_s_2;

        max_angular_speed = robot_constants.robot_max_ang_speed_rad_per_s;
        max_angular_accel = robot_constants.robot_max_ang_acceleration_rad_per_s_2;
    }

    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Point destination_point      = robot.position();
    double speed_at_goal         = 0.0;
    Angle angle_at_goal          = robot.orientation();
    const auto &robot_primitives = primitive_set.robot_primitives();
    auto primitive_iter          = robot_primitives.find(robot.id());
    if (primitive_iter != robot_primitives.end())
    {
        TbotsProto::Primitive primitive = primitive_iter->second;

        // TODO (#2873): This code block is repeated inside HRVOAgent.cpp.
        // and it just calculates the path point from the primitive.
        // this can be factored out to a function, so that its usage can called by the
        // simulator and for each agent.
        if (primitive.has_move())
        {
            const auto &move_primitive = primitive.move();
            const auto &motion_control = move_primitive.motion_control();
            // TODO (#2418): Update implementation of Primitive to support
            // multiple path points and remove this check
            if (motion_control.path().points().size() < 2)
            {
                LOG(WARNING) << "Empty path: " << motion_control.path().points().size()
                             << std::endl;
                return;
            }

            speed_at_goal = move_primitive.final_speed_m_per_s();
            max_speed     = move_primitive.max_speed_m_per_s();
            angle_at_goal = createAngle(move_primitive.final_angle());

            auto destination  = motion_control.path().points().at(1);
            destination_point = Point(destination.x_meters(), destination.y_meters());
        }
    }

    // Max distance which the robot can travel in one time step + scaling
    double max_radius = (max_speed * time_step.toSeconds()) / 2;

    auto path_points = {PathPoint(destination_point, speed_at_goal, angle_at_goal)};
    RobotPath path   = RobotPath(path_points, max_radius);

    std::shared_ptr<HRVOAgent> agent = std::make_shared<HRVOAgent>(
        robot.id(), robot.currentState(), path, ROBOT_MAX_RADIUS_METERS, max_speed,
        max_accel, max_angular_speed, max_angular_accel,
        FRIENDLY_ROBOT_RADIUS_MAX_INFLATION);
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

    RobotPath path = RobotPath(
        {PathPoint(destination, 0.0, robot.currentState().orientation())}, path_radius);

    std::shared_ptr<LVAgent> agent = std::make_shared<LVAgent>(
        robot.id(), robot.currentState(), path, ROBOT_MAX_RADIUS_METERS, max_speed, 0.0,
        0.0, 0.0, ENEMY_ROBOT_RADIUS_MAX_INFLATION); // TODO: Double check the 0.0s

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
    // loops are separated so that all robot fields that need to updated,
    // are updated separately. Otherwise, some robots would already be updated with new
    // velocity, while others aren't.
    for (auto &robot : robots)
    {
        robot.second->computeNewVelocity(robots, time_step);
    }

    for (auto &robot : robots)
    {
        robot.second->computeNewAngularVelocity(time_step);
    }

    // Update the positions of all agents given their velocity
    for (auto &robot : robots)
    {
        robot.second->update(time_step);
    }
}

void HRVOSimulator::visualize(unsigned int robot_id, TeamColour friendly_team_colour)
{
    if (robots.empty())
    {
        return;
    }

    auto friendly_robot = robots.find(robot_id);
    if (friendly_robot == robots.end())
    {
        LOG(WARNING) << "Attempt to visualize untracked robot with id " << robot_id;
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


void HRVOSimulator::updateRobotAngularVelocity(
    RobotId robot_id, const AngularVelocity &new_angular_velocity)
{
    auto hrvo_agent = robots.find(robot_id);
    if (hrvo_agent != robots.end())
    {
        hrvo_agent->second->setAngularVelocity(new_angular_velocity);
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


AngularVelocity HRVOSimulator::getRobotAngularVelocity(unsigned int robot_id) const
{
    auto hrvo_agent = robots.find(robot_id);
    if (hrvo_agent != robots.end())
    {
        return hrvo_agent->second->getAngularVelocity();
    }
    LOG(WARNING) << "Angular velocity for robot " << robot_id
                 << " can not be found since it does not exist in HRVO Simulator"
                 << std::endl;
    return AngularVelocity();
}


std::size_t HRVOSimulator::getRobotCount()
{
    return robots.size();
}

std::map<RobotId, std::shared_ptr<Agent>> HRVOSimulator::getRobots()
{
    return robots;
}
