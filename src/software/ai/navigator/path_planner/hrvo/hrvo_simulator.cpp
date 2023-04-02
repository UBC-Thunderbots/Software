#include "hrvo_simulator.h"

HRVOSimulator::HRVOSimulator() : robots(), world(std::nullopt), primitive_set() {}

void HRVOSimulator::updateWorld(const World &world,
                                const RobotConstants_t &robot_constants,
                                Duration time_step)
{
    this->world = world;

    auto &friendlies = world.friendlyTeam();
    auto &enemies    = world.enemyTeam();

    // initialize an array of bits, with each bit corresponding to the robot whose id is
    // the index this keeps track of all the friendly robot ids in the world packet
    std::vector<bool> tracked_friendlies(
        std::max(static_cast<int>(MAX_ROBOT_IDS_PER_SIDE),
                 static_cast<int>(friendlies.numRobots())),
        false);
    std::vector<bool> tracked_enemies(std::max(static_cast<int>(MAX_ROBOT_IDS_PER_SIDE),
                                               static_cast<int>(enemies.numRobots())),
                                      false);

    for (const Robot &friendly_robot : friendlies.getAllRobots())
    {
        auto hrvo_agent = robots.find(friendly_robot.id());

        tracked_friendlies[friendly_robot.id()] = true;

        if (hrvo_agent != robots.end())
        {
            updateAgent(hrvo_agent->second, friendly_robot);
        }
        else
        {
            std::cout << "Adding robot " << std::to_string(friendly_robot.id()) << std::endl;
            configureHRVORobot(friendly_robot, robot_constants, time_step);
        }
    }

    for (unsigned int i = 0; i < tracked_friendlies.size(); ++i)
    {
        if (!tracked_friendlies[i])
        {
            auto robot_it = robots.find(i);
            if (robot_it != robots.end())
            {
                std::cout << "Removing robot " << std::to_string(i) << std::endl;
                robots.erase(robot_it);
            }
        }
    }

    for (const Robot &enemy_robot : enemies.getAllRobots())
    {
        auto lv_agent = robots.find(enemy_robot.id() + ENEMY_LV_ROBOT_OFFSET);
        tracked_enemies[enemy_robot.id()] = true;

        if (lv_agent != robots.end())
        {
            updateAgent(lv_agent->second, enemy_robot);
        }
        else
        {
            std::cout << "Adding robot " << std::to_string(enemy_robot.id() + ENEMY_LV_ROBOT_OFFSET) << std::endl;
            configureLVRobot(enemy_robot, robot_constants, time_step);
        }
    }

    // flip all the tracked bits to get all the robot ids which are NOT in the world
    // packet, and delete enemy agent that aren't present in the world packet.
    for (unsigned int i = 0; i < tracked_enemies.size(); ++i)
    {
        if (!tracked_enemies[i])
        {
            auto robot_it = robots.find(i + ENEMY_LV_ROBOT_OFFSET);
            if (robot_it != robots.end())
            {
                std::cout << "Removing robot " << std::to_string(i) << std::endl;
                robots.erase(robot_it);
            }
        }
    }
    std::cout << "Sim has : ";
    for (auto &[robot_id, robot] : robots)
    {
        std::cout << std::to_string(robot_id) << ", ";
    }
    std::cout << "" << std::endl;
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

void HRVOSimulator::updateAgent(const std::shared_ptr<Agent> &agent,
                                const Robot &friendly_robot)
{
    agent->setPosition(friendly_robot.position());
    agent->setVelocity(friendly_robot.velocity());
    agent->setOrientation(friendly_robot.orientation());
    agent->setAngularVelocity(friendly_robot.angularVelocity());
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
