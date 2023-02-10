#include "hrvo_simulator.h"
#include "software/ai/navigator/path_planner/hrvo/path_point.h"
#include "software/ai/navigator/path_planner/hrvo/robot_path.h"

HRVOSimulator::HRVOSimulator(float time_step, const RobotConstants_t &robot_constants,
                             const TeamColour friendly_team_colour) :
        robot_constants(robot_constants),
        primitive_set(),
        world(std::nullopt),
        robots(),
        friendly_team_colour(friendly_team_colour) {
}

void HRVOSimulator::updateWorld(const World &world) {
    this->world = world;
    const auto &world_friendly_team = world.friendlyTeam().getAllRobots();
    const auto &world_enemy_team = world.enemyTeam().getAllRobots();
    // TODO (#2498): Update implementation to correctly support adding and removing agents
    //               to represent the newly added and removed friendly/enemy robots in the
    //               World.
    robots.clear();
    for (const Robot &friendly_robot: world_friendly_team) {
        configureHRVORobot(friendly_robot);
    }

    for (const Robot &enemy_robot: world_enemy_team) {
//        TODO
//        std::size_t robot =
//                addLinearVelocityRobotAgent(enemy_robot, goal_position);
//        enemy_team.insert(enemy_robot.id(), agent_index);
    }
}


void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set) {
    primitive_set = new_primitive_set;

    // Update all friendly agent's primitives
    for (auto &[robot_id, primitive]: primitive_set.robot_primitives()) {
        auto agent = robots[robot_id];
        if (world.has_value()) {
            // TODO time_step
            agent->updatePrimitive(primitive, world.value(), 1 / 60);
        }
    }
}

void HRVOSimulator::configureHRVORobot(const Robot &robot) {
    float max_accel = 1e-4;
    float max_speed = 1e-4;
    // TODO move this logic to constructor for agent
    const std::set<RobotCapability> &unavailable_capabilities =
            robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) ==
                    unavailable_capabilities.end();
    if (can_move) {
        max_accel = robot_constants.robot_max_acceleration_m_per_s_2;
        max_speed = robot_constants.robot_max_speed_m_per_s;
    }

    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Point destination_point = robot.position();
    double speed_at_goal = 0.0;
    const auto &robot_primitives = primitive_set.robot_primitives();
    auto primitive_iter = robot_primitives.find(robot.id());
    if (primitive_iter != robot_primitives.end()) {
        TbotsProto::Primitive primitive = primitive_iter->second;
        TbotsProto::Point destination_point_proto;

        if (primitive.has_move()) {
            const auto &move_primitive = primitive.move();
            // TODO (#2418): Update implementation of Primitive to support
            // multiple path points and remove this check
            CHECK(move_primitive.motion_control().path().points().size() >= 2)
                << "Empty path: "
                << move_primitive.motion_control().path().points().size() << std::endl;
            destination_point_proto =
                    move_primitive.motion_control().path().points().at(1);
            destination_point =
                    Point(static_cast<float>(destination_point_proto.x_meters()),
                           static_cast<float>(destination_point_proto.y_meters()));
            speed_at_goal = move_primitive.final_speed_m_per_s();
            max_speed = move_primitive.max_speed_m_per_s();
        }
    }

    // Max distance which the robot can travel in one time step + scaling
    float path_radius = (max_speed * (1 / 60)) / 2;

    RobotPath path =
            RobotPath({PathPoint(destination_point, speed_at_goal)}, path_radius);

    std::shared_ptr<Agent> agent = std::make_shared<HRVOAgent>(robot.id(), robot.currentState(),
                                                                   TeamSide::FRIENDLY, path_radius, path, max_speed, max_accel,
                                                                   FRIENDLY_ROBOT_RADIUS_MAX_INFLATION);
    robots.insert({robot.id(), agent});

}

void HRVOSimulator::addLinearVelocityRobotAgent(const Robot &robot, const Vector &destination, TeamSide type) {
    // TODO move this stuff
    // Set goal of enemy robot to be the farthest point, when moving in the
    // current direction
    Segment segment(robot.position(),
                    robot.position() + robot.velocity() * 100);

    // Enemy robot should not enter the friendly defense area
    std::unordered_set<Point> intersection_point_set =
            intersection(world->field().friendlyDefenseArea(), segment);
    if (intersection_point_set.empty() &&
        contains(world->field().fieldLines(), robot.position())) {
        // If the robot is in the field, then move in the current direction
        // towards the field edge
        intersection_point_set =
                intersection(world->field().fieldLines(), segment);
    }

    if (intersection_point_set.empty()) {
        // If there is no intersection point (robot is outside the field),
        // continue moving in the current direction
        intersection_point_set.insert(robot.position() +
                                      robot.velocity() * 5);
    }

    Vector goal_position = intersection_point_set.begin()->toVector();

    Vector position = robot.position().toVector();
    Vector velocity = robot.velocity();
    float max_accel = 0.f;
    float max_speed = robot_constants.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    // TODO no time step initially passed
    float path_radius = (max_speed * (1 / 60)) / 2;

    RobotPath path = RobotPath({PathPoint(destination, 0.0f)}, path_radius);
    //addLinearVelocityAgent(position, ROBOT_MAX_RADIUS_METERS,
//                                  ENEMY_ROBOT_RADIUS_MAX_INFLATION, velocity, max_speed,
//                                  max_accel, path, robot.id(), type);

    std::shared_ptr<LVAgent> agent = std::make_shared<LVAgent>(robot.id(), robot.currentState(),
                                                               type, position, ROBOT_MAX_RADIUS_METERS,
                                                               max_radius_inflation, curr_velocity, max_speed,
                                                               max_accel, path, robot_id, type);

    robots.push_back(std::move(agent));
    return agents.size() - 1;
}

void HRVOSimulator::doStep(double time_step) {

    if (time_step == 0.0f) {
        throw std::runtime_error("Time step is zero");
    }


    if (robots.empty()) {
        return;
    }

    // Update all the hrvo robots velocities radii based on their current velocity
    for (auto &robot: robots) {
        // Linearly increase radius based on the current agent velocity
        robot->updateRadiusFromVelocity();
    }

    // Compute what velocity each agent will take next
    for (auto &robot: friendly_team) {
        robot->computeNewVelocity(agents, time_step);
    }

    // Update the positions of all agents given their velocity
    for (auto &agent: agents) {
        agent->update(time_step);
    }
}

void HRVOSimulator::visualize(unsigned int robot_id)
{
    std::shared_ptr<HRVOAgent> robot = std::static_pointer_cast<HRVOAgent>(robots[robot_id]);
    std::vector<TbotsProto::VelocityObstacle> velocity_obstacles;
    for (const VelocityObstacle &vo : velocity_obstacles)
    {
        velocity_obstacles.emplace_back(*createVelocityObstacleProto(vo, getPosition()));
    }
    return;
}


