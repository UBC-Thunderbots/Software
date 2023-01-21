#include "new_simulator.h"

MotionPlanningSimulator::MotionPlanningSimulator(float time_step, const RobotConstants_t &robot_constants,
                                                 const TeamColour friendly_team_colour):
    robot_constants(robot_constants),
    primitive_set(),
    world(std::nullopt),
    agents(),
    friendly_robot_id_map(),
    enemy_robot_id_map(),
    friendly_team_colour(friendly_team_colour)
{
}

void MotionPlanningSimulator::updateWorld(const World &world)
{
    this->world               = world;
    const auto &friendly_team = world.friendlyTeam().getAllRobots();
    const auto &enemy_team    = world.enemyTeam().getAllRobots();
    // TODO (#2498): Update implementation to correctly support adding and removing agents
    //               to represent the newly added and removed friendly/enemy robots in the
    //               World.
    if (friendly_robot_id_map.empty() && enemy_robot_id_map.empty())
    {
        for (const Robot &friendly_robot : friendly_team)
        {
            std::size_t agent_index = addHRVORobotAgent(friendly_robot);
            friendly_robot_id_map.emplace(friendly_robot.id(), agent_index);
        }

        for (const Robot &enemy_robot : enemy_team)
        {
            // Set goal of enemy robot to be the farthest point, when moving in the
            // current direction
            Segment segment(enemy_robot.position(),
                            enemy_robot.position() + enemy_robot.velocity() * 100);

            // Enemy robot should not enter the friendly defense area
            std::unordered_set<Point> intersection_point_set =
                    intersection(world.field().friendlyDefenseArea(), segment);
            if (intersection_point_set.empty() &&
                contains(world.field().fieldLines(), enemy_robot.position()))
            {
                // If the robot is in the field, then move in the current direction
                // towards the field edge
                intersection_point_set =
                        intersection(world.field().fieldLines(), segment);
            }

            if (intersection_point_set.empty())
            {
                // If there is no intersection point (robot is outside the field),
                // continue moving in the current direction
                intersection_point_set.insert(enemy_robot.position() +
                                              enemy_robot.velocity() * 5);
            }

            Vector goal_position = intersection_point_set.begin()->toVector();
            std::size_t agent_index =
                    addLinearVelocityRobotAgent(enemy_robot, goal_position);
            enemy_robot_id_map.emplace(enemy_robot.id(), agent_index);
        }
    }
    else
    {
        // Update Agents
        for (const Robot &friendly_robot : friendly_team)
        {
            auto hrvo_agent = getFriendlyAgentFromRobotId(friendly_robot.id());
            if (hrvo_agent.has_value())
            {
                hrvo_agent.value()->setPosition(friendly_robot.position().toVector());
                // We do not use velocity feedback for friendly robots as it results
                // in the robots not being able to accelerate properly.
            }
        }

        for (const Robot &enemy_robot : enemy_team)
        {
            auto agent_index_iter = enemy_robot_id_map.find(enemy_robot.id());
            if (agent_index_iter != enemy_robot_id_map.end())
            {
                unsigned int agent_index = agent_index_iter->second;
                agents[agent_index].setPosition(enemy_robot.position().toVector());
                agents[agent_index].setVelocity(enemy_robot.velocity());
            }
        }
    }
}


void MotionPlanningSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set)
{
    primitive_set = new_primitive_set;

    // Update all friendly agent's primitives
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        auto hrvo_agent_opt = getFriendlyAgentFromRobotId(robot_id);
        if (hrvo_agent_opt.has_value() && world.has_value())
        {
            hrvo_agent_opt.value()->updatePrimitive(primitive, world.value());
        }
    }
}

std::size_t MotionPlanningSimulator::addHRVORobotAgent(const Robot &robot, TeamSide type) {
    return 0;
}

std::size_t
MotionPlanningSimulator::addLinearVelocityRobotAgent(const Robot &robot, const Vector &destination, TeamSide type) {
    return 0;
}

void MotionPlanningSimulator::doStep(double time_step)
{

    if (time_step == 0.0f)
    {
        throw std::runtime_error("Time step is zero");
    }


    if (agents.empty())
    {
        return;
    }

    // Update all agent radii based on their velocity
    for (auto &agent : agents)
    {
        agent->updateRadiusFromVelocity();
    }

    // Compute what velocity each agent will take next
    for (auto &agent : agents)
    {
        agent->computeNewVelocity(agents, time_step);
    }

    // Update the positions of all agents given their velocity
    for (auto &agent : agents)
    {
        agent->update(time_step);
    }
}

std::optional<std::shared_ptr<HRVOAgent>> MotionPlanningSimulator::getFriendlyAgentFromRobotId(
        unsigned int robot_id) const
{
    auto agent_index_iter = friendly_robot_id_map.find(robot_id);
    if (agent_index_iter != friendly_robot_id_map.end())
    {
        unsigned int agent_index = agent_index_iter->second;
        auto hrvo_agent = std::static_pointer_cast<HRVOAgent>(agents[agent_index]);
        if (hrvo_agent != nullptr)
        {
            return hrvo_agent;
        }
    }
    return std::nullopt;
}
