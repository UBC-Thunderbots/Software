#include "new_simulator.h"

HRVOSimulatorNew::HRVOSimulatorNew(float time_step, const RobotConstants_t &robot_constants,
                                   const TeamColour friendly_team_colour):
    robot_constants(robot_constants),
    agent_visitor(3),
    primitive_set(),
    global_time(0.0f),
    time_step(time_step),
    kd_tree(std::make_unique<KdTree>(this)),
    world(std::nullopt),
    agents(),
    friendly_team_colour(friendly_team_colour)
{
}

void HRVOSimulatorNew::updateWorld(const World &world)
{
    this->world               = world;
    const auto &friendly_team = world.friendlyTeam().getAllRobots();
    const auto &enemy_team    = world.enemyTeam().getAllRobots();

    updateRemovedAgents(world);

    // update agents
    for (const Robot &friendly_robot : friendly_team)
    {
        auto hrvo_agent = getFriendlyAgentFromRobotId(friendly_robot.id());
        if (hrvo_agent.has_value())
        {
            hrvo_agent.value()->setPosition(friendly_robot.position().toVector());
            // We do not use velocity feedback for friendly robots as it results
            // in the robots not being able to accelerate properly.
        }
        else
        {
            addHRVORobotAgent(friendly_robot, TeamSide::FRIENDLY);
        }
    }

    for (const Robot &enemy_robot : enemy_team)
    {
        auto agent_iter = std::find_if(
                agents.begin(), agents.end(), [&enemy_robot](std::shared_ptr<Agent> agent) {
                    return (agent->getRobotId() == enemy_robot.id() &&
                            agent->getAgentType() == TeamSide::ENEMY);
                });

        if (agent_iter != agents.end())
        {
            (*agent_iter)->setPosition(enemy_robot.position().toVector());
            (*agent_iter)->setVelocity(enemy_robot.velocity());
        }
        else
        {
            Vector destination =
                    (enemy_robot.position() + enemy_robot.velocity() * 5).toVector();
            addLinearVelocityRobotAgent(enemy_robot, destination, TeamSide::ENEMY);
        }
    }
}

void HRVOSimulatorNew::computeNeighbors(const HRVOAgent &agent)
{
    agent->
    const auto kek = agent.pref_speed_;
    const auto current_path_point_opt = agent.getCurrentPathPoint();
    if (!current_path_point_opt.has_value())
    {
        // Don't draw any velocity obstacles if we do not have a destination
        return;
    }

    // Only consider agents within this distance away from our position
    auto current_destination = current_path_point_opt.value().getPosition();
    double dist_to_obstacle_threshold =
            std::min(static_cast<double>(max_neighbor_dist),
                     (getPosition() - current_destination).length());
    // Re-calculate all agents (neighbors) within the distance threshold
    // which we want to create velocity obstacles for
    kd_tree()->query(agent, neighbor_dist_threshold);
}

void HRVOSimulatorNew::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set)
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

std::size_t HRVOSimulatorNew::addHRVORobotAgent(const Robot &robot, TeamSide type) {
    return 0;
}

std::size_t
HRVOSimulatorNew::addLinearVelocityRobotAgent(const Robot &robot, const Vector &destination, TeamSide type) {
    return 0;
}

void HRVOSimulatorNew::doStep()
{
    if (kd_tree == nullptr)
    {
        throw std::runtime_error(
                "Simulation not initialized when attempting to do step.");
    }

    if (time_step == 0.0f)
    {
        throw std::runtime_error("Time step not set when attempting to do step.");
    }

    reached_goals = true;

    if (agents.size() == 0)
    {
        return;
    }

    kd_tree->build();

    // Update all agent radii based on their velocity
    for (auto &agent : agents)
    {
        agent->updateRadiusFromVelocity();
    }

    // Compute what velocity each agent will take next
    for (auto &agent : agents)
    {
        agent->computeNewVelocity(time_step);
    }

    // Update the positions of all agents given their velocity
    for (auto &agent : agents)
    {
        agent->update(time_step);
    }

    global_time += time_step;
}
