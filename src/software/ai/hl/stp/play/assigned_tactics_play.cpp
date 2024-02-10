#include "software/ai/hl/stp/play/assigned_tactics_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

AssignedTacticsPlay::AssignedTacticsPlay(TbotsProto::AiConfig config)
    : Play(config, false),
      assigned_tactics(),
      override_motion_constraints(),
      obstacle_factory(config.robot_navigation_obstacle_config())
{
}

void AssignedTacticsPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                         const World &world)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

void AssignedTacticsPlay::updateControlParams(
    std::map<RobotId, std::shared_ptr<Tactic>> assigned_tactics,
    std::map<RobotId, std::set<TbotsProto::MotionConstraint>> motion_constraints)
{
    this->assigned_tactics            = assigned_tactics;
    this->override_motion_constraints = motion_constraints;
}

std::unique_ptr<TbotsProto::PrimitiveSet> AssignedTacticsPlay::get(
    const World &world, const InterPlayCommunication &,
    const SetInterPlayCommunicationCallback &)
{
    obstacle_list.Clear();
    path_visualization.Clear();

    auto primitives_to_run = std::make_unique<TbotsProto::PrimitiveSet>();
    for (const auto &robot : world.friendlyTeam().getAllRobots())
    {
        if (assigned_tactics.contains(robot.id()))
        {
            auto tactic = assigned_tactics.at(robot.id());
            tactic_robot_id_assignment.emplace(tactic, robot.id());
            auto motion_constraints =
                buildMotionConstraintSet(world.gameState(), *goalie_tactic);
            if (override_motion_constraints.contains(robot.id()))
            {
                motion_constraints = override_motion_constraints.at(robot.id());
            }
            auto primitives = tactic->get(world);
            CHECK(primitives.contains(robot.id()))
                << "Couldn't find a primitive for robot id " << robot.id();
            auto primitive_proto = primitives[robot.id()]->generatePrimitiveProtoMessage(
                world, motion_constraints, obstacle_factory);
            primitives_to_run->mutable_robot_primitives()->insert(
                {robot.id(), *primitive_proto});
            tactic->setLastExecutionRobot(robot.id());

            primitives[robot.id()]->getVisualizationProtos(obstacle_list,
                                                           path_visualization);
        }
    }
    primitives_to_run->mutable_time_sent()->set_epoch_timestamp_seconds(
        world.getMostRecentTimestamp().toSeconds());

    // Visualize all obstacles and paths
    LOG(VISUALIZE) << obstacle_list;
    LOG(VISUALIZE) << path_visualization;

    return primitives_to_run;
}

void AssignedTacticsPlay::updateTactics(const PlayUpdate &play_update) {}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, AssignedTacticsPlay, TbotsProto::AiConfig>
    factory;
