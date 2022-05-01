#include "software/ai/hl/stp/play/assigned_tactics_play.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

AssignedTacticsPlay::AssignedTacticsPlay(std::shared_ptr<const AiConfig> config)
    : Play(config, false), assigned_tactics(), override_motion_constraints()
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
    const GlobalPathPlannerFactory &path_planner_factory, const World &world,
    const InterPlayCommunication &, const SetInterPlayCommunicationCallback &)
{
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
            auto primitives = getPrimitivesFromTactic(path_planner_factory, world, tactic,
                                                      motion_constraints)
                                  ->robot_primitives();
            CHECK(primitives.contains(robot.id()))
                << "Couldn't find a primitive for robot id " << robot.id();
            auto primitive = primitives.at(robot.id());
            primitives_to_run->mutable_robot_primitives()->insert(
                google::protobuf::MapPair(robot.id(), primitive));
            tactic->setLastExecutionRobot(robot.id());
        }
    }
    return primitives_to_run;
}

void AssignedTacticsPlay::updateTactics(const PlayUpdate &play_update) {}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, AssignedTacticsPlay, AiConfig> factory;
