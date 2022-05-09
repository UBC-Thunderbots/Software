#include "software/ai/hl/stp/play/assigned_tactics_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

AssignedTacticsPlay::AssignedTacticsPlay(TbotsProto::AiConfig config)
    : Play(config, false), assigned_tactics()
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
    std::map<RobotId, std::shared_ptr<Tactic>> assigned_tactics)
{
    this->assigned_tactics = assigned_tactics;
}

std::vector<std::unique_ptr<Intent>> AssignedTacticsPlay::get(
    RobotToTacticAssignmentFunction,
    MotionConstraintBuildFunction motion_constraint_builder, const World &new_world,
    const InterPlayCommunication &, const SetInterPlayCommunicationCallback &)
{
    std::vector<std::unique_ptr<Intent>> intents;

    for (const auto &robot : new_world.friendlyTeam().getAllRobots())
    {
        if (assigned_tactics.contains(robot.id()))
        {
            auto tactic = assigned_tactics.at(robot.id());
            auto intent = tactic->get(robot, new_world);
            intent->setMotionConstraints(motion_constraint_builder(*tactic));
            intents.push_back(std::move(intent));
        }
    }
    return intents;
}

void AssignedTacticsPlay::updateTactics(const PlayUpdate &play_update) {}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, AssignedTacticsPlay, TbotsProto::AiConfig>
    factory;
