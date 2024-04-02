#include "software/ai/ai.h"

#include <Tracy.hpp>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/assigned_tactics_play.h"
#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/tactic_factory.h"
#include "software/tracy/tracy_constants.h"


Ai::Ai(std::shared_ptr<Strategy> strategy)
    : strategy(strategy),
      fsm(std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{strategy})),
      override_play(nullptr),
      current_play(std::make_shared<HaltPlay>(strategy))
{
}

void Ai::overridePlay(std::unique_ptr<Play> play)
{
    override_play = std::move(play);
}

void Ai::overridePlayFromProto(const TbotsProto::Play& play_proto)
{
    overridePlay(std::move(createPlay(play_proto, strategy)));
}

void Ai::overrideTactics(
    const TbotsProto::AssignedTacticPlayControlParams& assigned_tactic_play_control_params)
{
    auto play = std::make_unique<AssignedTacticsPlay>(strategy);
    std::map<RobotId, std::shared_ptr<Tactic>> tactic_assignment_map;

    // override to stop primitive
    for (auto& assigned_tactic : assigned_tactic_play_control_params.assigned_tactics())
    {
        tactic_assignment_map[assigned_tactic.first] =
            createTactic(assigned_tactic.second, strategy);
    }

    play->updateControlParams(tactic_assignment_map);
    overridePlay(std::move(play));

    LOG(VISUALIZE) << getPlayInfo();
}

std::unique_ptr<TbotsProto::PrimitiveSet> Ai::getPrimitives(const World& world)
{
    FrameMarkStart(TracyConstants::AI_FRAME_MARKER);
    
    WorldPtr world_ptr = std::make_shared<const World>(world); 
    
    strategy->updateWorld(world_ptr);

    fsm->process_event(PlaySelectionFSM::Update(
        [this](std::shared_ptr<Play> play) { current_play = play; }, world_ptr));

    std::unique_ptr<TbotsProto::PrimitiveSet> primitive_set;
    if (static_cast<bool>(override_play))
    {
        primitive_set = override_play->get(world_ptr, inter_play_communication,
                                           [this](InterPlayCommunication comm) {
                                               inter_play_communication = std::move(comm);
                                           });
    }
    else
    {
        primitive_set = current_play->get(world_ptr, inter_play_communication,
                                          [this](InterPlayCommunication comm) {
                                              inter_play_communication = std::move(comm);
                                          });
    }

    FrameMarkEnd(TracyConstants::AI_FRAME_MARKER);

    return primitive_set;
}

TbotsProto::PlayInfo Ai::getPlayInfo() const
{
    std::vector<std::string> play_state = current_play->getState();
    auto tactic_robot_id_assignment     = current_play->getTacticRobotIdAssignment();

    if (static_cast<bool>(override_play))
    {
        play_state                 = override_play->getState();
        tactic_robot_id_assignment = override_play->getTacticRobotIdAssignment();
    }

    TbotsProto::PlayInfo info;

    for (const auto& state : play_state)
    {
        info.mutable_play()->add_play_state(state);
    }

    for (const auto& [tactic, robot_id] : tactic_robot_id_assignment)
    {
        TbotsProto::PlayInfo_Tactic tactic_msg;
        tactic_msg.set_tactic_name(objectTypeName(*tactic));
        tactic_msg.set_tactic_fsm_state(tactic->getFSMState());
        (*info.mutable_robot_tactic_assignment())[robot_id] = tactic_msg;
    }

    return info;
}
