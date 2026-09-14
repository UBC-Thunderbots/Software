#include "software/ai/ai.h"

#include <Tracy.hpp>

#include "software/ai/hl/stp/play/play_factory.h"
#include "software/tracy/tracy_constants.h"


Ai::Ai(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : ai_config_ptr(ai_config_ptr),
      fsm(std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config_ptr})),
      ai_config_changed(false)
{
    auto current_override = ai_config_ptr->ai_control_config().override_ai_play();
    if (current_override != TbotsProto::PlayName::UseAiSelection)
    {
        // Override to new play if we're not running Ai Selection
        TbotsProto::Play play_proto;
        play_proto.set_name(current_override);
        overridePlayFromProto(play_proto);
    }
}

void Ai::overridePlay(std::unique_ptr<Play> play)
{
    fsm->process_event(PlaySelectionFSM::Override(std::move(play)));
}

void Ai::overridePlayFromProto(TbotsProto::Play play_proto)
{
    overridePlay(std::move(createPlay(play_proto, ai_config_ptr)));
}

void Ai::updateAiConfig()
{
    ai_config_changed = true;
}

void Ai::checkAiConfig()
{
    if (ai_config_changed)
    {
        auto current_override = ai_config_ptr->ai_control_config().override_ai_play();
        std::unique_ptr<Play> override_play;
        if (current_override != TbotsProto::PlayName::UseAiSelection)
        {
            TbotsProto::Play play_proto;
            play_proto.set_name(current_override);
            override_play = createPlay(play_proto, ai_config_ptr);
        }

        fsm->process_event(PlaySelectionFSM::Reset(std::move(override_play)));
        ai_config_changed = false;
    }
}

std::unique_ptr<TbotsProto::PrimitiveSet> Ai::getPrimitives(const WorldPtr& world_ptr)
{
    FrameMarkStart(TracyConstants::AI_FRAME_MARKER);

    checkAiConfig();

    fsm->process_event(PlaySelectionFSM::Update(world_ptr->gameState(), *ai_config_ptr));

    auto primitive_set = static_cast<PlaySelectionFSM&>(*fsm).getSelectedPlay().get(
        world_ptr, inter_play_communication,
        [this](InterPlayCommunication comm)
        { inter_play_communication = std::move(comm); });

    FrameMarkEnd(TracyConstants::AI_FRAME_MARKER);

    return primitive_set;
}

TbotsProto::PlayInfo Ai::getPlayInfo() const
{
    Play& selected_play = static_cast<const PlaySelectionFSM&>(*fsm).getSelectedPlay();
    const std::vector<std::string> play_state = selected_play.getState();
    auto tactic_robot_id_assignment = selected_play.getTacticRobotIdAssignment();

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
