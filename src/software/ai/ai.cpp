#include "software/ai/ai.h"

#include <chrono>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/play_factory.h"

AI::AI(TbotsProto::AiConfig ai_config)
    : ai_config_(ai_config),
      fsm(std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config})),
      override_play(nullptr),
      current_play(std::make_unique<HaltPlay>(ai_config)),
      field_to_path_planner_factory(),
      prev_override(TbotsProto::PlayName::UseAiSelection)
{
}

void AI::overridePlay(std::unique_ptr<Play> play)
{
    override_play = std::move(play);
}

void AI::overridePlayFromProto(TbotsProto::Play play_proto)
{
    overridePlay(std::move(createPlay(play_proto, ai_config_)));
}

void AI::updateAiConfig(TbotsProto::AiConfig ai_config)
{
    ai_config_        = ai_config;
    ai_config_changed = true;
}

void AI::checkAiConfig()
{
    auto current_override = ai_config_.ai_control_config().override_ai_play();

    // If we have a new override, and its not back to the Ai selection,
    // lets override the play
    if (current_override != prev_override &&
        current_override != TbotsProto::PlayName::UseAiSelection)
    {
        TbotsProto::Play play_proto;
        play_proto.set_name(current_override);
        overridePlayFromProto(play_proto);
    }

    // If we have a new override but its back to the Ai selection, lets
    // clear the override
    if (current_override != prev_override &&
        current_override == TbotsProto::PlayName::UseAiSelection)
    {
        overridePlay(nullptr);
    }
    prev_override = current_override;
}

std::unique_ptr<TbotsProto::PrimitiveSet> AI::getPrimitives(const World& world)
{
    checkAiConfig();
    fsm->process_event(PlaySelectionFSM::Update(
        [this](std::unique_ptr<Play> play) { current_play = std::move(play); },
        world.gameState(), ai_config_));

    if (!field_to_path_planner_factory.contains(world.field()) || ai_config_changed)
    {
        field_to_path_planner_factory.insert_or_assign(
            world.field(),
            GlobalPathPlannerFactory(ai_config_.robot_navigation_obstacle_config(),
                                     world.field()));
        ai_config_changed = false;
    }

    if (static_cast<bool>(override_play))
    {
        return override_play->get(field_to_path_planner_factory.at(world.field()), world,
                                  inter_play_communication,
                                  [this](InterPlayCommunication comm) {
                                      inter_play_communication = std::move(comm);
                                  });
    }
    else
    {
        return current_play->get(field_to_path_planner_factory.at(world.field()), world,
                                 inter_play_communication,
                                 [this](InterPlayCommunication comm) {
                                     inter_play_communication = std::move(comm);
                                 });
    }
}

TbotsProto::PlayInfo AI::getPlayInfo() const
{
    std::string info_play_name      = objectTypeName(*current_play);
    auto tactic_robot_id_assignment = current_play->getTacticRobotIdAssignment();

    if (static_cast<bool>(override_play))
    {
        info_play_name             = objectTypeName(*override_play);
        tactic_robot_id_assignment = override_play->getTacticRobotIdAssignment();
    }

    TbotsProto::PlayInfo info;
    info.mutable_play()->set_play_name(info_play_name);

    for (const auto& [tactic, robot_id] : tactic_robot_id_assignment)
    {
        TbotsProto::PlayInfo_Tactic tactic_msg;
        tactic_msg.set_tactic_name(objectTypeName(*tactic));
        tactic_msg.set_tactic_fsm_state(tactic->getFSMState());
        (*info.mutable_robot_tactic_assignment())[robot_id] = tactic_msg;
    }

    return info;
}
