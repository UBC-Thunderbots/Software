#include "software/ai/ai.h"

#include <chrono>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/play_factory.h"

AI::AI(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      fsm(std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config})),
      override_play(nullptr),
      current_play(std::make_unique<HaltPlay>(ai_config)),
      field_to_path_planner_factory(),
      prev_override()
{
}

void AI::overridePlay(std::unique_ptr<Play> play)
{
    override_play = std::move(play);
}

void AI::overridePlayFromProto(TbotsProto::Play play_proto)
{
    overridePlay(std::move(createPlay(play_proto, ai_config)));
}

void AI::checkAiConfig()
{
    if (ai_config.ai_control_config().has_override_ai_play())
    {
        auto current_override = ai_config.ai_control_config().override_ai_play();

        if (current_override && (current_override != prev_override))
        {
            TbotsProto::Play play_proto;
            play_proto.set_name(current_override);
            overridePlayFromProto(play_proto);
        }

        prev_override = current_override;
    }
}

std::unique_ptr<TbotsProto::PrimitiveSet> AI::getPrimitives(const World& world)
{
    checkAiConfig();
    fsm->process_event(PlaySelectionFSM::Update(
        [this](std::unique_ptr<Play> play) { current_play = std::move(play); },
        world.gameState()));

    if (!field_to_path_planner_factory.contains(world.field()))
    {
        field_to_path_planner_factory.emplace(
            world.field(),
            GlobalPathPlannerFactory(ai_config.robot_navigation_obstacle_config(),
                                     world.field()));
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
