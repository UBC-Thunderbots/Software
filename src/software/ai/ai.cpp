#include "software/ai/ai.h"

#include <chrono>

#include "software/ai/hl/stp/play/halt_play.h"

AI::AI(std::shared_ptr<const AiConfig> ai_config)
    : ai_config(ai_config),
      fsm(std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{ai_config})),
      override_play(nullptr),
      current_play(std::make_unique<HaltPlay>(ai_config)),
      field_to_path_planner_factory(),
      prev_override(false),
      prev_override_name("")
{
}

void AI::overridePlay(std::unique_ptr<Play> play)
{
    override_play = std::move(play);
}

void AI::overridePlayFromName(std::string name)
{
    overridePlay(GenericFactory<std::string, Play, AiConfig>::create(name, ai_config));
}

void AI::checkAiConfig()
{
    bool current_override = ai_config->getAiControlConfig()->getOverrideAiPlay()->value();
    std::string current_override_name =
        ai_config->getAiControlConfig()->getCurrentAiPlay()->value();
    if (current_override && (current_override != prev_override ||
                             current_override_name != prev_override_name))
    {
        overridePlayFromName(
            ai_config->getAiControlConfig()->getCurrentAiPlay()->value());
    }
    prev_override      = current_override;
    prev_override_name = current_override_name;
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
            GlobalPathPlannerFactory(ai_config->getRobotNavigationObstacleConfig(),
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
    std::vector<std::string> play_state = current_play->getState();
    auto tactic_robot_id_assignment     = current_play->getTacticRobotIdAssignment();

    if (static_cast<bool>(override_play))
    {
        play_state                 = current_play->getState();
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
