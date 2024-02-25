#include "software/ai/ai.h"

#include <chrono>
#include <memory>
#include <utility>

#include "software/ai/hl/stp/play/halt_play.h"
#include "software/ai/hl/stp/play/play_factory.h"

Ai::Ai(std::shared_ptr<Strategy> strategy)
    : strategy(strategy),
      fsm(std::make_unique<FSM<PlaySelectionFSM>>(PlaySelectionFSM{strategy})),
      override_play(nullptr),
      current_play(std::make_shared<HaltPlay>(strategy))
{
    updateOverridePlay();
}

void Ai::overridePlay(std::unique_ptr<Play> play)
{
    override_play = std::move(play);
}

void Ai::overridePlayFromProto(TbotsProto::Play play_proto)
{
    overridePlay(std::move(createPlay(play_proto, strategy)));
}

void Ai::updateOverridePlay()
{
    auto current_override =
        strategy->getAiConfig().ai_control_config().override_ai_play();
    if (current_override != TbotsProto::PlayName::UseAiSelection)
    {
        // Override to new play if we're not running Ai Selection
        TbotsProto::Play play_proto;
        play_proto.set_name(current_override);
        overridePlayFromProto(play_proto);
    }
    else
    {
        // Clear play override if we're running Ai Selection
        overridePlay(nullptr);
    }
}

void Ai::updateAiConfig(TbotsProto::AiConfig& ai_config)
{
    strategy->updateAiConfig(ai_config);
    updateOverridePlay();
}

std::unique_ptr<TbotsProto::PrimitiveSet> Ai::getPrimitives(const World& world)
{
    strategy->updateWorld(world);

    fsm->process_event(PlaySelectionFSM::Update(
        [this](std::shared_ptr<Play> play) { current_play = play; }, world.gameState(),
        strategy->getAiConfig()));

    if (static_cast<bool>(override_play))
    {
        return override_play->get(world, inter_play_communication,
                                  [this](InterPlayCommunication comm) {
                                      inter_play_communication = std::move(comm);
                                  });
    }
    else
    {
        return current_play->get(world, inter_play_communication,
                                 [this](InterPlayCommunication comm) {
                                     inter_play_communication = std::move(comm);
                                 });
    }
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
