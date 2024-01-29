#include "software/ai/hl/stp/tactic/attacker/skill/dribble_skill.h"

DribbleSkill::DribbleSkill(const TbotsProto::AiConfig& ai_config,
                           std::shared_ptr<Strategy> strategy, double inital_score)
    : Skill(ai_config, strategy, inital_score),
      fsm(DribbleFSM(ai_config.dribble_tactic_config())),
      control_params{DribbleFSM::ControlParams{.dribble_destination       = std::nullopt,
                                               .final_dribble_orientation = std::nullopt,
                                               .allow_excessive_dribbling = false}}
{
}

bool DribbleSkill::done() const
{
    return fsm.is(boost::sml::X);
}

void DribbleSkill::updatePrimitive(const TacticUpdate& tactic_update)
{
    Pose dribble_pose = (*strategy_)->getBestDribblePose(tactic_update.robot);

    control_params.dribble_destination       = dribble_pose.point();
    control_params.final_dribble_orientation = dribble_pose.orientation();
    control_params.allow_excessive_dribbling = false;

    fsm.process_event(DribbleFSM::Update(control_params, tactic_update));
}

static TGenericFactory<std::string, Skill, DribbleSkill, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>, double>
    factory;
