#include "software/ai/hl/stp/skill/dribble_skill.h"
#include "software/util/generic_factory/generic_factory.h"

DribbleSkill::DribbleSkill(const TbotsProto::AiConfig& ai_config, double inital_score)
    : Skill(ai_config, inital_score),
      fsm(DribbleFSM(ai_config.dribble_tactic_config())),
      control_params{DribbleFSM::ControlParams{.dribble_destination = std::nullopt,
          .final_dribble_orientation = std::nullopt,
          .allow_excessive_dribbling = false}}
{
}

void DribbleSkill::updatePrimitive(const Robot& robot, const World& world, const TacticUpdate& tactic_update,
        std::shared_ptr<Strategy> strategy)
{
    Pose dribble_pose = strategy->getBestDribblePose(robot);

    control_params.dribble_destination = dribble_pose.point();
    control_params.final_dribble_orientation = dribble_pose.orientation();
    control_params.allow_excessive_dribbling = false;

    fsm.process_event(DribbleFSM::Update(control_params, tactic_update));   
}

static TGenericFactory<std::string, Skill, DribbleSkill, TbotsProto::AiConfig, double> factory;
