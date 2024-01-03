#include "software/ai/hl/stp/tactic/attacker/skill/shoot_skill.h"

ShootSkill::ShootSkill(const TbotsProto::AiConfig& ai_config,
                       std::shared_ptr<Strategy> strategy, double initial_score)
    : Skill(ai_config, strategy, initial_score),
      fsm(AttackerFSM(ai_config.attacker_tactic_config())),
      control_params({.best_pass_so_far = std::nullopt,
                      .pass_committed   = false,
                      .shot             = std::nullopt,
                      .chip_target      = std::nullopt})
{
}

double ShootSkill::calculateViability(const Robot& robot, const World& world)
{
    if (!strategy_->getBestShot(robot, w orld))
    {
        return 0;
    }

    return score;
}

bool ShootSkill::done() const
{
    return fsm.is(boost::sml::X);
}

void ShootSkill::updatePrimitive(const TacticUpdate& tactic_update)
{
    control_params.shot =
        strategy_->getBestShot(tactic_update.robot, tactic_update.world);

    fsm.process_event(AttackerFSM::Update(control_params, tactic_update));
}

static TGenericFactory<std::string, Skill, ShootSkill, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>, double>
    factory;
