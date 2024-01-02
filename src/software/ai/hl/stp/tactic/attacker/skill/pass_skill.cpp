#include "software/ai/hl/stp/tactic/attacker/skill/pass_skill.h"
#include "software/util/generic_factory/generic_factory.h"

PassSkill::PassSkill(const TbotsProto::AiConfig& ai_config, double initial_score)
    : Skill(ai_config, initial_score),
      fsm(AttackerFSM(ai_config.attacker_tactic_config())),
      control_params({.best_pass_so_far = std::nullopt,
              .pass_committed = false,
              .shot = std::nullopt,
              .chip_target = std::nullopt})
{
}

double PassSkill::calculateViability(const Robot& robot, const World& world, std::shared_ptr<Strategy> strategy)
{
    // fix
}

bool PassSkill::done() const
{
    return fsm.is(boost::sml::X);
}

void PassSkill::updatePrimitive(const Robot& robot, const World& world, const TacticUpdate& tactic_update,
        std::shared_ptr<Strategy> strategy)
{
   Pass best_pass = strategy->getBestPass(robot); 
   control_params.best_pass_so_far = best_pass;
   control_params.pass_committed = true;

   fsm.process_event(AttackerFSM::Update(control_params, tactic_update));
}

static TGenericFactory<std::string, Skill, PassSkill, TbotsProto::AiConfig, double>
