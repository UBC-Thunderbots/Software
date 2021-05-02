#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency


ShadowEnemyTactic::ShadowEnemyTactic()
    : Tactic(false, {RobotCapability::Move, RobotCapability::Kick}),
      fsm(),
      control_params{ShadowEnemyFSM::ControlParams{.enemy_threat    = std::nullopt,
                                                   .shadow_distance = 0}}
{
}

void ShadowEnemyTactic::updateWorldParams(const World &world) {}

void ShadowEnemyTactic::updateControlParams(std::optional<EnemyThreat> enemy_threat,
                                            double shadow_distance)
{
    control_params.enemy_threat            = enemy_threat;
    control_params.shadow_distance         = shadow_distance;
}

double ShadowEnemyTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    if (!control_params.enemy_threat)
    {
        return 0;
    }
    // Prefer robots closer to the enemy being shadowed
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - control_params.enemy_threat->robot.position()).length() /
        world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void ShadowEnemyTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

bool ShadowEnemyTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void ShadowEnemyTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(ShadowEnemyFSM::Update(control_params, tactic_update));
}

void ShadowEnemyTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
