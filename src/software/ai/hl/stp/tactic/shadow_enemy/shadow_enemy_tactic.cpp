#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

ShadowEnemyTactic::ShadowEnemyTactic()
    : Tactic({RobotCapability::Move, RobotCapability::Kick}),
      fsm(),fsm_map(),
      control_params{ShadowEnemyFSM::ControlParams{.enemy_threat    = std::nullopt,
                                                   .shadow_distance = 0}}
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<ShadowEnemyFSM>>();
    }
}

void ShadowEnemyTactic::updateControlParams(std::optional<EnemyThreat> enemy_threat,
                                            double shadow_distance)
{
    control_params.enemy_threat    = enemy_threat;
    control_params.shadow_distance = shadow_distance;
}

double ShadowEnemyTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    if (!control_params.enemy_threat)
    {
        return 0;
    }
    // If shadowee does not have the ball we want robots that are closer to the block
    // pass point and if the shadowee does have the ball, we want robots that are
    // closer to the block shot point. We normalize with the total field length so
    // that robots that are within the field have a cost less than 1
    Point block_point = ShadowEnemyFSM::findBlockPassPoint(
        world.ball().position(), control_params.enemy_threat->robot,
        control_params.shadow_distance);
    if (control_params.enemy_threat->has_ball)
    {
        block_point = ShadowEnemyFSM::findBlockShotPoint(
            robot, world.field(), world.friendlyTeam(), world.enemyTeam(),
            control_params.enemy_threat->robot, control_params.shadow_distance);
    }
    double cost =
        (robot.position() - block_point).length() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void ShadowEnemyTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(ShadowEnemyFSM::Update(control_params, tactic_update));
}

void ShadowEnemyTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void ShadowEnemyTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<ShadowEnemyFSM>>();
    }
    fsm.process_event(ShadowEnemyFSM::Update(control_params, tactic_update));
}
