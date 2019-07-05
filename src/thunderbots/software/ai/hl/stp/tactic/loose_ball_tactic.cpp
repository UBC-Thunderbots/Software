#include "ai/hl/stp/tactic/loose_ball_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/action/pivot_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/evaluation/intercept.h"
#include "util/parameter/dynamic_parameters.h"
#include "geom/rectangle.h"
#include "util/logger/init.h"

LooseBallTactic::LooseBallTactic(const Field &field, const Team &friendly_team,
        const Team &enemy_team, const Ball &ball,
        Angle min_net_open_angle,
        std::optional<Point> chip_target, bool loop_forever)
    : field(field),
    friendly_team(friendly_team),
    enemy_team(enemy_team),
    ball(ball),
    min_net_open_angle(min_net_open_angle),
    chip_target(chip_target),
    has_shot_available(false),
    Tactic(loop_forever, {RobotCapabilityFlags::Kick})
{
}

std::string LooseBallTactic::getName() const
{
    return "Loose Ball Tactic";
}

void LooseBallTactic::updateParams(const Field &field, const Team &friendly_team,
        const Team &enemy_team, const Ball &ball,
        std::optional<Point> chip_target)
{
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
    this->ball          = ball;
    this->chip_target   = chip_target;
}

double LooseBallTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    double cost = 0;
    cost = (world.ball().position() - robot.position()).len() /
        world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

void LooseBallTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    KickAction kick_action = KickAction();
    PivotAction pivot_action = PivotAction();

    do{
        yield(pivot_action.updateStateAndGetNextIntent(*robot,
                    ball.position(), 
                    ((*robot).position()-field.friendlyGoal()).orientation(), 
                    AngularVelocity::ofDegrees(Util::DynamicParameters::PivotAction::arb_scaling.value()), true));
    } while(!pivot_action.done());

    do
    {
        LOG(DEBUG) << "Entering calc best shot and kick";
        auto shot_target = Evaluation::calcBestShotOnFriendlyGoal(field, friendly_team, enemy_team, ball.position());
        if(shot_target){
            yield(kick_action.updateStateAndGetNextIntent(
                    *robot, ball, ball.position(), shot_target->first,
                    BALL_MAX_SPEED_METERS_PER_SECOND - 0.5));
        }
        else{
            break;
        }
    } while (true);
}
