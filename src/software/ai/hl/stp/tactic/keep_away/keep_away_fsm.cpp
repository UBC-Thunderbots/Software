#include "software/ai/hl/stp/tactic/keep_away/keep_away_fsm.h"

void KeepAwayFSM::keepAway(const Update& event,
                           boost::sml::back::process<DribbleFSM::Update> processEvent)
{
    // ball possession is threatened, get into a better position to take the
    // best pass so far
    DribbleFSM::ControlParams control_params;

    auto best_pass_so_far = Pass(event.common.robot.position(),
                                 event.common.world_ptr->field().enemyGoalCenter(),
                                 BALL_MAX_SPEED_METERS_PER_SECOND);

    if (event.control_params.best_pass_so_far)
    {
        best_pass_so_far = *event.control_params.best_pass_so_far;
    }
    else
    {
        // we didn't get a best_pass_so_far, so we will be using the default pass.
        LOG(INFO) << "KeepAwayFSM has no best pass so far, using default pass "
                  << "to enemy goal center.";
    }

    auto keepaway_dribble_dest = findKeepAwayTargetPoint(
        *event.common.world_ptr, best_pass_so_far, ai_config.passing_config());

    const auto& enemy_team = event.common.world_ptr->enemyTeam();
    const auto& ball       = event.common.world_ptr->ball();

    auto final_dribble_orientation = best_pass_so_far.passerOrientation();

    // there is a robot on the enemy team close to us, face away from the nearest one
    auto nearest_enemy_robot = enemy_team.getNearestRobot(event.common.robot.position());
    if (nearest_enemy_robot.has_value() &&
        distance(ball.position(), nearest_enemy_robot->position()) <
            ai_config.attacker_tactic_config().enemy_about_to_steal_ball_radius())
    {
        auto dribble_orientation_vec = ball.position() - nearest_enemy_robot->position();
        final_dribble_orientation    = dribble_orientation_vec.orientation();
    }

    control_params = {.dribble_destination       = keepaway_dribble_dest,
                      .final_dribble_orientation = final_dribble_orientation,
                      .allow_excessive_dribbling = false};

    processEvent(DribbleFSM::Update(control_params, event.common));
}
