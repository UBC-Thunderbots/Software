/**
 * Implementation of the PenaltyKickTactic
 */
#include "penalty_kick_tactic.h"

#include "ai/hl/stp/action/dribble_action.h"
#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "geom/circle.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"


PenaltyKickTactic::PenaltyKickTactic(const Ball& ball, const Field& field,
                                     const std::optional<Robot>& enemy_goalie,
                                     bool loop_forever)
    : ball(ball), field(field), enemy_goalie(enemy_goalie), Tactic(loop_forever)
{
}

std::string PenaltyKickTactic::getName() const
{
    return "Penalty Kick Tactic";
}

void PenaltyKickTactic::updateParams(const Ball& updated_ball,
                                     const std::optional<Robot>& updated_enemy_goalie,
                                     const Field& updated_field)
{
    this->enemy_goalie = updated_enemy_goalie;
    this->ball         = updated_ball;
    this->field        = updated_field;
}

double PenaltyKickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the pass start position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - world.ball().position()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

// Evaluation function designed specifically for 1-on-1 penalty shots
bool PenaltyKickTactic::evaluate_penalty_shot() {

    Segment enemy_goal_line = Segment(field.enemyGoalpostNeg(), field.enemyGoalpostPos());
    Ray robot_shot_ray = Ray( ball.position(), (ball.position() - robot->position()).norm() );
    Point shot_origin = ball.position();

    auto [goal_intersection, extra_intersect] = raySegmentIntersection(robot_shot_ray, enemy_goal_line);

    // If there is no enemy goalie, just shoot and score
    if(!enemy_goalie.has_value()) {
        return true;
    }

    if(goal_intersection.has_value()){
        // Check if we have a good shot

        const Point enemy_goalie_distance_from_goal = (goal_intersection.value() - enemy_goalie->position());

        const double time_to_score =  (goal_intersection.value() - ball.position()).len() / PENALTY_KICK_SHOT_SPEED;

        const double vision_adjusted_time = time_to_score - SSL_VISION_DELAY;

        const Circle enemy_goalie_area = Circle(enemy_goalie->position(), ROBOT_MAX_RADIUS_METERS*1.1);

        // If we are aiming at the goalie don't shoot
        if(contains(enemy_goalie_area, goal_intersection.value())) {
            return false;
        }
        // We have seconds, we know current velocity and acc.
        double enemy_goalie_acc = enemy_goalie_distance_from_goal.y() >= 0 ? PENALTY_KICK_GOALIE_MAX_ACC : -1*PENALTY_KICK_GOALIE_MAX_ACC;

        // The only speed that matters is Y direction because the goalie is bound to the crease during penalty kicks
        //d = v0*t + 0.5*a*t^2
        const double max_distance_goalie_can_travel = enemy_goalie->velocity().y()*vision_adjusted_time+ 0.5*enemy_goalie_acc*(pow(vision_adjusted_time, 2));

        // If the enemy goalie can't travel far enough to block our shot, shoot
        if( fabs(max_distance_goalie_can_travel) < fabs(enemy_goalie_distance_from_goal.len())) {
            return true;
        }
    }
    else {
        // If we cannot score in the current position, don't shoot
        return false;
    }

    return false;
}

Point PenaltyKickTactic::evaluate_next_position() {

    // If there is no enemy goalie, shoot at the center
    if(!enemy_goalie.has_value()) {
        return field.enemyGoal();
    }


    const Point close_to_neg_post = field.enemyGoalpostNeg() + Point(0,0.1);
    const Point close_to_pos_post = field.enemyGoalpostPos() - Point(0,0.1);

    const double goalie_dist_to_neg_shot = (close_to_neg_post - enemy_goalie->position()).len();
    const double goalie_dist_to_pos_shot = (close_to_pos_post - enemy_goalie->position()).len();

    return fabs(goalie_dist_to_neg_shot) > fabs(goalie_dist_to_pos_shot) ? close_to_neg_post : close_to_pos_post;
}

void PenaltyKickTactic::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // Keep track if a shot has been taken
    bool shot_taken             = false;

    MoveAction get_behind_ball_act = MoveAction();
    KickAction kick_ball_act = KickAction();
    Timestamp start_of_shot = robot->getMostRecentTimestamp();

    do {

        Point new_target = evaluate_next_position();

        bool YEET_SHOOTING = false;

        Vector behind_ball_direction = (ball.position() - new_target).norm();

        Point behind_ball = ball.position() + behind_ball_direction.norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS + 0.04);

        Angle shot_orientation = behind_ball_direction.orientation() + Angle::half();

        if(!get_behind_ball_act.done() && !evaluate_penalty_shot() && !YEET_SHOOTING) {
            printf("\nGetting to position");
            yield(get_behind_ball_act.updateStateAndGetNextIntent(*robot, behind_ball, robot->orientation(), 0, true, false, AutokickType::NONE));
            printf("evaluate shot %d", evaluate_penalty_shot());
        }
        else if(evaluate_penalty_shot()) {
            printf("\nYEET");
            printf("Are we timing out?%d",(robot->getMostRecentTimestamp() - start_of_shot) >= Duration::fromSeconds(7) );

            YEET_SHOOTING = true;

            yield(kick_ball_act.updateStateAndGetNextIntent(*robot, ball, ball.position(), shot_orientation, PENALTY_KICK_SHOT_SPEED ));
        }

    } while(true);
}
