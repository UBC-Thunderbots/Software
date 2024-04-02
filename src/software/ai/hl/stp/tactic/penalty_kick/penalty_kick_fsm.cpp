#include "software/ai/hl/stp/tactic/penalty_kick/penalty_kick_fsm.h"

PenaltyKickFSM::PenaltyKickFSM() : complete_approach(std::nullopt), shot_angle() {}

bool PenaltyKickFSM::evaluatePenaltyShot(std::optional<Robot> enemy_goalie, Field field,
                                         Point ball_position, Robot robot)
{
    double min_shot_x_position =
        ((field.totalXLength() / 2) -
         (field.totalXLength() * PENALTY_KICK_MIN_SHOT_X_DISTANCE_FACTOR));

    // don't try to shoot if we're far from the net or we're in the middle of an
    // autokick
    if (robot.position().x() < min_shot_x_position)
    {
        return false;
    }

    // If there is no goalie, the net is wide open
    if (!enemy_goalie.has_value())
    {
        return true;
    }

    /**
            B
           +--\----------------------------------+ goal line
              \ 		      +-------+
               \ 		    C |       |	enemy_goalie
                \ 		   ---+-------+
                 \	       ---/
                  \        ---/
                   \   ---/
                  A --/
                     \
                      \
                       \
                        \ D
                    +-------+
                    |       | bot
                    +-------+
        Segment BD represents the ball's path to the goal
        Segment AC is the smallest path required by the enemy to intercept BD

        This function returns true when the enemy goalie doesn't have enough time to
       block the shot at A before the ball moves past that point.
    */

    // point B: ball intersection point on goal line
    Point shot_intersection = evaluateNextShotPosition(enemy_goalie, field);

    // segment BD: ball's path to goal
    Segment ball_to_goal = Segment(ball_position, shot_intersection);

    // point A: closest point for goalie to intercept ball's path
    const Point block_position =
        closestPoint(enemy_goalie.value().position(), ball_to_goal);

    // Line AC: line from goalie to ball trajectory
    const Vector goalie_to_block_position =
        (block_position - enemy_goalie.value().position());

    // Segment AD: ball's path to potential goalie block point
    const Segment ball_to_block = Segment(ball_position, block_position);

    const double time_to_pass_keeper =
        fabs(ball_to_block.length() / PENALTY_KICK_SHOT_SPEED) + SSL_VISION_DELAY;

    // Based on constant acceleration -> // dX = init_vel*t + 0.5*a*t^2
    const double max_enemy_movement_x =
        enemy_goalie.value().velocity().x() * time_to_pass_keeper +
        0.5 * std::copysign(1, goalie_to_block_position.x()) *
            PENALTY_KICK_GOALIE_MAX_ACC * pow(time_to_pass_keeper, 2);
    const double max_enemy_movement_y =
        enemy_goalie.value().velocity().y() * time_to_pass_keeper +
        0.5 * std::copysign(1, goalie_to_block_position.y()) *
            PENALTY_KICK_GOALIE_MAX_ACC * pow(time_to_pass_keeper, 2);

    // If the maximum distance that the goalie can move is less than actual
    // distance it must move to reach the ball, return true for a viable
    // shot
    // Not simplifying this if statement makes the code logic slightly
    // easier to understand
    if ((fabs(goalie_to_block_position.x()) >
         (fabs(max_enemy_movement_x) + ROBOT_MAX_RADIUS_METERS)) ||
        (fabs(goalie_to_block_position.y()) >
         (fabs(max_enemy_movement_y) + ROBOT_MAX_RADIUS_METERS)))
    {
        return true;
    }
    else
    {
        return false;
    }
}

const Point PenaltyKickFSM::evaluateNextShotPosition(std::optional<Robot> enemy_goalie,
                                                     Field field)
{
    // Evaluate if the goalie is closer to the negative or positive goalpost
    if (enemy_goalie.has_value())
    {
        double goalie_dist_to_neg_goalpost =
            (field.enemyGoalpostNeg() - enemy_goalie.value().position()).lengthSquared();
        double goalie_dist_to_pos_goalpost =
            (field.enemyGoalpostPos() - enemy_goalie.value().position()).lengthSquared();

        return goalie_dist_to_neg_goalpost > goalie_dist_to_pos_goalpost
                   ? field.enemyGoalpostNeg() + Vector(0, PENALTY_KICK_POST_OFFSET)
                   : field.enemyGoalpostPos() + Vector(0, -PENALTY_KICK_POST_OFFSET);
    }
    else
    {
        // Return the center of the enemy goal
        return Point(field.enemyGoalpostPos().x(), 0);
    }
}

void PenaltyKickFSM::shoot(const Update &event,
                           boost::sml::back::process<KickSkillFSM::Update> processEvent)
{
    KickSkillFSM::ControlParams control_params{
        .kick_origin                  = event.common.world_ptr->ball().position(),
        .kick_direction               = shot_angle,
        .kick_speed_meters_per_second = PENALTY_KICK_SHOT_SPEED};
    processEvent(KickSkillFSM::Update(
        control_params, SkillUpdate(event.common.robot, event.common.world_ptr, strategy,
                                    event.common.set_primitive)));
}

void PenaltyKickFSM::updateApproachKeeper(
    const Update &event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    Field field                       = event.common.world_ptr->field();
    std::optional<Robot> enemy_goalie = event.common.world_ptr->enemyTeam().goalie();
    const Point next_shot_position =
        evaluateNextShotPosition(enemy_goalie, event.common.world_ptr->field());
    shot_angle =
        (next_shot_position - event.common.world_ptr->ball().position()).orientation();
    Point position = field.enemyGoalCenter() + Vector(-field.defenseAreaXLength(), 0);

    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination       = std::optional<Point>(position),
        .final_dribble_orientation = std::optional<Angle>(Angle::zero()),
        .allow_excessive_dribbling = false};
    processEvent(DribbleSkillFSM::Update(
        control_params, SkillUpdate(event.common.robot, event.common.world_ptr, strategy,
                                    event.common.set_primitive)));
}

void PenaltyKickFSM::adjustOrientationForShot(
    const Update &event, boost::sml::back::process<DribbleSkillFSM::Update> processEvent)
{
    std::optional<Robot> enemy_goalie = event.common.world_ptr->enemyTeam().goalie();
    const Point next_shot_position =
        evaluateNextShotPosition(enemy_goalie, event.common.world_ptr->field());
    Point final_position = event.common.world_ptr->ball().position();
    shot_angle           = (next_shot_position - final_position).orientation();
    DribbleSkillFSM::ControlParams control_params{
        .dribble_destination       = std::optional<Point>(final_position),
        .final_dribble_orientation = std::optional<Angle>(shot_angle),
        .allow_excessive_dribbling = false};
    processEvent(DribbleSkillFSM::Update(
        control_params, SkillUpdate(event.common.robot, event.common.world_ptr, strategy,
                                    event.common.set_primitive)));
}

bool PenaltyKickFSM::takePenaltyShot(const Update &event)
{
    Field field = event.common.world_ptr->field();

    // if the complete approach timestamp hasn't been set (because this is the
    // first run), set the initial
    //  values
    if (!complete_approach.has_value())
    {
        Timestamp future_approach_complete_time =
            event.common.world_ptr->getMostRecentTimestamp() +
            PENALTY_FINISH_APPROACH_TIMEOUT;
        complete_approach = std::optional<Timestamp>(future_approach_complete_time);
    }
    std::optional<Robot> enemy_goalie = event.common.world_ptr->enemyTeam().goalie();
    Timestamp force_shoot_timestamp =
        complete_approach.value() + PENALTY_FORCE_SHOOT_TIMEOUT;
    bool should_shoot =
        evaluatePenaltyShot(enemy_goalie, field,
                            event.common.world_ptr->ball().position(),
                            event.common.robot) ||
        (event.common.world_ptr->getMostRecentTimestamp() >= force_shoot_timestamp);
    return should_shoot;
}

bool PenaltyKickFSM::timeOutApproach(const Update &event)
{
    return event.common.world_ptr->getMostRecentTimestamp() > complete_approach;
}
