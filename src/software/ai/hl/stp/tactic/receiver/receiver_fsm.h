#pragma once

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/intent.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/closest_point.h"

struct ReceiverFSM
{
    struct ControlParams
    {
        // The pass to receive
        std::optional<Pass> pass = std::nullopt;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // The minimum proportion of open net we're shooting on vs the entire size of the net
    // that we require before attempting a shot
    static constexpr double MIN_SHOT_NET_PERCENT_OPEN    = 0.3;
    static constexpr double MAX_SPEED_FOR_ONE_TOUCH_SHOT = 6.5;

    // The maximum deflection angle that we will attempt a one-touch kick towards the
    // enemy goal with
    static constexpr Angle MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT = Angle::fromDegrees(90);


    /**
     * TODO
     *
     * @param shot
     * @param ball
     */
    static Angle getOneTimeShotDirection(const Ray& shot, const Ball& ball)
    {
        Vector shot_vector = shot.toUnitVector();
        Angle shot_dir     = shot.getDirection();

        Vector ball_vel    = ball.velocity();
        Vector lateral_vel = ball_vel.project(shot_vector.perpendicular());

        // The lateral speed is roughly a measure of the lateral velocity we need to
        // "cancel out" in order for our shot to go in the expected direction.
        // The scaling factor of 0.3 is a magic number that was carried over from the old
        // code. It seems to work well on the field.
        double lateral_speed = 0.3 * lateral_vel.length();

        // This kick speed is based off of the value used in the firmware `MovePrimitive`
        // when autokick is enabled
        double kick_speed = BALL_MAX_SPEED_METERS_PER_SECOND - 1;
        Angle shot_offset = Angle::asin(lateral_speed / kick_speed);

        // check which direction the ball is going in so we can decide which direction to
        // apply the offset in
        if (lateral_vel.dot(shot_vector.rotate(Angle::quarter())) > 0)
        {
            // need to go clockwise
            shot_offset = -shot_offset;
        }
        return shot_dir + shot_offset;
    }

    /**
     *
     * @param robot
     * @param ball
     * @param best_shot_target
     */
    static Shot getOneTimeShotPositionAndOrientation(const Robot& robot, const Ball& ball,
                                                     const Point& best_shot_target)
    {
        double dist_to_ball_in_dribbler =
            DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS;
        Point ball_contact_point =
            robot.position() + Vector::createFromAngle(robot.orientation())
                                   .normalize(dist_to_ball_in_dribbler);

        // Find the closest point to the ball contact point on the ball's trajectory
        Point closest_ball_pos = ball.position();
        if (ball.velocity().length() != 0)
        {
            closest_ball_pos =
                closestPoint(ball_contact_point,
                             Line(ball.position(), ball.position() + ball.velocity()));
        }
        Ray shot(closest_ball_pos, best_shot_target - closest_ball_pos);

        Angle ideal_orientation      = getOneTimeShotDirection(shot, ball);
        Vector ideal_orientation_vec = Vector::createFromAngle(ideal_orientation);

        // The best position is determined such that the robot stays in the ideal
        // orientation, but moves the shortest distance possible to put its contact
        // point in the ball's path.
        Point ideal_position =
            closest_ball_pos - ideal_orientation_vec.normalize(dist_to_ball_in_dribbler);

        return Shot(ideal_position, ideal_orientation);
    }

    static std::optional<Shot> findFeasibleShot(const World& world,
                                                const Robot& assigned_robot)
    {
        // Check if we can shoot on the enemy goal from the receiver position
        std::optional<Shot> best_shot_opt = calcBestShotOnGoal(
            world.field(), world.friendlyTeam(), world.enemyTeam(),
            assigned_robot.position(), TeamType::ENEMY, {assigned_robot});

        // Vector from the ball to the robot
        Vector robot_to_ball = world.ball().position() - assigned_robot.position();

        // The angle the robot will have to deflect the ball to shoot
        Angle abs_angle_between_pass_and_shot_vectors;
        // The percentage of open net the robot would shoot on
        double net_percent_open;
        if (best_shot_opt)
        {
            Vector robot_to_shot_target =
                best_shot_opt->getPointToShootAt() - assigned_robot.position();
            abs_angle_between_pass_and_shot_vectors =
                (robot_to_ball.orientation() - robot_to_shot_target.orientation())
                    .clamp()
                    .abs();

            Angle goal_angle =
                acuteAngle(world.field().friendlyGoalpostPos(), assigned_robot.position(),
                           world.field().friendlyGoalpostNeg())
                    .abs();
            net_percent_open =
                best_shot_opt->getOpenAngle().toDegrees() / goal_angle.toDegrees();
        }

        // If we have a shot with a sufficiently large enough opening, and the
        // deflection angle that is reasonable, we should one-touch kick the ball
        // towards the enemy net
        if (best_shot_opt && net_percent_open > MIN_SHOT_NET_PERCENT_OPEN &&
            abs_angle_between_pass_and_shot_vectors < MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT)
        {
            return best_shot_opt;
        }
        return std::nullopt;
    }


    auto operator()()
    {
        using namespace boost::sml;

        const auto move_s     = state<MoveFSM>;
        const auto receive_s  = state<MoveFSM>;
        const auto onetouch_s = state<MoveFSM>;
        const auto update_e   = event<Update>;

        /**
         * Action that updates the MoveFSM
         *
         * @param event MoveFSM::Update event
         */
        const auto update_onetouch = [](auto event,
                                        back::process<MoveFSM::Update> processEvent) {
            MoveFSM::ControlParams control_params;
            std::cerr << "update_onetouch" << std::endl;

            auto best_shot = findFeasibleShot(event.common.world, event.common.robot);

            if (best_shot)
            {
                Shot shot = getOneTimeShotPositionAndOrientation(
                    event.common.robot, event.common.world.ball(),
                    best_shot->getPointToShootAt());

                control_params = MoveFSM::ControlParams{
                    .destination            = event.control_params.pass->receiverPoint(),
                    .final_orientation      = shot.getOpenAngle(),
                    .final_speed            = 0.0,
                    .dribbler_mode          = DribblerMode::OFF,
                    .ball_collision_type    = BallCollisionType::ALLOW,
                    .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                        MAX_SPEED_FOR_ONE_TOUCH_SHOT},
                    .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                    .target_spin_rev_per_s  = 0.0,
                };
            }
            if (event.control_params.pass)
            {
                // Update the dribble fsm
                processEvent(MoveFSM::Update(control_params, event.common));
            }
        };

        const auto update_receive = [](auto event,
                                       back::process<MoveFSM::Update> processEvent) {
            MoveFSM::ControlParams control_params;
            std::cerr << "update_move_to_receive" << std::endl;

            if (event.control_params.pass)
            {
                control_params = {
                    .destination       = event.control_params.pass->receiverPoint(),
                    .final_orientation = event.control_params.pass->receiverOrientation(),
                    .final_speed       = 0.0,
                    .dribbler_mode     = DribblerMode::OFF,
                    .ball_collision_type    = BallCollisionType::ALLOW,
                    .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                                        MAX_SPEED_FOR_ONE_TOUCH_SHOT},
                    .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                    .target_spin_rev_per_s  = 0.0,
                };
            }

            // Update the dribble fsm
            processEvent(MoveFSM::Update(control_params, event.common));
        };

        /**
         * @param ReceiverFSM::Update
         * @return true if the ball has started moving
         */
        const auto pass_started = [](auto event) {
            return event.common.world.ball().velocity().length() > 0.5;
        };

        const auto pass_finished = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        /**
         * @param ReceiverFSM::Update
         * @return true if a one touch shot is possible on net
         */
        const auto onetouch_possible = [](auto event) { return false; };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *move_s + update_e[!pass_started && onetouch_possible] / update_onetouch =
                onetouch_s,
            move_s + update_e[!pass_started && !onetouch_possible] / update_receive =
                receive_s,
            receive_s + update_e[pass_started && !pass_finished] / update_receive =
                receive_s,
            onetouch_s + update_e[pass_started && !pass_finished] / update_onetouch =
                onetouch_s,
            receive_s + update_e[pass_finished] / update_receive   = X,
            onetouch_s + update_e[pass_finished] / update_onetouch = X);
    }
};
