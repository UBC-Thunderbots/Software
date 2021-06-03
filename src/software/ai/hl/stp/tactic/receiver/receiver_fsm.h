#pragma once

#include "shared/constants.h"
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
    class OneTouchShotState;
    class ReceiveAndDribbleState;
    class UndecidedState;

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
    static constexpr double MIN_PASS_START_SPEED         = 1.0;

    // The maximum deflection angle that we will attempt a one-touch kick towards the
    // enemy goal with
    static constexpr Angle MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT = Angle::fromDegrees(90);

    /**
     * Given a shot and the ball, figures out the angle the robot should be facing
     * to perform a one-touch shot.
     *
     * @param shot The shot to take
     * @param ball The ball on the field
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
     * Figures out the location of the one-time shot and orientation the robot should face
     *
     * @param robot The robot performing the one-touch
     * @param ball The ball on the field
     * @param best_shot_target The point to shoot at
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

    /*
     * Finds a shot that is greater than MIN_SHOT_NET_PERCENT_OPEN and
     * respects MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT for the highest chance
     * of scoring with a one-touch shot. If neither of those are true, return a nullopt
     *
     * @param world The world to find a feasible shot on
     * @param assigned_robot The robot that will be performing the one-touch
     */
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

        const auto undecided_s = state<UndecidedState>;
        const auto receive_s   = state<ReceiveAndDribbleState>;
        const auto onetouch_s  = state<OneTouchShotState>;
        const auto update_e    = event<Update>;

        /**
         * Checks if a one touch shot is possible
         *
         * @param event ReceiverFSM::Update event
         * @return true if one-touch possible
         */
        const auto onetouch_possible = [](auto event) {
            return findFeasibleShot(event.common.world, event.common.robot) !=
                   std::nullopt;
        };

        /**
         * If we have a shot on net, then update the receiver fsm
         * to setup for a one-touch shot.
         *
         * NOTE: This must be used with the onetouch_possible guard,
         * which checks for one-touch feasibility.
         *
         * @param event ReceiverFSM::Update event
         */
        const auto update_onetouch = [this](auto event) {
            auto best_shot = findFeasibleShot(event.common.world, event.common.robot);
            auto one_touch = getOneTimeShotPositionAndOrientation(
                event.common.robot, event.common.world.ball(),
                best_shot->getPointToShootAt());

            if (best_shot)
            {
                if (event.control_params.pass)
                {
                    event.common.set_intent(std::make_unique<MoveIntent>(
                        event.common.robot.id(), one_touch.getPointToShootAt(),
                        one_touch.getOpenAngle(), 0, DribblerMode::OFF,
                        BallCollisionType::ALLOW,
                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                       MAX_SPEED_FOR_ONE_TOUCH_SHOT},
                        MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
                }
            }
        };

        /**
         * One touch shot is not possible, just receive ball as cleanly as possible.
         *
         * @param event ReceiverFSM::Update event
         */
        const auto update_receive = [this](auto event) {
            if (event.control_params.pass)
            {
                event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(), event.control_params.pass->receiverPoint(),
                    event.control_params.pass->receiverOrientation(), 0,
                    DribblerMode::OFF, BallCollisionType::ALLOW,
                    AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
            }
        };

        const auto adjust_receive = [this](auto event) {
            auto ball      = event.common.world.ball();
            auto robot_pos = event.common.robot.position();

            if ((ball.position() - robot_pos).length() >
                DIST_TO_FRONT_OF_ROBOT_METERS + 2 * BALL_MAX_RADIUS_METERS)
            {
                Point ball_receive_pos = ball.position();

                if (ball.velocity().length() != 0)
                {
                    ball_receive_pos = closestPoint(
                        robot_pos,
                        Line(ball.position(), ball.position() + ball.velocity()));
                }

                Angle ball_receive_orientation =
                    (ball.position() - robot_pos).orientation();

                event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(), ball_receive_pos, ball_receive_orientation,
                    0, DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
                    AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
            }
        };

        /**
         * Check if the pass has started by checking if the ball is moving faster
         * than a minimum speed.
         *
         * @param event ReceiverFSM::Update event
         * @return true if the pass has started
         */
        const auto pass_started = [](auto event) {
            return event.common.world.ball().velocity().length() > MIN_PASS_START_SPEED;
        };

        /**
         * Check if the pass has finished by checking if we the robot has
         * a ball near its dribbler.
         *
         * @param event ReceiverFSM::Update event
         * @return true if the ball is near a robots mouth
         */
        const auto pass_finished = [](auto event) {
            // We tolerate imperfect passes that hit the edges of the robot,
            // so that we can quickly transition out and grab the ball.
            return event.common.robot.isNearDribbler(event.common.world.ball().position(),
                                                     ROBOT_MAX_RADIUS_METERS);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *undecided_s + update_e[onetouch_possible] / update_onetouch = onetouch_s,
            undecided_s + update_e[!onetouch_possible] / update_receive  = receive_s,
            receive_s + update_e[!pass_started] / update_receive         = receive_s,
            onetouch_s + update_e[!pass_started] / update_onetouch       = onetouch_s,
            receive_s + update_e[pass_started && !pass_finished] / adjust_receive =
                receive_s,
            onetouch_s + update_e[pass_started && !pass_finished] / update_onetouch =
                onetouch_s,
            receive_s + update_e[pass_started && pass_finished] / update_receive   = X,
            onetouch_s + update_e[pass_started && pass_finished] / update_onetouch = X);
    }
};
