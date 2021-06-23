#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/intercept.h"
#include "software/ai/evaluation/pass.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

struct DribbleFSM
{
   public:
    class GetPossessionState;
    class DribbleState;

    /**
     * Constructor for DribbleFSM
     *
     * @param continuous_dribbling_start_point A pointer to a Point to track the
     * continuous dribbling start point
     */
    explicit DribbleFSM(const std::shared_ptr<Point> &continuous_dribbling_start_point)
        : continuous_dribbling_start_point(continuous_dribbling_start_point)
    {
    }

    struct ControlParams
    {
        // The destination for dribbling the ball
        std::optional<Point> dribble_destination;
        // The final orientation to face the ball when finishing dribbling
        std::optional<Angle> final_dribble_orientation;
        // whether to allow excessive dribbling, i.e. more than 1 metre at a time
        bool allow_excessive_dribbling;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // Threshold to determine if the ball is at the destination determined experimentally
    static constexpr double BALL_CLOSE_TO_DEST_THRESHOLD = 0.1;
    // Threshold to determine if the robot has the expected orientation when completing
    // the dribble
    static constexpr Angle FINAL_DESTINATION_CLOSE_THRESHOLD = Angle::fromDegrees(1);
    // Kick speed when breaking up continuous dribbling
    static constexpr double DRIBBLE_KICK_SPEED = 0.15;
    // Maximum distance to continuously dribble the ball, slightly conservative to not
    // break the 1 meter rule
    static constexpr double MAX_CONTINUOUS_DRIBBLING_DISTANCE = 0.9;
    // robot speed at which the robot is done dribbling
    static constexpr double ROBOT_DRIBBLING_DONE_SPEED = 0.2;  // m/s
    // if ball and front of robot are separated by this amount, then we've lost possession
    static constexpr double LOSE_BALL_POSSESSION_THRESHOLD = 0.01;
    // when we think the ball is moving slow enough that we should go directly to it
    static constexpr double BALL_MOVING_SLOW_SPEED_THRESHOLD = 0.3;
    // if we are close to ball don't intercept
    static constexpr auto INTERCEPT_BALL_RADIUS = 0.2;


    /**
     * Converts the ball position to the robot's position given the direction that the
     * robot faces the ball
     *
     * @param ball_position The ball position
     * @param face_ball_angle The angle to face the ball
     *
     * @return the point that the robot should be positioned to face the ball and dribble
     * the ball
     */
    static Point robotPositionToFaceBall(const Point &ball_position,
                                         const Angle &face_ball_angle)
    {
        return ball_position - Vector::createFromAngle(face_ball_angle)
                                   .normalize(DIST_TO_FRONT_OF_ROBOT_METERS +
                                              BALL_MAX_RADIUS_METERS - 0.005);
    }

	/**
	 * Returns true if the ball is far enough that we can register a new continuous dribble start position.
	 *
	 * @param ball_position The ball position
	 * @param robot The robot
	 *
	 * @return true if the ball is sufficiently far enough for us to consider the ball to be considered in
	 *         a new start position
	 */
	static bool isRobotFarFromBall(const Point &ball_position,
								   const Robot &robot)
	{
		double distance_robot_ball = (ball_position - robot.position()).length();
		return distance_robot_ball >= LOSE_BALL_POSSESSION_THRESHOLD;
	}

    /**
     * Gets the destination to dribble the ball to from the update event
     *
     * @param event DribbleFSM::Update
     *
     * @return the destination to dribble the ball to
     */
    static Point getDribbleBallDestination(const Point &ball_position,
                                           std::optional<Point> dribble_destination)
    {
        // Default is the current ball position
        Point target_dest = ball_position;
        if (dribble_destination)
        {
            target_dest = dribble_destination.value();
        }
        return target_dest;
    }

    /**
     * Gets the final dribble orientation from the update event
     *
     * @param event DribbleFSM::Update
     *
     * @return the final orientation to finish dribbling facing
     */
    static Angle getFinalDribbleOrientation(
        const Point &ball_position, const Point &robot_position,
        std::optional<Angle> final_dribble_orientation)
    {
        // Default is face ball direction
        Angle target_orientation = (ball_position - robot_position).orientation();
        if (final_dribble_orientation)
        {
            target_orientation = final_dribble_orientation.value();
        }
        return target_orientation;
    }

    /**
     * Calculates the next dribble destination and orientation
     *
     * @param ball The ball
     * @param robot The robot
     * @param dribble_destination_opt The dribble destination
     * @param final_dribble_orientation_opt The final dribble orientation
     *
     * @return the next dribble destination and orientation
     */
    static std::tuple<Point, Angle> calculateNextDribbleDestinationAndOrientation(
        const Ball &ball, const Robot &robot,
        std::optional<Point> dribble_destination_opt,
        std::optional<Angle> final_dribble_orientation_opt)
    {
        Point dribble_destination =
            getDribbleBallDestination(ball.position(), dribble_destination_opt);

        // Default destination and orientation assume ball is at the destination
        // pivot to final face ball destination
        Angle target_orientation = getFinalDribbleOrientation(
            ball.position(), robot.position(), final_dribble_orientation_opt);
        Point target_destination =
            robotPositionToFaceBall(dribble_destination, target_orientation);

        return std::make_tuple(target_destination, target_orientation);
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto get_possession_s = state<GetPossessionState>;
        const auto dribble_s        = state<DribbleState>;

        const auto update_e = event<Update>;

        /**
         * Guard that checks if the robot has possession of the ball
         *
         * @param event DribbleFSM::Update
         *
         * @return if the ball has been have_possession
         */
        const auto have_possession = [](auto event) {
            return event.common.robot.isNearDribbler(
                event.common.world.ball().position());
        };

        /**
         * Guard that checks if the robot has lost possession of the ball
         *
         * @param event DribbleFSM::Update
         *
         * @return if the ball possession has been lost
         */
        const auto lost_possession = [](auto event) {
            return !event.common.robot.isNearDribbler(
                // avoid cases where ball is exactly on the edge fo the robot
                event.common.world.ball().position(), LOSE_BALL_POSSESSION_THRESHOLD);
        };

        /**
         * Guard that checks if the ball is at the dribble_destination and robot is facing
         * the right direction with possession of the ball
         *
         * @param event DribbleFSM::Update
         *
         * @return if the ball is at the dribble_destination, robot is facing the correct
         * direction and ahs possession of the ball
         */
        const auto dribbling_done = [have_possession](auto event) {
            return comparePoints(event.common.world.ball().position(),
                                 getDribbleBallDestination(
                                     event.common.world.ball().position(),
                                     event.control_params.dribble_destination),
                                 BALL_CLOSE_TO_DEST_THRESHOLD) &&
                   compareAngles(event.common.robot.orientation(),
                                 getFinalDribbleOrientation(
                                     event.common.world.ball().position(),
                                     event.common.robot.position(),
                                     event.control_params.final_dribble_orientation),
                                 FINAL_DESTINATION_CLOSE_THRESHOLD) &&
                   have_possession(event) &&
                   robotStopped(event.common.robot, ROBOT_DRIBBLING_DONE_SPEED);
        };

        /**
         * Action to get possession of the ball
         *
         * If the ball is moving quickly, then move in front of the ball
         * If the ball is moving slowly, then chase the ball
         *
         * @param event DribbleFSM::Update
         */
        const auto get_possession = [this](auto event) {
            auto ball_position = event.common.world.ball().position();
			double robot_ball_distance = (ball_position - event.common.robot.position()).length();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position())
                    .orientation();
			if (isRobotFarFromBall(ball_position, event.common.robot))
			{
				continuous_dribbling_start_point = nullptr;
			}
			
			bool is_close_to_ball = robot_ball_distance <= ROBOT_MAX_RADIUS_METERS;
			if (continuous_dribbling_start_point == nullptr && is_close_to_ball)
			{
				continuous_dribbling_start_point = std::make_shared<Point>(ball_position);
			}

            auto speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT;

            auto intercept_result =
                findBestInterceptForBall(event.common.world.ball(),
                                         event.common.world.field(), event.common.robot)
                    .value_or(
                        std::make_pair(event.common.world.ball().position(), Duration()));
            auto intercept_position = intercept_result.first;

            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), intercept_position, face_ball_orientation, 0,
                DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0}, speed_mode, 0.0,
                event.common.robot.robotConstants()));
        };

        /**
         * Action to dribble the ball
         *
         * This action will orient the robot towards the destination, dribble to the
         * destination, and then pivot to face the expected orientation
         *
         * @param event DribbleFSM::Update
         */
        const auto dribble = [this](auto event) {
            Point ball_position = event.common.world.ball().position();
            auto [target_destination, target_orientation] =
                calculateNextDribbleDestinationAndOrientation(
                    event.common.world.ball(), event.common.robot,
                    event.control_params.dribble_destination,
                    event.control_params.final_dribble_orientation);
            AutoChipOrKick auto_chip_or_kick = AutoChipOrKick{AutoChipOrKickMode::OFF, 0};

            if (!event.control_params.allow_excessive_dribbling &&
                !comparePoints(ball_position, *continuous_dribbling_start_point,
                               MAX_CONTINUOUS_DRIBBLING_DISTANCE))
            {
                // give the ball a little kick
                auto_chip_or_kick =
                    AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, DRIBBLE_KICK_SPEED};
            }

            for (const auto &enemy_robot : event.common.world.enemyTeam().getAllRobots())
            {
                if (enemy_robot.isNearDribbler(ball_position, 0.005))
                {
                    if (acuteAngle(enemy_robot.position(), event.common.robot.position(),
                                   ball_position) < Angle::fromDegrees(90))
                    {
                        target_orientation += Angle::fromDegrees(45);
                        break;
                    }
                }
            }

            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), target_destination, target_orientation, 0,
                DribblerMode::MAX_FORCE, BallCollisionType::ALLOW, auto_chip_or_kick,
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
                event.common.robot.robotConstants()));
        };

        /**
         * Start dribbling
         *
         * @param event DribbleFSM::Update
         */
        const auto start_dribble = [this, dribble](auto event) {
			if (continuous_dribbling_start_point == nullptr)
			{
				// update continuous_dribbling_start_point once we start dribbling
				continuous_dribbling_start_point = std::make_shared<Point>(event.common.world.ball().position());
			}
            dribble(event);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *get_possession_s + update_e[have_possession] / start_dribble = dribble_s,
            get_possession_s + update_e[!have_possession] / get_possession,
            dribble_s + update_e[lost_possession] / get_possession = get_possession_s,
            dribble_s + update_e[!dribbling_done] / dribble,
            dribble_s + update_e[dribbling_done] / dribble  = X,
            X + update_e[!have_possession] / get_possession = get_possession_s,
            X + update_e[!dribbling_done] / dribble = dribble_s, X + update_e / dribble);
    }

   private:
    std::shared_ptr<Point> continuous_dribbling_start_point;
};
