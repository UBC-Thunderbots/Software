#pragma once

#include <include/boost/sml.hpp>

#include "proto/tactic.pb.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"

struct DribbleSkillFSM
{
    class GetBallControl;
    class Dribble;
    class LoseBall;
    class WaitForBackspin;

    struct ControlParams
    {
        // The destination for dribbling the ball
        std::optional<Point> dribble_destination;
        // The final orientation to face the ball when finishing dribbling
        std::optional<Angle> final_dribble_orientation;
        // Controls whether to allow excessive dribbling, i.e. more than 1 metre at a time
        TbotsProto::ExcessiveDribblingMode excessive_dribbling_mode;
        // Max allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_speed_dribble =
            TbotsProto::MaxAllowedSpeedMode::DRIBBLE;
        TbotsProto::MaxAllowedSpeedMode max_speed_get_possession =
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Converts the ball position to the robot's position given the direction that the
     * robot faces the ball
     *
     * @param ball_position The ball position
     * @param face_ball_angle The angle to face the ball
     * @param additional_offset Additional offset from facing the ball
     *
     * @return the point that the robot should be positioned to face the ball and dribble
     * the ball
     */
    static Point robotPositionToFaceBall(const Point &ball_position,
                                         const Angle &face_ball_angle,
                                         double additional_offset = 0.0);

    /**
     * Calculates the interception point for intercepting balls
     *
     * @param robot The robot to do the interception
     * @param ball The ball to intercept
     * @param field The field to intercept on
     * @param dribble_config The dribble config to use
     *
     * @return the best interception point
     */
    // TODO (#1968): Merge this functionality with findBestInterceptForBall in the
    // evaluation folder
    static Point findInterceptionPoint(const Robot &robot, const Ball &ball,
                                       const Field &field,
                                       const TbotsProto::DribbleConfig &dribble_config);

    /**
     * Gets the destination to dribble the ball to from the update event
     *
     * @param ball_position the position of the ball
     * @param dribble_destination the dribble destination
     *
     * @return the destination to dribble the ball to
     */
    static Point getDribbleBallDestination(const Point &ball_position,
                                           std::optional<Point> dribble_destination);

    /**
     * Gets the final dribble orientation from the update event
     *
     * @param ball_position the position of the ball
     * @param robot_position the position of the robot
     * @param dribble_destination the final dribble orientation
     *
     * @return the final orientation to finish dribbling facing
     */
    static Angle getFinalDribbleOrientation(
        const Point &ball_position, const Point &robot_position,
        std::optional<Angle> final_dribble_orientation);

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
        std::optional<Angle> final_dribble_orientation_opt);

    /**
     * Action to get control of the ball
     *
     * If the ball is moving quickly, then move in front of the ball
     * If the ball is moving slowly, then chase the ball
     *
     * @param event the Update event
     */
    void getBallControl(const Update &event);

    /**
     * Action to start waiting
     *
     * @param event the Update event
     */
    void startWait(const Update &event);

    /**
     * Guard to check if wait is over
     *
     * @param event the Update event
     */
    bool waitDone(const Update &event);

    /**
     * Action to wait for ball to be accelerated by the dribbler
     *
     *
     * @param event the Update event
     */
    void waitForBackspin(const Update &event);

    /**
     * Action to dribble the ball
     *
     * This action will orient the robot towards the destination, dribble to the
     * destination, and then pivot to face the expected orientation
     *
     * @param event the Update event
     */
    void dribble(const Update &event);

    /**
     * Action to lose control of the ball
     *
     * @param event the Update event
     */
    void loseBall(const Update &event);

    /**
     * Guard that checks if the robot has control of the ball
     *
     * @param event the Update event
     *
     * @return if the robot has control of the ball
     */
    bool haveBallControl(const Update &event);

    /**
     * Guard that checks if the robot has lost control of the ball
     *
     * @param event the Update event
     *
     * @return if the robot has lost control of the ball
     */
    bool lostBallControl(const Update &event);

    /**
     * Guard that checks if the ball is at the dribble_destination and robot is facing
     * the right direction with control of the ball
     *
     * @param event the Update event
     *
     * @return if the ball is at the dribble_destination, robot is facing the correct
     * direction and has control of the ball
     */
    bool dribblingDone(const Update &event);

    /**
     * Guard that checks if the the robot should lose control of the ball to avoid
     * excessive dribbling
     *
     * @param event the Update event
     *
     * @return if the ball should be lost
     */
    bool shouldLoseBall(const Update &event);

    /**
     * Guard that checks whether the excessive_dribbling_mode control param
     * is NOT set to TbotsProto::ExcessiveDribblingMode::TERMINATE
     *
     * @param event the Update event
     *
     * @return false if the excessive_dribbling_mode control param is set
     * to TbotsProto::ExcessiveDribblingMode::TERMINATE, true otherwise
     */
    bool shouldExcessivelyDribble(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBallControl)
        DEFINE_SML_STATE(WaitForBackspin)
        DEFINE_SML_STATE(Dribble)
        DEFINE_SML_STATE(LoseBall)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(haveBallControl)
        DEFINE_SML_GUARD(waitDone)
        DEFINE_SML_GUARD(lostBallControl)
        DEFINE_SML_GUARD(dribblingDone)
        DEFINE_SML_GUARD(shouldLoseBall)
        DEFINE_SML_GUARD(shouldExcessivelyDribble)

        DEFINE_SML_ACTION(loseBall)
        DEFINE_SML_ACTION(getBallControl)
        DEFINE_SML_ACTION(startWait)
        DEFINE_SML_ACTION(waitForBackspin)
        DEFINE_SML_ACTION(dribble)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBallControl_S + Update_E[haveBallControl_G] / startWait_A = WaitForBackspin_S,
            GetBallControl_S + Update_E / getBallControl_A,

            WaitForBackspin_S + Update_E[lostBallControl_G] / getBallControl_A = GetBallControl_S,
            WaitForBackspin_S + Update_E[waitDone_G] / dribble_A = Dribble_S,
            WaitForBackspin_S + Update_E / waitForBackspin_A,

            Dribble_S + Update_E[lostBallControl_G] / getBallControl_A = GetBallControl_S,
            Dribble_S + Update_E[shouldLoseBall_G && shouldExcessivelyDribble_G] /
                            loseBall_A = LoseBall_S,
            Dribble_S +
                Update_E[shouldLoseBall_G && !shouldExcessivelyDribble_G] / dribble_A = X,
            Dribble_S + Update_E[dribblingDone_G] / dribble_A                         = X,
            Dribble_S + Update_E / dribble_A,

            LoseBall_S + Update_E[lostBallControl_G] / getBallControl_A =
                GetBallControl_S,
            LoseBall_S + Update_E / loseBall_A,

            X + Update_E[!shouldExcessivelyDribble_G] / dribble_A,
            X + Update_E[lostBallControl_G] / getBallControl_A = GetBallControl_S,
            X + Update_E[!dribblingDone_G] / dribble_A         = Dribble_S,
            X + Update_E / dribble_A);
    }

    Timestamp start_time;
    constexpr static double const WAIT_FOR_BACKSPIN_S = 0.2;

};
