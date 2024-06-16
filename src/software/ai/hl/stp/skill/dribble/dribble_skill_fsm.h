#pragma once

#include <include/boost/sml.hpp>

#include "software/ai/hl/stp/skill/skill_fsm.h"

struct DribbleSkillFSM
{
    class GetBallControl;
    class Dribble;
    class LoseBall;

    struct ControlParams
    {
        // The destination for dribbling the ball
        std::optional<Point> dribble_destination;
        // The final orientation to face the ball when finishing dribbling
        std::optional<Angle> final_dribble_orientation;
        // whether to allow excessive dribbling, i.e. more than 1 metre at a time
        bool allow_excessive_dribbling;
    };

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Gets the destination to dribble the ball to from the update event
     *
     * @param event the Update event
     *
     * @return the destination to dribble the ball to
     */
    static Point getDribbleBallDestination(const Point &ball_position,
                                           std::optional<Point> dribble_destination);

    /**
     * Gets the final dribble orientation from the update event
     *
     * @param event the Update event
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
     * Action to dribble the ball
     *
     * This action will orient the robot towards the destination, dribble to the
     * destination, and then pivot to face the expected orientation
     *
     * @param event the Update event
     */
    void dribble(const Update &event);

    /**
     * Start dribbling
     *
     * @param event the Update event
     */
    void startDribble(const Update &event);

    /**
     * Action to lose BallControl of the ball
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

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetBallControl)
        DEFINE_SML_STATE(Dribble)
        DEFINE_SML_STATE(LoseBall)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(haveBallControl)
        DEFINE_SML_GUARD(lostBallControl)
        DEFINE_SML_GUARD(dribblingDone)
        DEFINE_SML_GUARD(shouldLoseBall)
        DEFINE_SML_ACTION(startDribble)
        DEFINE_SML_ACTION(loseBall)
        DEFINE_SML_ACTION(getBallControl)
        DEFINE_SML_ACTION(dribble)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetBallControl_S + Update_E[haveBallControl_G] / startDribble_A = Dribble_S,
            GetBallControl_S + Update_E / getBallControl_A,
            Dribble_S + Update_E[lostBallControl_G] / getBallControl_A = GetBallControl_S,
            Dribble_S + Update_E[shouldLoseBall_G] / loseBall_A        = LoseBall_S,
            Dribble_S + Update_E[dribblingDone_G] / dribble_A          = X,
            Dribble_S + Update_E / dribble_A,
            LoseBall_S + Update_E[lostBallControl_G] / getBallControl_A =
                GetBallControl_S,
            LoseBall_S + Update_E / loseBall_A,
            X + Update_E[lostBallControl_G] / getBallControl_A = GetBallControl_S,
            X + Update_E[!dribblingDone_G] / dribble_A         = Dribble_S,
            X + Update_E / dribble_A                           = X);
    }

   private:
    Point continuous_dribbling_start_point;
};
