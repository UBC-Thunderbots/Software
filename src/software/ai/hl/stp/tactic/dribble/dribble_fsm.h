#pragma once

#include "shared/constants.h"
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
    class GetPossession;
    class Dribble;
    class LoseBall;

    /**
     * Constructor for DribbleFSM
     *
     * @param continuous_dribbling_start_point A pointer to a Point to track the
     * continuous dribbling start point
     */
    explicit DribbleFSM(void) : continuous_dribbling_start_point(Point()) {}

    struct ControlParams
    {
        // The destination for dribbling the ball
        std::optional<Point> dribble_destination;
        // The final orientation to face the ball when finishing dribbling
        std::optional<Angle> final_dribble_orientation;
        // whether to allow excessive dribbling, i.e. more than 1 metre at a time
        bool allow_excessive_dribbling;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // Threshold to determine if the ball is at the destination determined experimentally
    static constexpr double BALL_CLOSE_TO_DEST_THRESHOLD = 0.1;
    // Threshold to determine if the robot has the expected orientation when dribbling the
    // ball
    static constexpr Angle FACE_DESTINATION_CLOSE_THRESHOLD = Angle::fromDegrees(5);
    // Threshold to determine if the robot has the expected orientation when completing
    // the dribble
    static constexpr Angle FINAL_DESTINATION_CLOSE_THRESHOLD = Angle::fromDegrees(1);
    // Kick speed when breaking up continuous dribbling
    static constexpr double DRIBBLE_KICK_SPEED = 0.05;
    // Maximum distance to continuously dribble the ball, slightly conservative to not
    // break the 1 meter rule
    static constexpr double MAX_CONTINUOUS_DRIBBLING_DISTANCE = 0.9;
    // robot speed at which the robot is done dribbling
    static constexpr double ROBOT_DRIBBLING_DONE_SPEED = 0.2;  // m/s

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
                                         const Angle &face_ball_angle,
                                         double additional_offset = 0.0);

    /**
     * Calculates the interception point for intercepting balls
     *
     * @param robot The robot to do the interception
     * @param ball The ball to intercept
     * @field The field to intercept on
     *
     * @return the best interception point
     */
    // TODO (#1968): Merge this functionality with findBestInterceptForBall in the
    // evaluation folder
    static Point findInterceptionPoint(const Robot &robot, const Ball &ball,
                                       const Field &field);

    /**
     * Gets the destination to dribble the ball to from the update event
     *
     * @param event DribbleFSM::Update
     *
     * @return the destination to dribble the ball to
     */
    static Point getDribbleBallDestination(const Point &ball_position,
                                           std::optional<Point> dribble_destination);

    /**
     * Gets the final dribble orientation from the update event
     *
     * @param event DribbleFSM::Update
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
     * Action to get possession of the ball
     *
     * If the ball is moving quickly, then move in front of the ball
     * If the ball is moving slowly, then chase the ball
     *
     * @param event DribbleFSM::Update
     */
    void getPossession(const Update &event);

    /**
     * Action to dribble the ball
     *
     * This action will orient the robot towards the destination, dribble to the
     * destination, and then pivot to face the expected orientation
     *
     * @param event DribbleFSM::Update
     */
    void dribble(const Update &event);

    /**
     * Start dribbling
     *
     * @param event DribbleFSM::Update
     */
    void startDribble(const Update &event);

    void loseBall(const Update &event);

    /**
     * Guard that checks if the robot has possession of the ball
     *
     * @param event DribbleFSM::Update
     *
     * @return if the ball has been have_possession
     */
    bool havePossession(const Update &event);

    /**
     * Guard that checks if the robot has lost possession of the ball
     *
     * @param event DribbleFSM::Update
     *
     * @return if the ball possession has been lost
     */
    bool lostPossession(const Update &event);

    /**
     * Guard that checks if the ball is at the dribble_destination and robot is facing
     * the right direction with possession of the ball
     *
     * @param event DribbleFSM::Update
     *
     * @return if the ball is at the dribble_destination, robot is facing the correct
     * direction and ahs possession of the ball
     */
    bool dribblingDone(const Update &event);

    bool shouldLoseBall(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(GetPossession)
        DEFINE_SML_STATE(Dribble)
        DEFINE_SML_STATE(LoseBall)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(havePossession)
        DEFINE_SML_GUARD(lostPossession)
        DEFINE_SML_GUARD(dribblingDone)
        DEFINE_SML_GUARD(shouldLoseBall)
        DEFINE_SML_ACTION(startDribble)
        DEFINE_SML_ACTION(loseBall)
        DEFINE_SML_ACTION(getPossession)
        DEFINE_SML_ACTION(dribble)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *GetPossession_S + Update_E[havePossession_G] / startDribble_A = Dribble_S,
            GetPossession_S + Update_E[!havePossession_G] / getPossession_A,
            Dribble_S + Update_E[lostPossession_G] / getPossession_A = GetPossession_S,
            Dribble_S + Update_E[shouldLoseBall_G] / loseBall_A      = LoseBall_S,
            Dribble_S + Update_E[!dribblingDone_G] / dribble_A,
            Dribble_S + Update_E[dribblingDone_G] / dribble_A = X,
            LoseBall_S + Update_E[!lostPossession_G] / loseBall_A,
            LoseBall_S + Update_E[lostPossession_G] / getPossession_A = GetPossession_S,
            X + Update_E[lostPossession_G] / getPossession_A          = GetPossession_S,
            X + Update_E[!dribblingDone_G] / dribble_A                = Dribble_S,
            X + Update_E / dribble_A);
    }

   private:
    Point continuous_dribbling_start_point;

    // if ball and front of robot are separated by this amount, then we've lost possession
    static constexpr double LOSE_BALL_POSSESSION_THRESHOLD = 0.02;
};
