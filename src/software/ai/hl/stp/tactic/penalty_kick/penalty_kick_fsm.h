#pragma once

#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/intersection.h"

struct PenaltyKickFSM
{
    /**
     * Constructor for PenaltyKickFSM
     */
    PenaltyKickFSM();

    struct ControlParams
    {
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Helper function that determines whether the shooter robot has a viable shot on net.
     *
     * @param enemy_goalie  the enemy goalie that we are scoring against
     * @param field         the current field being played on
     * @param ball_position the coordinate location of the ball
     * @param robot         the shooter robot
     *
     * @return true if the robot has a viable shot and false if the enemy goalkeeper will
     * likely save the shot.
     */
    static bool evaluatePenaltyShot(std::optional<Robot> enemy_goalie, Field field,
                                    Point ball_position, Robot robot);

    /**
     * Helper function that returns the point on the enemy goal line where the shooter
     * should aim at.
     *
     * @param enemy_goalie  the goalie that the shooter is scoring against
     * @param field         the field being played on
     *
     * @return the Point on the enemy goal-line where the shooter robot should aim
     */
    static const Point evaluateNextShotPosition(std::optional<Robot> enemy_goalie,
                                                Field field);

    /**
     * Action that causes the shooter to shoot the ball.
     *
     * @param event          PenaltyKickFSM::Update event
     * @param processEvent   processes the KickFSM::Update
     */
    void shoot(const Update &event,
               boost::sml::back::process<KickFSM::Update> processEvent);

    /**
     * Action that updates the shooter's approach to the opposition net.
     *
     * @param event          PenaltyKickFSM::Update event
     * @param processEvent   processes the DribbleSkillFSM::Update
     */
    void updateApproachKeeper(
        const Update &event,
        boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    /**
     * Action that orients the shooter to prepare for a shot.
     *
     * @param event          PenaltyKickFSM::Update
     * @param processEvent   processes the DribbleSkillFSM::Update
     */
    void adjustOrientationForShot(
        const Update &event,
        boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    /**
     * Guard that returns true if the shooter has a good shot on goal or if it is
     * forced to shoot due to the penalty timeout.
     *
     * Requires complete approach to already be set.
     *
     * @param event  PenaltyKickFSM::Update
     */
    bool takePenaltyShot(const Update &event);

    /**
     * Returns true if we pass the timeout for completing the approach play towards
     * the keeper.
     *
     * Requires complete approach to already be set.
     *
     * @param event PenaltyKickFSM::Update
     */
    bool timeOutApproach(const Update &event);


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DribbleSkillFSM)
        DEFINE_SML_STATE(KickFSM)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(takePenaltyShot)
        DEFINE_SML_GUARD(timeOutApproach)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(shoot, KickFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(updateApproachKeeper, DribbleSkillFSM)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(adjustOrientationForShot, DribbleSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *DribbleSkillFSM_S + Update_E[!takePenaltyShot_G] / updateApproachKeeper_A,
            DribbleSkillFSM_S + Update_E[timeOutApproach_G] / shoot_A = KickFSM_S,
            DribbleSkillFSM_S + Update_E / adjustOrientationForShot_A,
            DribbleSkillFSM_S = KickFSM_S, KickFSM_S + Update_E / shoot_A, KickFSM_S = X,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    };

   private:
    static constexpr double PENALTY_KICK_POST_OFFSET = 0.03;
    static constexpr double PENALTY_KICK_SHOT_SPEED  = 5.0;

    // expected maximum acceleration of the opposition goalie robot
    static constexpr double PENALTY_KICK_GOALIE_MAX_ACC = 1.5;
    static constexpr double SSL_VISION_DELAY            = 0.30;  // seconds

    // the fraction of the enemy side of the field that we define to be the lower limit of
    // where we choose to shoot in other words, it helps define the minimum distance at
    // which we decide to potentially shoot
    static constexpr double PENALTY_KICK_MIN_SHOT_X_DISTANCE_FACTOR = 1.0 / 3.0;

    // timeout that forces a shot after the robot approaches the ball and advances
    // towards the keeper
    // these two timeouts together must be <= 9 seconds
    static const inline Duration PENALTY_FORCE_SHOOT_TIMEOUT = Duration::fromSeconds(4);
    static const inline Duration PENALTY_FINISH_APPROACH_TIMEOUT =
        Duration::fromSeconds(4);

   private:
    std::optional<Timestamp> complete_approach;
    Angle shot_angle;

    // TODO: Remove this once we actually pass Strategy into this tactic
    std::shared_ptr<Strategy> strategy;
};
