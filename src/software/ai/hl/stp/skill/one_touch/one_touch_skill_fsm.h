#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/skill/skill_fsm.h"
#include "software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/primitive/move_primitive.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/ray.h"

struct OneTouchSkillFSM
{
    class OneTouchState;

    struct ControlParams {};

    DEFINE_SKILL_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Given a shot and the ball, figures out the angle the robot should be facing
     * to perform a one-touch shot.
     *
     * @param shot The shot to take
     * @param ball The ball on the field
     */
    static Angle getOneTouchShotDirection(const Ray& shot, const Ball& ball);

    /**
     * Figures out the location of the one-touch shot and orientation the robot should
     * face
     *
     * @param robot The robot performing the one-touch
     * @param ball The ball on the field
     * @param best_shot_target The point to shoot at
     */
    static Shot getOneTouchShotPositionAndOrientation(const Robot& robot,
                                                      const Ball& ball,
                                                      const Field &field,
                                                      const Point& best_shot_target);

    /**
     * @param event the Update event
     */
    bool foundShot(const Update& event);

    /**
     * @param event the Update event
     */
    bool ballKicked(const Update& event);

    /**
     * Action that updates the DribbleSkillFSM to get control of the ball and pivot
     *
     * @param event the Update event
     * @param processEvent processes the DribbleSkillFSM::Update event
     */
    void getBallControlAndPivot(
        const Update& event,
        boost::sml::back::process<DribbleSkillFSM::Update> processEvent);

    /**
     * Action to perform one touch
     *
     * @param event the Update event
     */
    void updateOneTouch(const Update& event);

    static constexpr double BALL_MIN_MOVEMENT_SPEED = 0.04;

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(DribbleSkillFSM)
        DEFINE_SML_STATE(OneTouchState)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_GUARD(foundShot)
        DEFINE_SML_GUARD(ballKicked)
        DEFINE_SML_ACTION(updateOneTouch)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(getBallControlAndPivot, DribbleSkillFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *DribbleSkillFSM_S + Update_E[!foundShot_G] / SET_STOP_PRIMITIVE_ACTION = X,
            DribbleSkillFSM_S + Update_E / getBallControlAndPivot_A,
            DribbleSkillFSM_S = OneTouchState_S,
            OneTouchState_S + Update_E[!foundShot_G || ballKicked_G] / SET_STOP_PRIMITIVE_ACTION = X,
            OneTouchState_S + Update_E / updateOneTouch_A,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION = X);
    }

   private:
    std::optional<Shot> best_shot_;
};
