#pragma once

#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"
#include "software/geom/ray.h"

/**
 * This tactic is for a robot receiving a pass. It should be used in conjunction with
 * the `AttackerTactic` in order to complete the pass.
 *
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to receive the pass as possible
 */
class ReceiverTactic : public Tactic
{
   public:
    explicit ReceiverTactic(const TbotsProto::ReceiverTacticConfig& receiver_config);

    /**
     * Updates the control parameters for this ReceiverTactic.
     *
     * @param updated_pass The pass this tactic should try to receive
     * @param disable_one_touch_shot If set to true, the receiver will not perform a
     * one-touch The robot will simply receive and dribble.
     */
    void updateControlParams(std::optional<Pass> updated_pass,
                             bool disable_one_touch_shot = false);

    /**
     * Calculate the angle the robot should be at in order to perform the given shot
     *
     * @param shot A Ray representing the shot we want to take
     * @param ball The ball we want to shoot
     *
     * @return The angle to position the robot at so that it can redirect the ball to
     *         the shot vector at the position where the shot vector and ball velocity
     *         vectors intersect
     */
    static Angle getOneTimeShotDirection(const Ray& shot, const Ball& ball);

    /**
     * Get the position the receiver robot should be in, and the orientation it should
     * have in order to perform a one-time shot when receiving a pass.
     *
     * @param robot The receiver robot
     * @param ball The ball
     * @param best_shot_target The point the receiver robot should shoot at
     * @return A Shot that the given robot can perform with the given ball in order to
     * shoot at the given point
     */
    static Shot getOneTimeShotPositionAndOrientation(const Robot& robot, const Ball& ball,
                                                     const Point& best_shot_target);

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    // The minimum proportion of open net we're shooting on vs the entire size of the net
    // that we require before attempting a shot
    static constexpr double MIN_SHOT_NET_PERCENT_OPEN = 0.3;

    // The maximum deflection angle that we will attempt a one-touch kick towards the
    // enemy goal with
    static constexpr Angle MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT = Angle::fromDegrees(90);

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<ReceiverFSM>>> fsm_map;

    /**
     * Finds a feasible shot for the robot, if any.
     *
     * A feasible shot is one where the robot does not have to rotate to much to
     * take the shot, and there is a sufficient percentage of the net open for the shot.
     *
     * @return A feasible shot or std::nullopt if there is no feasible shot
     */
    std::optional<Shot> findFeasibleShot();

    ReceiverFSM::ControlParams control_params;
    TbotsProto::ReceiverTacticConfig receiver_config;
};
