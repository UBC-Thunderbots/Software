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
    /**
     * Creates a new ReceiverTactic
     *
     * @param pass The pass this tactic should try to receive
     * @param loop_forever Whether or not this Tactic should never complete. If true,
     *                     the tactic will be restarted every time it completes
     */
    explicit ReceiverTactic(const Pass pass);

    ReceiverTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the control parameters for this ReceiverTactic.
     *
     * @param updated_pass The pass this tactic should try to receive
     */
    void updateControlParams(const Pass& updated_pass);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

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
    bool done() const override;

   private:
    // The minimum proportion of open net we're shooting on vs the entire size of the net
    // that we require before attempting a shot
    static constexpr double MIN_SHOT_NET_PERCENT_OPEN = 0.3;

    // The maximum deflection angle that we will attempt a one-touch kick towards the
    // enemy goal with
    static constexpr Angle MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT = Angle::fromDegrees(90);

    void calculateNextAction(ActionCoroutine::push_type& yield) override;
    void updateIntent(const TacticUpdate& tactic_update) override;

    /**
     * Finds a feasible shot for the robot, if any.
     *
     * A feasible shot is one where the robot does not have to rotate to much to
     * take the shot, and there is a sufficient percentage of the net open for the shot.
     *
     * @return A feasible shot or std::nullopt if there is no feasible shot
     */
    std::optional<Shot> findFeasibleShot();

    Pass pass;

    FSM<ReceiverFSM> fsm;

    ReceiverFSM::ControlParams control_params;
};
