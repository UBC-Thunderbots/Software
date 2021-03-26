#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/kick_intent.h"

/**
 * The KickTactic will move the assigned robot to the given kick origin and then
 * kick the ball to the kick target.
 */

class KickTactic : public Tactic
{
   public:
    /**
     * Creates a new KickTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit KickTactic(bool loop_forever);

    KickTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the params for this tactic that cannot be derived from the world
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The distance between the starting location
     * of the kick and the location of the first bounce
     */
    void updateControlParams(const Point& kick_origin, const Angle& kick_direction,
                             double kick_speed_meters_per_second);

    /**
     * Updates the control parameters for this KickTactic.
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     */
    void updateControlParams(const Point& kick_origin, const Point& kick_target,
                             double kick_speed_meters_per_second);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    void accept(TacticVisitor& visitor) const override;

    bool done() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;
    void updateIntent(const TacticUpdate& tactic_update) override;

    HFSM<KickFSM> fsm;

    // Tactic parameters
    KickFSM::ControlParams control_params;
};
