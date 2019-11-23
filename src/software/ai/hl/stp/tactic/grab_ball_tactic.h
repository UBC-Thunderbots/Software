#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The GrabBallTactic will try gain possession of the ball. If the ball is loose on the
 * field it will move to try intercept it, and if an enemy has the ball the robot will try
 * take it away.
 */
class GrabBallTactic : public Tactic
{
   public:
    /**
     * Creates a new GrabBallTactic
     *
     * @param field The field being played on
     * @param ball The Ball being played with
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit GrabBallTactic(const Field &field, const Ball &ball, const Team &enemy_team,
                            bool loop_forever);

    std::string getName() const override;

    /**
     * Updates the parameters for this MoveTactic.
     *
     * @param field The field being played on
     * @param ball The ball being played with on the field
     */
    void updateParams(const Field &field, const Ball &ball, const Team &enemy_team);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) override;

    void accept(TacticVisitor &visitor) const override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type &yield) override;

    // Spin 2.5 rotations per second to steal the ball
    const AngularVelocity STEAL_BALL_SPIN_SPEED = AngularVelocity::fromDegrees(360 * 2.5);

    // Tactic parameters
    Field field;
    Ball ball;
    Team enemy_team;

    const double BALL_DIST_FROM_ENEMY = 2 * ROBOT_MAX_RADIUS_METERS;
};
