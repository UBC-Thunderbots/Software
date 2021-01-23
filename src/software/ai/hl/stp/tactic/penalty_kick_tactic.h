#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * This tactic is for a robot performing a penalty kick.
 */

class PenaltyKickTactic : public Tactic
{
   public:
    /**
     * Creates a new PenaltyKickTactic
     *
     * @param ball : The ball that we're trying to shoot
     * @param field : The field we are playing on
     * @param enemy_goalie : Optional variable for the enemy goalie robot
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit PenaltyKickTactic(const Ball &ball, const Field &field,
                               const std::optional<Robot> &enemy_goalie,
                               bool loop_forever);

    PenaltyKickTactic() = delete;

    void updateWorldParams(const World &world) override;

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) override;

    void accept(TacticVisitor &visitor) const override;

    Ball getBall() const;
    Field getField() const;

    // Evaluation function designed specifically for 1-on-1 penalty shots
    bool evaluatePenaltyShot();

    // Evaluation function designed specifically for determining the next potential shot
    // in a penalty kick
    Point evaluateNextShotPosition();

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;

    // Tactic parameters
    Ball ball;
    Field field;
    std::optional<Robot> enemy_goalie;

    static constexpr double PENALTY_KICK_SHOT_SPEED = 5.0;
    // expected maximum acceleration of the opposition goalie robot
    static constexpr double PENALTY_KICK_GOALIE_MAX_ACC = 1.5;
    static constexpr double SSL_VISION_DELAY            = 0.30;  // seconds
	// offset from the goal post in y direction when shooting
    static constexpr double PENALTY_KICK_POST_OFFSET = 0.04;

    // this is the timeout that forces a shot after the robot approaches the
    // ball and advances towards the keeper
    const Duration PENALTY_FORCE_SHOOT_TIMEOUT = Duration::fromSeconds(4);
    const Duration PENALTY_SHOT_TIMEOUT        = Duration::fromSeconds(10);
};
