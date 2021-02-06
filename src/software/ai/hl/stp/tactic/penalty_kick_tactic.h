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
    double calculateRobotCost(const Robot &robot, const World &world) const override;

    void accept(TacticVisitor &visitor) const override;

    Ball getBall() const;
    Field getField() const;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;

    // Evaluation function designed specifically for 1-on-1 penalty shots
    bool evaluate_penalty_shot();
    // Evaluation function designed specifically for determining the next potential shot
    // in a penalty kick
    Point evaluate_next_position();

    // Tactic parameters
    Ball ball;
    Field field;
    std::optional<Robot> enemy_goalie;

    static constexpr double PENALTY_KICK_SHOT_SPEED     = 5.0;
    static constexpr double PENALTY_KICK_GOALIE_MAX_ACC = 1.5;
    static constexpr double SSL_VISION_DELAY            = 0.30;  // seconds

    const Duration penalty_shot_timeout = Duration::fromSeconds(10);
};
