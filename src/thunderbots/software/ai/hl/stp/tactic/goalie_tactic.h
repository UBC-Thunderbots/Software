#pragma once

#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/tactic/tactic.h"

/**
// TODO: commment
 */
class GoalieTactic : public Tactic
{
   public:
    /**
     * Creates a new GoalieTactic
     */
    explicit GoalieTactic(const Ball &ball, const Field &field, const Team &friendly_team,
                          const Team &enemy_team);

    std::string getName() const override;

    /**
     * Updates the parameters for this GoalieTactic.
     *
// TODO: comment
     */
    void updateParams(const Ball &ball, const Field &field, const Team &friendly_team,
                      const Team &enemy_team);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
// TODO:
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type &yield) override;

    // Tactic parameters
    Ball ball;
    Field field;
    Team friendly_team;
    Team enemy_team;

    // How slow the ball must be moving for us to clear it from the defense area
    double BALL_SLOW_SPEED_THRESHOLD = 0.1;
};
