#pragma once

#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * This tactic is used to defend the ball from going into the goal. The tactic
 * is assigned to the robot that is selected using a DynamicParameter, and stays
 * that way throughout all the plays that require a goalie.
 *
 * If the ball is moving faster than a threshold towards the net, moves to intercept
 * the ball. If not, returns intents that position the robot in a cone between the ball
 * and the two goal posts, in such a way that the robot would have to move a minimal
 * distance either way to intercept a potential straight shot into the net.
 *
 */
class GoalieTactic : public Tactic
{
   public:
    /**
     * Creates a new GoalieTactic
     *
     * @param goalie_tactic_config The config to fetch parameters from
     * @param max_allowed_speed_mode The maximum allowed speed mode
     */
    explicit GoalieTactic(
        std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config,
        MaxAllowedSpeedMode max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    GoalieTactic() = delete;

    void updateWorldParams(const World &world) override;

    double calculateRobotCost(const Robot &robot, const World &world) const override;

    void accept(TacticVisitor &visitor) const override;
    bool done() const override;
    bool isGoalieTactic() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate &tactic_update) override;

    FSM<GoalieFSM> fsm;
    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config;
};
