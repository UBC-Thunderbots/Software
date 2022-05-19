#pragma once

#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * This tactic is used to defend the ball from going into the goal. The tactic
 * is assigned to the robot that is selected using a DynamicParameter, and stays
 * that way throughout all the plays that require a goalie.
 *
 * If the ball is moving faster than a threshold towards the net, moves to intercept
 * the ball. If not, returns primitives that position the robot in a cone between the ball
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
     * @param ai_config The AI configuration
     * @param max_allowed_speed_mode The maximum allowed speed mode
     */
    explicit GoalieTactic(std::shared_ptr<const AiConfig> ai_config,
                          TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode =
                              TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    GoalieTactic() = delete;

    void updateControlParams(bool should_move_to_goal_line);

    void accept(TacticVisitor &visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<GoalieFSM>>> fsm_map;

    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;

    GoalieFSM::ControlParams control_params;
    std::shared_ptr<const AiConfig> ai_config;
};
