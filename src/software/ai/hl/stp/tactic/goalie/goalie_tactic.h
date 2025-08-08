#pragma once

#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"

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
class GoalieTactic : public TacticBase<GoalieFSM, PivotKickFSM, DribbleFSM>
{
   public:
    /**
     * Creates a new GoalieTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     * @param max_allowed_speed_mode The maximum allowed speed mode
     */
    explicit GoalieTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    GoalieTactic() = delete;

    // TODO: make sure the calling function uses this instead
    /**
     * Updates the current goalie speed mode.
     * Modifies max_allowed_speed_mode
     * @param new_speed_mode the new speed mode to use.
     */
    void updateMaxSpeedMode(TbotsProto::MaxAllowedSpeedMode new_speed_mode);

    void updateControlParams(bool should_move_to_goal_line);

    void accept(TacticVisitor &visitor) const override;

   private:
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
};
