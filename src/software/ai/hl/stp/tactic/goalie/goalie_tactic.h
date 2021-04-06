#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/segment.h"

/**
 * This tactic is used to defend the ball from going into the goal. The tactic
 * is assigned to the robot that is selected using a DynamicParameter, and stays
 * that way throughout all the plays that require a goalie.
 *
 * If the ball is moving faster than a threshold torwards the net, moves to intercept
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
     */
    explicit GoalieTactic(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config);

    GoalieTactic() = delete;

    /*
     * Restrains the goalie to a rectangle, with the preferred point being the one
     * that intersects the point the goalie wants to move to and the center of the
     * goal
     *
     * @param goalie_desired_position The point the goalie would like to go to
     * @param goalie_restricted_area The rectangle that the goalie is to stay in
     * @returns goalie_suggested_position That the goalie should go to
     */
    /*std::optional<Point> restrainGoalieInRectangle(Point goalie_desired_position,
                                                   Rectangle goalie_restricted_area);*/

    void updateWorldParams(const World &world) override;

    /**
     * Updates the params assuming that the max allowed speed mode is the physical limits
     *
     */
    void updateControlParams();

    double calculateRobotCost(const Robot &robot, const World &world) const override;

    bool isGoalieTactic() const override;

    void accept(TacticVisitor &visitor) const override;
    bool done() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate &tactic_update) override;

    // Tactic parameters
    HFSM<GoalieFSM> fsm;
    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config;
    GoalieFSM::ControlParams control_params;
};
