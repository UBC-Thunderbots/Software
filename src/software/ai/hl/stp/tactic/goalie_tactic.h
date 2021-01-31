#pragma once

#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/action/autochip_move_action.h"
#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/segment.h"
#include "software/parameter/dynamic_parameters.h"

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
     * @param ball The ball
     * @param field The field
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     * @param goalie_tactic_config The config to fetch parameters from
     */
    explicit GoalieTactic(const Ball &ball, const Field &field, const Team &friendly_team,
                          const Team &enemy_team,
                          std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config =
                              DynamicParameters->getAiConfig()->getGoalieTacticConfig());

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
    std::optional<Point> restrainGoalieInRectangle(Point goalie_desired_position,
                                                   Rectangle goalie_restricted_area);

    void updateWorldParams(const World &world) override;

    double calculateRobotCost(const Robot &robot, const World &world) override;

    /**
     * Gets intersections between the ball velocity ray and the full goal segment
     */
    std::vector<Point> getIntersectionsBetweenBallVelocityAndFullGoalSegment();

    /**
     * Creates action to panic and stop the ball
     *
     * @param autochip_move_action The action to reuse
     * @param stop_ball_point The point to the stop the ball
     *
     * @return the action to use to stop the ball
     */
    std::shared_ptr<Action> panicAndStopBall(
        std::shared_ptr<AutochipMoveAction> autochip_move_action,
        const Point &stop_ball_point);

    /**
     * Chip ball if safe
     *
     * @param chip_action The chip action to reuse
     * @param stop_action The stop action to reuse
     *
     * @return Action to chip the ball if it is safe to do so
     */
    std::shared_ptr<Action> chipBallIfSafe(std::shared_ptr<ChipAction> chip_action,
                                           std::shared_ptr<StopAction> stop_action);

    /**
     * Position robot to block potential shots
     *
     * @param autochip_move_action The action to reuse
     *
     * @return The Action to position the robot to block the shot
     */
    std::shared_ptr<Action> positionToBlockShot(
        std::shared_ptr<AutochipMoveAction> autochip_move_action);

    bool isGoalieTactic() const override;

    void accept(TacticVisitor &visitor) const override;

    Ball getBall() const;
    Field getField() const;
    Team getFriendlyTeam() const;
    Team getEnemyTeam() const;

    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;

    // Tactic parameters
    Ball ball;
    Field field;
    Team friendly_team;
    Team enemy_team;
    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config;
};
