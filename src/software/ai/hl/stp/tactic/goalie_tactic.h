#pragma once

#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"

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
     */
    explicit GoalieTactic(const Ball &ball, const Field &field, const Team &friendly_team,
                          const Team &enemy_team);

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

    std::string getName() const override;

    /**
     * Updates the world parameters for this GoalieTactic.
     *
     * @param ball The const reference to the ball on the field
     * @param field The const reference to the field this tactic will run
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     */
    void updateWorldParams(const Ball &ball, const Field &field,
                           const Team &friendly_team, const Team &enemy_team);

    double calculateRobotCost(const Robot &robot, const World &world) override;

    bool isGoalieTactic() const override;

    void accept(MutableTacticVisitor &visitor) override;

    Ball getBall() const;
    Field getField() const;
    Team getFriendlyTeam() const;
    Team getEnemyTeam() const;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;

    // Tactic parameters
    Ball ball;
    Field field;
    Team friendly_team;
    Team enemy_team;
};
