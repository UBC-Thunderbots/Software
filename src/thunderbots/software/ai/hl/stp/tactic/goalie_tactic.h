#pragma once

#include <geom/point.h>
#include <geom/rectangle.h>
#include <geom/segment.h>

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
     * @param ball The const reference to the ball on the field
     * @param field The const reference to the field this tactic will run
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     */
    void updateParams(const Ball &ball, const Field &field, const Team &friendly_team,
                      const Team &enemy_team);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. The goalie
     * assigned through the dynamic parameter will be prioritized by setting the cost
     * very high, so only the robot assigned to be the goalie will be the cheapest to
     * do so.
     *
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type &yield) override;

    /*
     * Restrains the goalie to a rectangle, with the prefered point being the one
     * that intersects the point the goalie wants to move to and the center of the
     * goal
     *
     * TODO Fix this function name, it doesn't make sense
     *
     * @param goalie_desired_position The point the goalie would like to go to
     * @param goalie_restricted_area The rectangle that the goalie is to stay in
     * @returns goalie_suggested_position That the goalie should go to
     */
    std::optional<Point> restrainGoalieInRectangle(Point goalie_desired_position,
                                                   Rectangle goalie_restricted_area);

    // Tactic parameters
    Ball ball;
    Field field;
    Team friendly_team;
    Team enemy_team;
};
