#pragma once

#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/segment.h"

/**
 * A crease defender moves around the exterior of our defense box to help shadow
 * shots against the enemy
 *
 *                        XX   <-- Enemy
 *                        XX
 *                          O  <-- Ball
 *
 *
 *
 *                           ++   ++   <--- Two crease defenders
 *                           ++   ++
 *
 *                       +--------------------+
 *                       |                    |
 *                       |                    |
 *                       |         ++    <-- Goalie
 *+----------------------+---------++---------+------------------+
 */
class CreaseDefenderTactic : public Tactic
{
   public:
    enum LeftOrRight
    {
        LEFT,
        RIGHT
    };

    /**
     * Creates a new CreaseDefenderTactic
     */
    explicit CreaseDefenderTactic(const Field &field, const Ball &ball,
                                  const Team &friendly_team, const Team &enemy_team,
                                  LeftOrRight left_or_right);

    std::string getName() const override;

    /**
     * Updates the world parameters for this CreaseDefenderTactic.
     *
     * @param ball
     * @param field
     * @param friendly_team
     * @param enemy_team
     */
    void updateWorldParams(const Ball &ball, const Field &field,
                           const Team &friendly_team, const Team &enemy_team);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) override;

    void accept(TacticVisitor &visitor) const override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type &yield) override;

    /**
     * Calculate the position and orientation we would like the defender to be in
     *
     * @return The position and orientation we would like the defender to be in, or
     * std::nullopt if we could not compute one
     */
    std::optional<std::pair<Point, Angle>> calculateDesiredState(const Robot &robot);

    /**
     * Gets the segments that make up the path the Crease Defender should follow
     * @param field
     * @return The segments that make up the path the Crease Defender should follow
     */
    static std::vector<Segment> getPathSegments(Field field);

    /**
     * Gets a point on the defender crease path where a ray from the goalie at the given
     * angle will intersect
     *
     * @param field The field the path is onAngle angle
     * @param goalie The goalie the defenders are working with
     * @param ball The ball
     * @param offset The angle to offset ray formed from the goalie to the ball by
     *
     * @return The point on the path the ray from the goalie intersects, if
     */
    static std::optional<Point> getPointOnCreasePath(Field field, Robot goalie, Ball ball,
                                                     Angle offset);

    // Tactic parameters
    Field field;
    Ball ball;
    Team friendly_team;
    Team enemy_team;
    LeftOrRight left_or_right;

    // How slow the ball must be moving for us to clear it from the defense area
    double BALL_SLOW_SPEED_THRESHOLD = 0.2;
};
