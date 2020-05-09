#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The PatrolTactic will make the robot continuously patrol through the positions it's
 * given
 */
class PatrolTactic : public Tactic
{
   public:
    /**
     * Creates a new PatrolTactic
     *
     * @param points The sequence of points to patrol, in order
     * @param at_patrol_point_tolerance How from the current point in the
     * patrol sequence the robot must be before it moves on to the next point
     * @param linear_speed_at_patrol_points The desired linear speed
     * of the robot at each patrol point
     * @param orientation_at_patrol_points The orientation the robot
     * should have while patrolling
     */
    explicit PatrolTactic(const std::vector<Point>& points,
                          double at_patrol_point_tolerance,
                          Angle orientation_at_patrol_points,
                          double linear_speed_at_patrol_points);

    std::string getName() const override;

    /**
     * Calculates the cost of assigning the given robot to this Tactic. If a robot is
     * already assigned to this tactic, that robot is preferred to be assigned again.
     * Otherwise, prefers robots closer to the current patrol point being moved to.
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

    void accept(MutableTacticVisitor& visitor) override;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;

    // Tactic parameters
    // The list of points the robot will patrol, in order
    std::vector<Point> patrol_points;
    // How far the robot must be from each patrol point before moving on to the next one
    double at_patrol_point_tolerance;
    // The orientation the robot should have when it arrives at its destination
    Angle orientation_at_patrol_points;
    // The final speed the robot will aim to have at each patrol point. Combined with the
    // 'at_patrol_point_tolerance', this affect whether the robot will stop at each patrol
    // point, or maintain speed and roughly drive through them
    double linear_speed_at_patrol_points;
    // The desired linear speed of the robot at each patrol point
    unsigned int patrol_point_index;
};
