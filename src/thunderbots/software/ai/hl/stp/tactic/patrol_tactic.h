#pragma once

#include "ai/hl/stp/tactic/tactic.h"

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
     * @param dist_from_points How from from the current point in the patrol sequence the
     * robot must be before it moves on to the next point
     */
    explicit PatrolTactic(const std::vector<Point>& points, double dist_from_points);

    std::string getName() const override;

    /**
     * Updates the parameters for this PatrolTactic.
     *
     * @param orientation The orientation the robot should have while patrolling
     * @param final_speed The final speed the robot should aim to have at each patrol
     * point
     */
    void updateParams(Angle orientation, double final_speed);

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

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Tactic parameters
    // The list of points the robot will patrol, in order
    std::vector<Point> patrol_points;
    // How far the robot must be from each point before moving on to the next one
    double dist_from_points;
    // The orientation the robot should have when it arrives at its destination
    Angle orientation;
    // The final speed the robot will aim to have at each patrol point
    double final_speed;
    // The index of the patrol point currently being moved to
    unsigned int patrol_point_index;
};
