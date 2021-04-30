#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h"
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
 *                       |         ++         |
 *                       |         ++    <-- Goalie
 *+----------------------+---------++---------+------------------+
 */
class CreaseDefenderTactic : public Tactic
{
   public:
    /**
     * Creates a new CreaseDefenderTactic
     *
     * @param robot_navigation_obstacle_config The config
     */
    explicit CreaseDefenderTactic(std::shared_ptr<const RobotNavigationObstacleConfig>
                                      robot_navigation_obstacle_config);

    CreaseDefenderTactic() = delete;

    void updateWorldParams(const World &world) override;
    bool done() const override;

    /**
     * Update control params for this tactic
     *
     * @param enemy_threat_origin The origin of the enemy threat
     * @param alignment The alignment for this crease defender
     */
    void updateControlParams(const Point &enemy_threat_origin,
                             const CreaseDefenderAlignment &alignment);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot &robot, const World &world) const override;

    void accept(TacticVisitor &visitor) const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate &tactic_update) override;

    // Tactic parameters
    FSM<CreaseDefenderFSM> fsm;
    CreaseDefenderFSM::ControlParams control_params;
    std::shared_ptr<const RobotNavigationObstacleConfig> robot_navigation_obstacle_config;
};
