#pragma once

#include "ai/hl/stp/stp_hl.h"
#include "ai/navigator/rrt/rrt.h"
#include "ai/primitive/primitive.h"
#include "ai/world/world.h"
#include "thunderbots_msgs/Ball.h"
#include "thunderbots_msgs/Field.h"
#include "thunderbots_msgs/Team.h"
#include "util/timestamp.h"

/**
 * This class wraps all our AI logic and decision making to help separate our
 * logic from ROS communication as much as possible.
 */
class AI final
{
   public:
    /**
     * Creates a new AI
     */
    explicit AI();

    /**
     * Calculates the Primitives that should be run by our Robots given the current
     * state of the world.
     *
     * @param timestamp The timestamp at which this function call is being made
     *
     * @return the Primitives that should be run by our Robots given the current state
     * of the world.
     */
    std::vector<std::unique_ptr<Primitive>> getPrimitives(
        const AITimestamp& timestamp) const;

    /**
     * Given a message containing new ball state, updates the state of the ball
     * in the world
     *
     * @param new_ball_msg The message containing new ball information
     */
    void updateWorldBallState(const thunderbots_msgs::Ball& new_ball_msg);

    /**
     * Given a message containing new field geometry, update the geometry of the
     * Field in the world
     *
     * @param new_field_msg The message containing new field geometry
     */
    void updateWorldFieldState(const thunderbots_msgs::Field& new_field_msg);

    /**
     * Given a message containing new information about the friendly team, updates
     * the state of the friendly team in the world
     *
     * @param new_friendly_team_msg The message containing new friendly team information
     */
    void updateWorldFriendlyTeamState(
        const thunderbots_msgs::Team& new_friendly_team_msg);

    /**
     * Given a message containing new information about the enemy team, updates
     * the state of the enemy team in the world
     *
     * @param new_enemy_team_msg The message containing new enemy team information
     */
    void updateWorldEnemyTeamState(const thunderbots_msgs::Team& new_enemy_team_msg);


   private:
    World world;
    STP_HL stp_high_level;
    RRTNav rrt_navigator;
};
