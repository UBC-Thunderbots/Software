#pragma once

#include "ai/hl/stp/stp_hl.h"
#include "ai/navigator/rrt/rrt.h"
#include "ai/primitive/primitive.h"
#include "ai/world/world.h"
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
     *
     * @param world The initial state of the world for the AI
     */
    explicit AI(const World& world);

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
        const Timestamp& timestamp) const;

    /**
     * Updates the state of the ball in the AI's world with the new ball data
     *
     * @param new_ball_data A Ball containing new ball information
     */
    void updateWorldBallState(const Ball& new_ball_data);

    /**
     * Updates the state of the field in the AI's world with the new field data
     *
     * @param new_field_data A Field containing new field information
     */
    void updateWorldFieldState(const Field& new_field_data);

    /**
     * Given a message containing new information about the friendly team, updates
     * the state of the friendly team in the world
     *
     * @param new_friendly_team_msg The message containing new friendly team information
     */
    void updateWorldFriendlyTeamState(const Team& new_friendly_team_data);

    /**
     * Given a message containing new information about the enemy team, updates
     * the state of the enemy team in the world
     *
     * @param new_enemy_team_msg The message containing new enemy team information
     */
    void updateWorldEnemyTeamState(const Team& new_enemy_team_data);

    void updateWorldRefboxGameState(const RefboxGameState& game_state);


   private:
    World world;
    std::unique_ptr<HL> high_level;
    std::unique_ptr<Navigator> navigator;
};
