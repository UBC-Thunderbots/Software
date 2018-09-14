#pragma once

#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/team.h"

/**
 * The world object describes the entire state of the world, which for us is all the
 * information we have about
 * the field, robots, ball, and referee commands. The world object acts as a convenient
 * way to pass all this information
 * around to modules that may need it.
 */
class World final
{
   public:
    /**
     * Creates a new default world.
     */
    explicit World();

    /**
     * Creates a new world.
     *
     * @param field the field for the world
     * @param ball the ball for the world
     * @param friendly_team the friendly team for the world
     * @param enemy_team the enemy_team for the world
     */
    explicit World(const Field& field, const Ball& ball, const Team& friendly_team,
                   const Team& enemy_team);

    /**
     * Given a message containing new field geometry, update the geometry of the
     * Field in the world
     *
     * @param new_field_msg The message containing new field geometry
     */
    void updateFieldGeometry(const thunderbots_msgs::Field& new_field_msg);

    /**
     * Updates the state of the ball in the world with the new ball data
     *
     * @param new_ball_data A Ball containing new ball information
     */
    void updateBallState(const Ball& new_ball_data);

    /**
     * Given a message containing new information about the friendly team, updates
     * the state of the friendly team in the world
     *
     * @param new_friendly_team_msg The message containing new friendly team information
     */
    void updateFriendlyTeam(const thunderbots_msgs::Team& new_friendly_team_msg);

    /**
     * Given a message containing new information about the enemy team, updates
     * the state of the enemy team in the world
     *
     * @param new_enemy_team_msg The message containing new enemy team information
     */
    void updateEnemyTeam(const thunderbots_msgs::Team& new_enemy_team_msg);

    /**
     * Returns a const reference to the Field in the world
     *
     * @return a const reference to the Field in the world
     */
    const Field& field();

    /**
     * Returns a const reference to the Ball in the world
     *
     * @return a const reference to the Ball in the world
     */
    const Ball& ball() const;

    /**
     * Returns a const reference to the Friendly Team in the world
     *
     * @return a const reference to the Friendly Team in the world
     */
    const Team& friendly_team() const;

    /**
     * Returns a const reference to the Enemy Team in the world
     *
     * @return a const reference to the Enemy Team in the world
     */
    const Team& enemy_team() const;

   private:
    Field field_;
    Ball ball_;
    Team friendly_team_;
    Team enemy_team_;
};
