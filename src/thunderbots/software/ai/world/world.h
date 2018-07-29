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
class World
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
    explicit World(
        const Field &field, const Ball &ball, const Team &friendly_team,
        const Team &enemy_team);

    /**
     * Updates the field in this world.
     *
     * @param new_field the new field
     */
    void updateField(const Field &new_field);

    /**
     * Updates the ball in this world.
     *
     * @param new_ball the new ball
     */
    void updateBall(const Ball &new_ball);

    /**
     * Updates the friendly team in this world.
     *
     * @param new_friendly_team the friendly team
     */
    void updateFriendlyTeam(const Team &new_friendly_team);

    /**
     * Updates the enemy team in this world.
     *
     * @param new_enemy_team the enemy team
     */
    void updateEnemyTeam(const Team &new_enemy_team);

    /**
     * Returns the ball in this world.
     *
     * @return the ball in this world.
     */
    Ball ball() const;

    /**
     * Returns the ball in this world.
     *
     * @return the ball in this world.
     */
    Field field() const;

    /**
     * Returns the friendly team in this world.
     *
     * @return the friendly team in this world.
     */
    Team friendlyTeam() const;

    /**
     * Returns the enemy team in this world.
     *
     * @return the enemy team in this world.
     */
    Team enemyTeam() const;

   private:
    Field field_;
    Ball ball_;
    Team friendly_team_;
    Team enemy_team_;
};
