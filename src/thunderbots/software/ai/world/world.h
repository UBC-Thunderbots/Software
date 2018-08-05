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
    explicit World(
        const Field &field, const Ball &ball, const Team &friendly_team,
        const Team &enemy_team);


    // We leave these members public to make them easy to access and update, rather
    // than providing redundant getters and setters.
    Field field;
    Ball ball;
    Team friendly_team;
    Team enemy_team;
};
