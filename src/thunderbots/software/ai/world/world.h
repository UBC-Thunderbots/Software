#pragma once

#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/game_state.h"
#include "ai/world/team.h"
#include "util/refbox_constants.h"

/**
 * The world object describes the entire state of the world, which for us is all the
 * information we have about the field, robots, and ball. The world object acts as a
 * convenient way to pass all this information around to modules that may need it.
 */
class World final
{
   public:
    /**
     * Creates an Empty World
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
     * Updates the state of the field in the world with the new field data
     *
     * @param new_field_data A Field containing new field information
     */
    void updateFieldGeometry(const Field& new_field_data);

    /**
     * Updates the state of the ball in the world with the new ball data
     *
     * @param new_ball_data A Ball containing new ball information
     */
    void updateBallState(const Ball& new_ball_data);

    /**
     * Updates the state of the friendly team in the world with the new team data
     *
     * @param new_friendly_team_msg The message containing new friendly team information
     */
    void updateFriendlyTeamState(const Team& new_friendly_team_data);

    /**
     * Updates the state of the enemy team in the world with the new team data
     *
     * @param new_enemy_team_msg The message containing new enemy team information
     */
    void updateEnemyTeamState(const Team& new_enemy_team_data);

    /**
     * Updates the refbox game state
     *
     * @param game_state the game state sent by refbox
     */
    void updateRefboxGameState(const RefboxGameState& game_state);

    /**
     * Returns a const reference to the Field in the world
     *
     * @return a const reference to the Field in the world
     */
    const Field& field() const;

    /**
     * Returns a mutable reference to the Field in the world
     *
     * @return a mutable reference to the Field in the world
     */
    Field& mutableField();

    /**
     * Returns a const reference to the Ball in the world
     *
     * @return a const reference to the Ball in the world
     */
    const Ball& ball() const;

    /**
     * Returns a mutable reference to the Ball in the world
     *
     * @return a mutable reference to the Ball in the world
     */
    Ball& mutableBall();

    /**
     * Returns a const reference to the Friendly Team in the world
     *
     * @return a const reference to the Friendly Team in the world
     */
    const Team& friendlyTeam() const;

    /**
     * Returns a mutable reference to the Friendly Team in the world
     *
     * @return a mutable reference to the Friendly Team in the world
     */
    Team& mutableFriendlyTeam();

    /**
     * Returns a const reference to the Enemy Team in the world
     *
     * @return a const reference to the Enemy Team in the world
     */
    const Team& enemyTeam() const;

    /**
     * Returns a mutable reference to the Enemy Team in the world
     *
     * @return a mutable reference to the Enemy Team in the world
     */
    Team& mutableEnemyTeam();

    /**
     * Returns a const reference to the Game State
     *
     * @return a const reference to the Game State
     */
    const GameState& gameState() const;

    /**
     * Returns a mutable reference to the Game State
     *
     * @return a mutable reference to the Game State
     */
    GameState& mutableGameState();

   private:
    Field field_;
    Ball ball_;
    Team friendly_team_;
    Team enemy_team_;
    GameState game_state_;
};
