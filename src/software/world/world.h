#pragma once


#include <boost/circular_buffer.hpp>

#include "software/sensor_fusion/refbox_data.h"
#include "software/world/ball.h"
#include "software/world/ball_state.h"
#include "software/world/field.h"
#include "software/world/game_state.h"
#include "software/world/team.h"

/**
 * The world object describes the entire state of the world, which for us is all the
 * information we have about the field, robots, and ball. The world object acts as a
 * convenient way to pass all this information around to modules that may need it.
 *
 * WARNING: This class should _never_ hold any data that is pointed to anywhere else. This
 * means no raw pointers, no shared pointers, no shared references. This is because in
 * some cases we copy World's over multiple threads where having a shared member
 * between multiple instances of a World on multiple threads could result in
 * data corruption that would lead to absurdly hard to track bugs. YOU HAVE BEEN WARNED.
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
                   const Team& enemy_team, unsigned int buffer_size = 20);

    /**
     * Updates the state of the field in the world with the new field data
     *
     * @param new_field_data A Field containing new field information
     */
    void updateFieldGeometry(const Field& new_field_data);

    /**
     * Updates the state of the ball in the world with the new ball data
     *
     * @param new_ball_data A BallState containing new ball information
     */
    void updateBallState(const BallState& new_ball_state);

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



    /**
     * Gets the most recent Timestamp stored in the history of the World
     *
     * @return returns Timestamp : The most recent Timestamp stored in the history
     */
    const Timestamp getMostRecentTimestamp() const;

    /**
     * Gets the update Timestamp history stored World
     *
     * @return returns circular_buffer<Timestamp> : The Timestamp history stored in the
     * World
     */
    boost::circular_buffer<Timestamp> getTimestampHistory();

    /**
     * Searches all member objects of world for the most recent Timestamp value
     *
     */
    Timestamp getMostRecentTimestampFromMembers();

    /**
     * Updates the timestamp history for World to include a new timestamp (On the
     * condition that the parameter Timestamp is newer than any Timestamp already in the
     * history)
     *
     * @param Timestamp corresponding to when the World was last updated
     */
    void updateTimestamp(Timestamp timestamp);

   private:
    Field field_;
    Ball ball_;
    Team friendly_team_;
    Team enemy_team_;
    GameState game_state_;
    // All previous timestamps of when the world was updated, with the most recent
    // timestamp at the front of the queue,
    boost::circular_buffer<Timestamp> last_update_timestamps;
    // A small buffer that stores previous refbox game state
    boost::circular_buffer<RefboxGameState> refbox_game_state_history;
};
