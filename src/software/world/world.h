#pragma once


#include <boost/circular_buffer.hpp>

#include "software/world/ball.h"
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
    World() = delete;

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
     * Creates a new world based on the TbotsProto::World protobuf representation.
     *
     * @param world_proto The TbotsProto::World protobuf which this world should be based
     * on
     */
    explicit World(const TbotsProto::World& world_proto);

    /**
     * Updates the state of the ball in the world with the new ball data
     *
     * @param new_ball A Ball containing new ball information
     */
    void updateBall(const Ball& new_ball);

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
     * Updates the referee command
     *
     * @param command the command sent by the referee
     */
    void updateRefereeCommand(const RefereeCommand& command);

    /**
     * Updates the referee command
     *
     * @param command the command sent by the referee
     * @param ball_placement_point ball placement point
     */
    void updateRefereeCommand(const RefereeCommand& command, Point ball_placement_point);

    /**
     * Updates the referee stage
     *
     * @param stage the stage sent by the referee
     */
    void updateRefereeStage(const RefereeStage& stage);

    /**
     * Returns a const reference to the Field in the world
     *
     * @return a const reference to the Field in the world
     */
    const Field& field() const;

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
    const Team& friendlyTeam() const;

    /**
     * Returns a const reference to the Enemy Team in the world
     *
     * @return a const reference to the Enemy Team in the world
     */
    const Team& enemyTeam() const;

    /**
     * Returns a const reference to the Game State
     *
     * @return a const reference to the Game State
     */
    const GameState& gameState() const;

    /**
     * Updates the current Game State
     *
     * @param game_state the game state to update with
     */
    void updateGameState(const GameState& game_state);

    /**
     * Updates the ball inside of game state
     *
     * @param ball the ball to update with
     */
    void updateGameStateBall(const Ball& ball);

    /**
     * Returns the current referee stage
     *
     * @return the current referee stage
     */
    const RefereeStage& getRefereeStage() const;

    /**
     * Returns the most recent timestamp value of all timestamped member
     * objects of the world
     *
     * @return the most recent timestamp value of all timestamped member
     * objects of the world
     */
    const Timestamp getMostRecentTimestamp() const;

    /**
     * Updates the last update timestamp
     *
     * @param timestamp The last time this World was updated
     *
     * @throws std::invalid_argument if timestamp is older than the current last update
     * timestamp
     */
    void updateTimestamp(Timestamp timestamp);

    /**
     * Sets the team with possession
     *
     * @param possession The team with possession
     */
    void setTeamWithPossession(TeamPossession possession);

    /**
     * Gets the team with possession
     *
     * @return The team with possession
     */
    TeamPossession getTeamWithPossession() const;

    /**
     * Defines the equality operator for a World. Worlds are equal if their field, ball
     * friendly_team, enemy_team and game_state are equal. The last update
     * timestamp and histories are not part of the equality.
     *
     * @param other The world to compare against for equality
     * @return True if the other robot is equal to this world, and false otherwise
     */
    bool operator==(const World& other) const;

    /**
     * Defines the inequality operator for a World.
     *
     * @param other The world to compare against for inequality
     * @return True if the other world is not equal to this world and false otherwise
     */
    bool operator!=(const World& other) const;

    // The size of the referee history buffers to filter out noise with
    static constexpr unsigned int REFEREE_COMMAND_BUFFER_SIZE = 3;

   private:
    /**
     * Searches all member objects of world for the most recent Timestamp value
     *
     */
    Timestamp getMostRecentTimestampFromMembers();

    Field field_;
    Ball ball_;
    Team friendly_team_;
    Team enemy_team_;
    GameState current_game_state_;
    RefereeStage current_referee_stage_;
    Timestamp last_update_timestamp_;
    // A small buffer that stores previous referee command
    boost::circular_buffer<RefereeCommand> referee_command_history_;
    // A small buffer that stores previous referee stage
    boost::circular_buffer<RefereeStage> referee_stage_history_;
    // which team has possession of the ball
    TeamPossession team_with_possession_;
};
