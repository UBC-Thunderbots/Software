#pragma once

#include "software/geom/point.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/ball.h"

// clang-format off
MAKE_ENUM(RefereeCommand,
          // these enums map to the enums in ssl_referee.proto
          HALT,
          STOP,
          NORMAL_START,
          FORCE_START,
          PREPARE_KICKOFF_US,
          PREPARE_KICKOFF_THEM,
          PREPARE_PENALTY_US,
          PREPARE_PENALTY_THEM,
          DIRECT_FREE_US,
          DIRECT_FREE_THEM,
          INDIRECT_FREE_US,
          INDIRECT_FREE_THEM,
          TIMEOUT_US,
          TIMEOUT_THEM,
          GOAL_US,
          GOAL_THEM,
          BALL_PLACEMENT_US,
          BALL_PLACEMENT_THEM);
// clang-format on

MAKE_ENUM(RefereeStage,
          // The first half is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          NORMAL_FIRST_HALF_PRE,
          // The first half of the normal game, before half time.
          NORMAL_FIRST_HALF,
          // Half time between first and second halves.
          NORMAL_HALF_TIME,
          // The second half is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          NORMAL_SECOND_HALF_PRE,
          // The second half of the normal game, after half time.
          NORMAL_SECOND_HALF,
          // The break before extra time.
          EXTRA_TIME_BREAK,
          // The first half of extra time is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          EXTRA_FIRST_HALF_PRE,
          // The first half of extra time.
          EXTRA_FIRST_HALF,
          // Half time between first and second extra halves.
          EXTRA_HALF_TIME,
          // The second half of extra time is about to start.
          // A kickoff is called within this stage.
          // This stage ends with the NORMAL_START.
          EXTRA_SECOND_HALF_PRE,
          // The second half of extra time.
          EXTRA_SECOND_HALF,
          // The break before penalty shootout.
          PENALTY_SHOOTOUT_BREAK,
          // The penalty shootout.
          PENALTY_SHOOTOUT,
          // The game is over.
          POST_GAME);

/**
 * @brief Holds the state of the game according to the referee
 *
 * @details Contains information on what period of the game it is, what type of
 * play to run, team scores, time remaining, etc.  During normal gameplay, the
 * information in this class is received from the
 * [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller)
 * program over the network.
 *
 * Taken from https://github.com/RoboJackets/robocup-software/
 */
class GameState
{
   public:
    enum PlayState
    {
        HALT,    // Robots must not move
        STOP,    // Robots must stay a set distance away from ball
        SETUP,   // Robots not on starting team must stay a set distance away from ball
        READY,   // A robot on the starting team may kick the ball
        PLAYING  // Normal play
    };

    // Types of restarts
    enum RestartReason
    {
        NONE,
        KICKOFF,
        DIRECT,
        INDIRECT,
        PENALTY,
        BALL_PLACEMENT
    };

    GameState()
        : play_state_(HALT),
          restart_reason_(NONE),
          command_(RefereeCommand::HALT),
          ball_(std::nullopt),
          our_restart_(false),
          ball_placement_point_(std::nullopt)
    {
    }

    /**
     * Creates a new game state based on the TbotsProto::GameState protobuf representation
     *
     * @param game_state_proto The TbotsProto::GameState protobuf which this game state
     * should be based on
     */
    explicit GameState(const TbotsProto::GameState& game_state_proto);

    /**
     * Updates the game state with a value from backend_input
     *
     * @param gameState the RefereeCommand from backend_input
     */
    void updateRefereeCommand(RefereeCommand gameState);

    /**
     * Updates the state of the ball used in calculating game state transitions
     *
     * @param ball The new ball
     */
    void updateBall(const Ball& ball);

    /**
     * Clears restart state and enters normal play. Should be
     * called when state transitions from a restart setup state
     * such as PREPARE_KICKOFF_FRIENDLY into NORMAL_START.
     */
    void setRestartCompleted();

    /**
     * Returns the current Referee command
     *
     * @return the current Referee command
     */
    const RefereeCommand& getRefereeCommand() const;

    /**
     * Returns the current restart reason
     *
     * @return the current restart reason
     */
    RestartReason getRestartReason() const;

    /*
     * Rule queries
     *
     * See https://download.tigers-mannheim.de/rules/2018_ssl-rules.pdf
     * for more details
     */

    /**
     * Returns true if robots may not move.
     *
     * @return true if robots should halt.
     */
    bool isHalted() const;

    /**
     * Returns true if robots must stay a set distance away from the ball.
     * See Robocup SSL Rules Law 5.5
     *
     * @return true if robots should stay away from the ball.
     */
    bool isStopped() const;

    /**
     * Returns true if the game is in play.
     *
     * @return true if game is in play.
     */
    bool isPlaying() const;

    /**
     * Returns true if a kickoff is occurring.
     * See Robocup SSL Rules Law 8.2.
     *
     * @return true if a kickoff is occurring.
     */
    bool isKickoff() const;

    /**
     * Returns true if a penalty is occurring.
     * See Robocup SSL Rules Law 14.
     *
     * @return true if a penalty is occurring.
     */
    bool isPenalty() const;

    /**
     * Returns true if ball placement is occurring.
     * See Robocup SSL Rules Law 9.2.
     *
     * @return true if ball placement is occurring.
     */
    bool isBallPlacement() const;

    /**
     * Returns true if our team is doing the restart, e.g. when
     * we get an indirect kick
     * This function will have undefined behaviour if a restart is
     * not being prepared for or occurring, such as after a transition to
     * HALT or STOP, because of how GameController state transitions work.
     *
     * @return true if our team is doing the restart
     */
    bool isOurRestart() const;

    /**
     * Returns true if a direct free kick is occurring.
     * See Robocup SSL Rules Law 13.
     *
     *
     * @return true if a direct free kick is occurring
     */
    bool isDirectFree() const;

    /**
     * Returns true if an indirect free kick is occurring.
     * See Robocup SSL Rules Law 13.
     *
     *
     * @return true if a direct free kick is occurring
     */
    bool isIndirectFree() const;

    /**
     * Returns true if we are taking the kickoff.
     *
     *
     * @return true if we are taking the kickoff.
     */
    bool isOurKickoff() const;

    /**
     * Returns true if we are taking the penalty kick.
     *
     *
     * @return true if we are taking the penalty kick.
     */
    bool isOurPenalty() const;

    /**
     * Returns true if we are taking a direct free kick.
     *
     *
     * @return true if we are taking a direct free kick.
     */
    bool isOurDirectFree() const;

    /**
     * Returns true if we are taking an indirect free kick.
     *
     *
     * @return true if we are taking an indirect free kick.
     */
    bool isOurIndirectFree() const;

    /**
     * Returns true if we are doing any type of free kick.
     *
     *
     * @return true if we are doing any type of free kick.
     */
    bool isOurFreeKick() const;

    /**
     * Returns true if we are doing ball placement.
     *
     *
     * @return true if we are doing ball placement.
     */
    bool isOurBallPlacement() const;

    /**
     * Returns true if opposing side is doing kickoff.
     *
     *
     * @return true if opposing side is doing kickoff.
     */
    bool isTheirKickoff() const;

    /**
     * Returns true if opposing side is taking a penalty kick.
     *
     *
     * @return true if opposing side is taking a penalty kick.
     */
    bool isTheirPenalty() const;

    /**
     * Returns true if opposing side is taking a direct free kick.
     *
     * @return true if opposing side is taking a direct free kick.
     */
    bool isTheirDirectFree() const;

    /**
     * Returns true if opposing side is taking an indirect free kick.
     *
     * @return true if opposing side is taking an indirect free kick.
     */
    bool isTheirIndirectFree() const;

    /**
     * Returns true if opposing side is taking any type of free kick.
     *
     * @return true if opposing side is taking any type of free kick.
     */
    bool isTheirFreeKick() const;

    /**
     * Returns true if opposing side is doing ball placement.
     *
     *
     * @return true if opposing side is doing ball placement.
     */
    bool isTheirBallPlacement() const;

    /**
     * Returns the play state
     *
     * @return play state
     */
    PlayState getPlayState(void) const;

    /**
     * Returns true if robots should be setting up for a restart e.g.
     * if a free kick is going to occur.
     *
     * @return true if robots should be setting up for a restart
     */
    bool isSetupRestart() const;

    /**
     * Returns true if we are currently getting ready for a kickoff or restart.
     * e.g. Getting ready to kick the ball for a free kick.
     *
     * @return true if we are currently getting ready for a kickoff or restart
     */
    bool isSetupState() const;
    /**
     * Returns true if we are ready for a kickoff or restart.
     *
     * @return  true if we are ready for a kickoff or restart.
     */
    bool isReadyState() const;

    /**
     * Returns true if we can kick the ball.
     *
     * @return  true if we can kick the ball.
     */
    bool canKick() const;

    /**
     * Returns true if we should stay a set distance from the ball.
     * See Robocup SSL Rules Law 13.4, 5.5, 8.2.1, 9.2
     *
     * @return  true if robots should stay a set distance from the ball.
     */
    bool stayAwayFromBall() const;

    /**
     * Returns true if robots should stay on their team's side of the field.
     * See Robocup SSL Rules Law 8.2.1.
     *
     * @return  true if robots should stay on their team's side of the field.
     */
    bool stayOnSide() const;

    /**
     * Returns true if our robots except the penalty kicker should stay on
     * our side of the field.
     *
     * @return  true if our robots except the penalty kicker should stay on
     *          our side of the field.
     */
    bool stayBehindPenaltyLine() const;

    /**
     * Defines the equality operator for a GameState
     *
     * @param other The GameState to compare against for equality
     * @return True if the other GameState is equal to this GameState and false otherwise
     */
    bool operator==(const GameState& other) const;

    /**
     * Defines the inequality operator for a GameState
     *
     * @param other The GameState to compare against for equality
     * @return True if the other GameState is not equal to this GameState and true
     * otherwise
     */
    bool operator!=(const GameState& other) const;

    /**
     * Returns the point on the field where the ball should be placed if one is specified.
     *
     * @return the requested ball placement position on the field if one is specified,
     * otherwise std::nullopt
     */
    std::optional<Point> getBallPlacementPoint(void) const;

    /**
     * Returns the ball
     *
     * @return the ball state if one is specified, otherwise std::nullopt
     */
    std::optional<Ball> getBall(void) const;

    /**
     * Sets the point on the field where the ball should be placed.
     * See Robocup SSL Rules Law 9.2.
     *
     * @param placementPoint the point where the ball should be placed.
     */
    void setBallPlacementPoint(Point placementPoint);

   private:
    PlayState play_state_;
    RestartReason restart_reason_;
    RefereeCommand command_;
    std::optional<Ball> ball_;

    // True if our team can kick the ball during a restart
    bool our_restart_;

    // The point at which the ball should be placed by robots before a restart. See
    // Robocup SSL Rules 9.2.
    std::optional<Point> ball_placement_point_;
};
