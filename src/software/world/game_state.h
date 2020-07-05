#pragma once

#include "software/new_geom/point.h"
#include "software/sensor_fusion/refbox_data.h"
#include "software/world/ball.h"


/**
 * @brief Holds the state of the game according to the referee
 *
 * @details Contains information on what period of the game it is, what type of
 * play to run, team scores, time remaining, etc.  During normal gameplay, the
 * information in this class is received from the
 * [ssl-refbox](https://github.com/RoboCup-SSL/ssl-refbox)
 * program over the network.
 *
 * Taken from https://github.com/RoboJackets/robocup-software/
 */
class GameState
{
   public:
    enum State
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

    State state;
    RestartReason restart_reason;
    RefboxGameState game_state;
    std::optional<Ball> ball_state;

    // True if our team can kick the ball during a restart
    bool our_restart;

    // The point at which the ball should be placed by robots before a restart. See
    // Robocup SSL Rules 9.2.
    std::optional<Point> ball_placement_point;

    GameState()
        : state(HALT),
          restart_reason(NONE),
          game_state(RefboxGameState::HALT),
          ball_state(std::nullopt),
          our_restart(false),
          ball_placement_point(std::nullopt)
    {
    }

    /**
     * Updates the game state with a value from backend_input
     *
     * @param gameState the RefboxGameState from backend_input
     */
    void updateRefboxGameState(RefboxGameState gameState);

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
     * Returns the current Refbox game state
     *
     * @return the current Refbox game state
     */
    const RefboxGameState& getRefboxGameState() const;

    /**
     * Returns the current restart reason
     *
     * @return the current restart reason
     */
    RestartReason getRestartReason() const;

    /*
     * Rule queries
     *
     * See http://wiki.robocup.org/images/7/73/Small_Size_League_-_Rules_2018.pdf
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
     * HALT or STOP, because of how Refbox state transitions work.
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
     * Sets the point on the field where the ball should be placed.
     * See Robocup SSL Rules Law 9.2.
     *
     * @param placementPoint the point where the ball should be placed.
     */
    void setBallPlacementPoint(Point placementPoint);

    /**
     * Returns the point where the ball should be placed
     *
     * @return the point where the ball should be placed, or std::nullopt if we are not
     *         in a state of play where the ball needs to be placed
     */
    std::optional<Point> getBallPlacementPoint() const;
};
