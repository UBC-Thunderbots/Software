#pragma once

#include <geom/point.h>
#include <util/refbox_constants.h>

/**
 * @brief Holds the state of the game according to the referee
 *
 * @details Contains information on what period of the game it is, what type of
 * play to run, team scores, time remaining, etc.  During normal gameplay, the
 * information in this class is received from the [ssl
 * refbox](https://github.com/RoboCup-SSL/ssl-refbox)
 * program over the network.
 *
 * Taken from
 */
class GameState
{
   public:
    enum State
    {
        HALT,    // Robots must not move
        STOP,    // Robots must stay 500mm away from ball
        SETUP,   // Robots not on starting team must stay 500mm away from ball
        READY,   // A robot on the starting team may kick the ball
        PLAYING  // Normal play
    };

    // Types of restarts
    enum Restart
    {
        NONE,
        KICKOFF,
        DIRECT,
        INDIRECT,
        PENALTY,
        BALL_PLACEMENT
    };

    State state;
    Restart restart;

    // True if our team can kick the ball during a restart
    bool ourRestart;

    // Scores
    int ourScore;
    int theirScore;

    // Time in seconds remaining in the current period
    int secondsRemaining;

    Point ballPlacementPoint;

    GameState()
    {
        state            = HALT;
        restart          = NONE;
        ourRestart       = false;
        ourScore         = 0;
        theirScore       = 0;
        secondsRemaining = 0;
    }

    void updateRefboxGameState(RefboxGameState gameState);

    // Rule queries

    bool halt() const;

    bool stopped() const;

    bool playing() const;

    bool kickoff() const;

    bool penalty() const;

    bool ballPlacement() const;

    bool isOurRestart() const;

    bool direct() const;

    bool indirect() const;

    bool ourKickoff() const;

    bool ourPenalty() const;

    bool ourDirect() const;

    bool ourIndirect() const;

    bool ourFreeKick() const;

    bool ourPlacement() const;

    bool theirKickoff() const;

    bool theirPenalty() const;

    bool theirDirect() const;

    bool theirIndirect() const;

    bool theirFreeKick() const;

    bool theirPlacement() const;

    // Robots must be in position for a restart
    bool setupRestart() const;

    bool inSetupState() const;

    bool inReadyState() const;

    // One of our robots can kick the ball
    bool canKick() const;

    // Our robots must stay 500mm away from the ball
    bool stayAwayFromBall() const;

    // Our robots must stay on our half of the field
    bool stayOnSide() const;

    // Our robots (except the penalty kicker) must stay 400mm behind the penalty
    // line
    bool stayBehindPenaltyLine() const;

    void setBallPlacementPoint(double x, double y);

    Point getBallPlacementPoint() const;
};
