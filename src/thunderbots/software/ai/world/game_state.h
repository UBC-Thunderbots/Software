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
    bool our_restart;

    Point ballPlacementPoint;

    GameState()
    {
        state       = HALT;
        restart     = NONE;
        our_restart = false;
    }

    void updateRefboxGameState(RefboxGameState gameState);

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
    bool halt() const;

    /**
     * Returns true if robots must stay a set distance away from the ball.
     * See Robocup SSL Rules Law 5.5
     *
     * @return true if robots should stay away from the ball.
     */
    bool stopped() const;

    /**
     * Returns true if the game is in play.
     *
     * @return true if game is in play.
     */
    bool playing() const;

    /**
     * Returns true if a kickoff is occuring.
     * See Robocup SSL Rules Law 8.2.
     *
     * @return true if a kickoff is occuring.
     */
    bool kickoff() const;

    /**
     * Returns true if a penalty is occuring.
     * See Robocup SSL Rules Law 14.
     *
     * @return true if a penalty is occuring.
     */
    bool penalty() const;

    /**
     * Returns true if ball placement is occuring.
     * See Robocup SSL Rules Law 9.2.
     *
     * @return true if ball placement is occuring.
     */
    bool ballPlacement() const;

    /**
     * Returns true if our team is doing the restart, e.g. when
     * we get an indirect kick
     *
     * @return true if our team is doing the restart
     */
    bool ourRestart() const;

    /**
     * Returns true if a direct free kick is occuring.
     * See Robocup SSL Rules Law 13.
     *
     *
     * @return true if a direct free kick is occuring
     */
    bool direct() const;

    /**
     * Returns true if an indirect free kick is occuring.
     * See Robocup SSL Rules Law 13.
     *
     *
     * @return true if a direct free kick is occuring
     */
    bool indirect() const;

    /**
     * Returns true if we are taking the kickoff.
     *
     *
     * @return true if we are taking the kickoff.
     */
    bool ourKickoff() const;

    /**
     * Returns true if we are taking the penalty kick.
     *
     *
     * @return true if we are taking the penalty kick.
     */
    bool ourPenalty() const;

    /**
     * Returns true if we are taking a direct free kick.
     *
     *
     * @return true if we are taking a direct free kick.
     */
    bool ourDirect() const;

    /**
     * Returns true if we are taking an indirect free kick.
     *
     *
     * @return true if we are taking an indirect free kick.
     */
    bool ourIndirect() const;

    /**
     * Returns true if we are doing any type of free kick.
     *
     *
     * @return true if we are doing any type of free kick.
     */
    bool ourFreeKick() const;

    /**
     * Returns true if we are doing ball placement.
     *
     *
     * @return true if we are doing ball placement.
     */
    bool ourPlacement() const;

    /**
     * Returns true if opposing side is doing kickoff.
     *
     *
     * @return true if opposing side is doing kickoff.
     */
    bool theirKickoff() const;

    /**
     * Returns true if opposing side is taking a penalty kick.
     *
     *
     * @return true if opposing side is taking a penalty kick.
     */
    bool theirPenalty() const;

    /**
     * Returns true if opposing side is taking a direct free kick.
     *
     * @return true if opposing side is tkaing a direct free kick.
     */
    bool theirDirect() const;

    /**
     * Returns true if opposing side is taking an indirect free kick.
     *
     * @return true if opposing side is taking an indirect free kick.
     */
    bool theirIndirect() const;

    /**
     * Returns true if opposing side is taking any type of free kick.
     *
     * @return true if opposing side is taking any type of free kick.
     */
    bool theirFreeKick() const;

    /**
     * Returns true if opposing side is doing ball placement.
     *
     *
     * @return true if opposing side is doing ball placement.
     */
    bool theirPlacement() const;

    /**
     * Returns true if robots should be setting up for a restart e.g.
     * if a free kick is going to occur.
     *
     * @return true if robots should be setting up for a restart
     */
    bool setupRestart() const;

    /**
     * Returns true if we are currently getting ready for a kickoff or restart.
     * e.g. Getting ready to kick the ball for a free kick.
     *
     * @return true if we are currently getting ready for a kickoff or restart
     */
    bool setupState() const;
    /**
     * Returns true if we are ready for a kickoff or restart.
     *
     * @return  true if we are ready for a kickoff or restart.
     */
    bool readyState() const;

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
     * Sets the point on the field where the ball should be placed.
     * See Robocup SSL Rules Law 9.2.
     *
     * @param placementPoint the point where the ball should be placed.
     */
    void setBallPlacementPoint(Point placementPoint);

    /**
     * Returns the point where the ball should be placed.
     *
     * @return the point where the ball should be placed.
     */
    Point getBallPlacementPoint() const;
};
