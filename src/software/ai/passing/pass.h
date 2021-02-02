#pragma once

#include <array>
#include <cstdlib>
#include <iostream>

#include "software/geom/point.h"
#include "software/time/timestamp.h"

/**
 * This class represents a Pass, with a given start position, end position,
 * speed, and start time
 */
class Pass
{
   public:
    Pass() = delete;

    /**
     * Create a pass with given parameters
     *
     * @param passer_point The point the pass should start at
     * @param receiver_point The point the receiver should be at to receive the pass
     * @param pass_speed_m_per_s The speed of the pass, in meters/second
     * @param pass_start_time The time that the pass should start at (ie. the time
     *                        that the ball should be kicked)
     */
    Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s,
         Timestamp pass_start_time);

    /**
     * Gets the value of the receiver point
     *
     * @return The value of the receiver point
     */
    Point receiverPoint() const;

    /**
     * Gets the angle the receiver should be facing to receive the pass
     *
     * @return The angle the receiver should be facing to receive the pass
     */
    Angle receiverOrientation() const;

    /**
     * Gets the angle the passer should be facing to perform the pass
     *
     * @return The angle the passer should be facing to perform the pass
     */
    Angle passerOrientation() const;

    /**
     * Gets the value of the passer point
     *
     * @return The value of the passer point
     */
    Point passerPoint() const;

    /**
     * Gets the value of the pass speed
     *
     * @return The value of the pass speed, in meters/second
     */
    double speed() const;

    /**
     * Gets the value of the pass start time
     *
     * @return The value of the pass start time
     */
    Timestamp startTime() const;

    /**
     * Estimate the time when the pass should be received
     *
     * This estimate does not account for friction on the ball
     *
     * @return An estimate of the time when the pass should be received
     */
    Timestamp estimateReceiveTime() const;

    /**
     * Estimate how long the pass will take, from kicking to receiving
     *
     * This estimate does not account for friction on the ball
     *
     * @return An estimate of how long the pass will take, from kicking to receiving
     */
    Duration estimatePassDuration() const;

    /**
     * Implement the "<<" operator for printing
     *
     * @param output_stream The stream to print to
     * @param pass The pass to print
     * @return The output stream with the string representation of the class appended
     */
    friend std::ostream& operator<<(std::ostream& output_stream, const Pass& pass);

   private:
    // The location of the receiver
    Point receiver_point;

    // The location of the passer
    Point passer_point;

    // The speed of the pass in meters/second
    double pass_speed_m_per_s;

    // The time to perform the pass at
    Timestamp pass_start_time;
};
