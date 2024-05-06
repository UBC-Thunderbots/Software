#pragma once

#include <array>
#include <cstdlib>
#include <iostream>

#include "shared/constants.h"
#include "software/geom/point.h"
#include "software/time/timestamp.h"

// The number of parameters (representing a pass) that we optimize
// (receive_location_x, receive_location_y)
static const int NUM_PARAMS_TO_OPTIMIZE = 2;

/**
 * This class represents a Pass, a receive point with a speed
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
     */
    Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s);

    /**
     * Create a pass from the given pass array
     *
     * @param passer_point The location of the passer location
     * @param pass_array [receiver_point.x(), receiver_point.y()]
     * @param pass_speed_m_per_s the speed of the pass in m/s
     * @return The Pass constructed from the pass array
     */
    static Pass fromPassArray(
        Point passer_point, const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array,
        double pass_speed_m_per_s);


    /**
     * Creates a pass from the given destination point and receive speed
     *
     * @param ball_position the current position of the ball (pass starting point)
     * @param pass_destination the end point of the pass
     * @param dest_speed_m_per_s the speed we want the pass to be received at
     * @param min_pass_speed_m_per_s the minimum speed a pass should be taken at
     * @param max_pass_speed_m_per_s the maximum speed a pass should be taken at
     * @return the Pass constructed from the start and end points, received at the
     intended speed clamped by the min and max pass speeds
     */
    static Pass fromDestReceiveSpeed(const Point& ball_position,
                                     const Point& pass_destination,
                                     double dest_speed_m_per_s,
                                     double min_pass_speed_m_per_s,
                                     double max_pass_speed_m_per_s);

    /**
     * Determines the speed at which a pass should be executed
     * Such that it reaches its destination at the given destination speed
     * Takes into account friction
     * Clamps the speed to the given mix and max speed values
     * @param ball_position the current ball position (starting point of the pass)
     * @param pass_destination the destination of the pass
     * @param receive_speed_m_per_s the speed we want the pass to be received at
     * @param min_pass_speed_m_per_s the minimum speed a pass should be taken at
     * @param max_pass_speed_m_per_s the maximum speed a pass should be taken at
     * @return the speed the pass should start with
     */
    static double getPassSpeed(const Point& ball_position, const Point& pass_destination,
                               double receive_speed_m_per_s,
                               double min_pass_speed_m_per_s,
                               double max_pass_speed_m_per_s);

    /**
     * Converts a pass to an array
     *
     * @returns the pass array: [receiver_point.x(), receiver_point.y()]
     */
    std::array<double, NUM_PARAMS_TO_OPTIMIZE> toPassArray() const;

    /**
     * Gets the value of the passer point
     *
     * @return The value of the passer point
     */
    Point passerPoint() const;

    /**
     * Gets the value of the receiver point
     *
     * @return The value of the receiver point
     */
    Point receiverPoint() const;

    /**
     * Given the ball position, returns the angle the receiver should be
     * facing to receive the pass.
     *
     * @return The angle the receiver should be facing
     */
    Angle receiverOrientation() const;

    /**
     * Given the ball position, returns the angle the passer should be
     * facing to pass.
     *
     * @return The angle the passer should be facing
     */
    Angle passerOrientation() const;

    /**
     * Gets the value of the pass speed
     *
     * @return The value of the pass speed, in meters/second
     */
    double speed() const;

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

    /**
     * Compares Passes for equality. Passes are considered
     * equal if all their member variables are equal.
     *
     * @param other the Pass to compare with for equality
     *
     * @return true if the Passes are equal and false otherwise
     */
    bool operator==(const Pass& other) const;

   private:
    // The location of the passer
    Point passer_point;

    // The location of the receiver
    Point receiver_point;

    // The speed of the pass in meters/second
    double pass_speed_m_per_s;
};
