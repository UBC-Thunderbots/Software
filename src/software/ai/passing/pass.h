#pragma once

#include <array>
#include <cstdlib>
#include <iostream>

#include "software/geom/point.h"
#include "software/time/timestamp.h"

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
     * @param receiver_point The point the receiver should be at to receive the pass
     * @param pass_speed_m_per_s The speed of the pass, in meters/second
     */
    Pass(Point receiver_point, double pass_speed_m_per_s);

    /**
     * Create a pass from the given pass array
     *
     * @param pass_array [receiver_point.x(), receiver_point.y(), pass_speed_m_per_s]
     * @return The Pass constructed from the pass array
     */
    static Pass fromPassArray(const std::array<double, 3>& pass_array);

    /**
     * Converts a pass to an array
     *
     * @returns the pass array: [receiver_point.x(), receiver_point.y(),
     * pass_speed_m_per_s]
     */
    std::array<double, 3> toPassArray() const;

    /**
     * Gets the value of the receiver point
     *
     * @return The value of the receiver point
     */
    Point receiverPoint() const;

    /**
     * Gets the value of the pass speed
     *
     * @return The value of the pass speed, in meters/second
     */
    double speed() const;

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

    // The speed of the pass in meters/second
    double pass_speed_m_per_s;

    // Store the pass as a pass array
    std::array<double, 3> pass_array;
};
