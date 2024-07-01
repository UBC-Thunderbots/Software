#pragma once

#include <array>
#include <cstdlib>
#include <iostream>

#include "software/geom/point.h"
#include "software/time/duration.h"


// The number of parameters (representing a pass) that we optimize
// (receive_location_x, receive_location_y)
static const int NUM_PARAMS_TO_OPTIMIZE = 2;

class BasePass 
{
    public:
    BasePass() = delete;

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
     * Gets the length of the pass in metres
     *
     * @return The length of the pass in metres
     */
    double length() const;

    /**
     * Estimate how long the pass will take, from kicking to receiving
     *
     * This estimate does not account for friction on the ball
     *
     * @return An estimate of how long the pass will take, from kicking to receiving
     */
    virtual Duration estimatePassDuration() const
    {
        return Duration::fromSeconds(0);
    }

    virtual Duration estimateTimeToPoint(Point& point) const;

    /**
     * Converts a pass to an array
     *
     * @returns the pass array: [receiver_point.x(), receiver_point.y()]
     */
    std::array<double, NUM_PARAMS_TO_OPTIMIZE> toPassArray() const;

   protected:
    /**
     * Create a pass with given parameters
     *
     * @param passer_point The point the pass should start at
     * @param receiver_point The point the receiver should be at to receive the pass
     */
    BasePass(Point passer_point, Point receiver_point);

    /**
    //  * Implement the "<<" operator for printing
    //  *
    //  * @param output_stream The stream to print to
    //  * @param pass The pass to print
    //  * @return The output stream with the string representation of the class appended
    //  */
    // friend std::ostream& printHelper(std::ostream& output_stream, const BasePass& pass);

    /**
     * Compares Passes for equality. Passes are considered
     * equal if all their member variables are equal.
     *
     * @param other the Pass to compare with for equality
     *
     * @return true if the Passes are equal and false otherwise
     */
    virtual bool operator==(const BasePass& other) const;

    // The location of the passer
    Point passer_point;

    // The location of the receiver
    Point receiver_point;
};