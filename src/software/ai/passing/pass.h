#pragma once

#include "shared/constants.h"
#include "software/time/timestamp.h"
#include "software/ai/passing/base_pass.h"

/**
 * This class represents a Pass, a receive point with a speed
 */
class Pass: public BasePass
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
     * Gets the value of the pass speed
     *
     * @return The value of the pass speed, in meters/second
     */
    double speed() const;    

    virtual Duration estimatePassDuration() const;

    virtual Duration estimateTimeToPoint(Point& point) const;

    virtual PassType type() const;

    friend std::ostream& operator<<(std::ostream& output_stream, const Pass& pass);

    virtual bool operator==(const Pass& other) const;

    private:

    // The speed of the pass in meters/second
    double pass_speed_m_per_s;
};
