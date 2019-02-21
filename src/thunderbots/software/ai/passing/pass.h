/**
 * Declaration for the "Pass" Class
 */

#include <array>
#include <cstdlib>
#include <iostream>

#include "geom/point.h"
#include "util/timestamp.h"

namespace AI::Passing
{
    // TODO: simple test for getters
    /**
     * This class represents a Pass, with a given start position, end position,
     * speed, and start time
     */
    class Pass
    {
       public:
        /**
         * Creates a pass with all 0 values
         */
        // Pass();

        /**
         * Create a pass with given parameters
         *
         * @param passer_point The point the pass should start at
         * @param receiver_point The point the receiver should be at to receive the pass
         * @param pass_speed_m_per_s The speed of the pass, in meters/second
         * @param pass_start_time The time that the pass should start at
         */
        Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s,
             Timestamp pass_start_time);

        /**
         * Gets the value of the receiver point
         *
         * @return The value of the receiver point
         */
        Point receiverPoint();

        /**
         * Gets the value of the passer point
         *
         * @return The value of the passer point
         */
        Point passerPoint();

        /**
         * Gets the value of the pass speed
         *
         * @return The value of the pass speed, in meters/second
         */
        double speed();

        /**
         * Gets the value of the pass start time
         *
         * @return The value of the pass start time
         */
        Timestamp startTime();

        /**
         * Implement the "<<" operator for printing
         *
         * This is implemented in the header because there were namespace issues with
         * putting it in the `.cpp`.
         *
         * @param output_stream The stream to print to
         * @param pass The pass to print
         * @return The output stream with the string representation of the class appended
         */
        friend std::ostream& operator<<(std::ostream& output_stream, const Pass& pass);
        //{
        //    output_stream << "Receiver: " << pass.receiver_point
        //                  << ", Passer: " << pass.passer_point
        //                  << " Speed (m/s): " << pass.pass_speed_m_per_s
        //                  << " Start Time (s): " << pass.pass_start_time.getSeconds();

        //    return output_stream;
        //}

       private:
        // The location of the passer
        Point passer_point;

        // The location of the receiver
        Point receiver_point;

        // The speed of the pass in meters/second
        double pass_speed_m_per_s;

        // The time to preform the pass at
        Timestamp pass_start_time;
    };


}  // namespace AI::Passing
