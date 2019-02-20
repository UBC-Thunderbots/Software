/**
 * Declaration for the "Pass" Class
 */

#include <cstdlib>
#include <array>
#include "geom/point.h"
#include "util/timestamp.h"

namespace AI::Passing {

    // TODO: simple test for getters
    /**
     * This class represents a Pass, with a given start position, end position,
     * speed, and start time
     */
    class Pass {
    public:

        // The number of parameters that make up a pass
        static const size_t NUM_PARAMS = 4;

        /**
         * Creates a pass with all 0 values
         */
        Pass();

        /**
         * Create a pass with given parameters
         *
         * @param passer_point The point the pass should start at
         * @param receiver_point The point the receiver should be at to receive the pass
         * @param pass_speed_m_per_s The speed of the pass, in meters/second
         * @param pass_start_time The time that the pass should start at
         */
        Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s, Timestamp pass_start_time);

        /**
         * Returns a reference to the receiver point
         *
         * @return A reference to the receiver point
         */
        Point & receiverPoint();

        /**
         * Returns a reference to the passer point
         *
         * @return A reference to the passer point
         */
        Point & passerPoint();

        /**
         * Returns a reference to the pass speed, in meters per second
         *
         * @return A reference to the pass speed, in meters per second
         */
        double & passSpeed();

        /**
         * Returns a reference to the pass start time
         *
         * @return A reference to the pass start time
         */
        Timestamp & passStartTime();

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


}

