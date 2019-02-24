/**
 * Implementation of the "Pass" class
 */

#include "ai/passing/pass.h"

using namespace AI::Passing;

Pass::Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s,
           Timestamp pass_start_time)
    : receiver_point(receiver_point),
      passer_point(passer_point),
      pass_speed_m_per_s(pass_speed_m_per_s),
      pass_start_time(pass_start_time)
{
}

Point Pass::receiverPoint()
{
    return receiver_point;
}

Point Pass::passerPoint()
{
    return passer_point;
}

double Pass::speed()
{
    return pass_speed_m_per_s;
}

Timestamp Pass::startTime()
{
    return pass_start_time;
}

namespace AI::Passing
{
    std::ostream& operator<<(std::ostream& output_stream, const Pass& pass)
    {
        output_stream << "Receiver: " << pass.receiver_point
                      << ", Passer: " << pass.passer_point
                      << " Speed (m/s): " << pass.pass_speed_m_per_s
                      << " Start Time (s): " << pass.pass_start_time.getSeconds();

        return output_stream;
    }
}  // namespace AI::Passing
