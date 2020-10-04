#include "software/ai/passing/pass.h"


Pass::Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s,
           Timestamp pass_start_time)
    : receiver_point(receiver_point),
      passer_point(passer_point),
      pass_speed_m_per_s(pass_speed_m_per_s),
      pass_start_time(pass_start_time)
{
    if (pass_speed_m_per_s < 0.0)
    {
        throw std::invalid_argument("Passes cannot have a negative pass speed");
    }
}

Point Pass::receiverPoint() const
{
    return receiver_point;
}

Angle Pass::receiverOrientation() const
{
    return (passerPoint() - receiverPoint()).orientation();
}

Angle Pass::passerOrientation() const
{
    return (receiverPoint() - passerPoint()).orientation();
}


Point Pass::passerPoint() const
{
    return passer_point;
}

double Pass::speed() const
{
    return pass_speed_m_per_s;
}

Timestamp Pass::startTime() const
{
    return pass_start_time;
}

Timestamp Pass::estimateReceiveTime() const
{
    return pass_start_time + estimatePassDuration();
}

Duration Pass::estimatePassDuration() const
{
    return Duration::fromSeconds((receiver_point - passer_point).length() /
                                 pass_speed_m_per_s);
}

std::ostream& operator<<(std::ostream& output_stream, const Pass& pass)
{
    output_stream << "Receiver: " << pass.receiver_point
                  << ", Passer: " << pass.passer_point
                  << " Speed (m/s): " << pass.pass_speed_m_per_s
                  << " Start Time (s): " << pass.pass_start_time.toSeconds();

    return output_stream;
}
