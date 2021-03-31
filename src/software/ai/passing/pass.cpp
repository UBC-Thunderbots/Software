#include "software/ai/passing/pass.h"


Pass::Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s)
    : passer_point(passer_point),
      receiver_point(receiver_point),
      pass_speed_m_per_s(pass_speed_m_per_s)
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

Pass Pass::fromPassArray(Point passer_point, const std::array<double, 3>& array)
{
    return Pass(passer_point, Point(array.at(0), array.at(1)), array.at(2));
}

std::array<double, 3> Pass::toPassArray() const
{
    return {receiver_point.x(), receiver_point.y(), pass_speed_m_per_s};
}

Duration Pass::estimatePassDuration() const
{
    return Duration::fromSeconds((receiver_point - passer_point).length() /
                                 pass_speed_m_per_s);
}

std::ostream& operator<<(std::ostream& output_stream, const Pass& pass)
{
    output_stream << "Receiver Point: " << pass.receiver_point
                  << " w/ Speed (m/s): " << pass.pass_speed_m_per_s;

    return output_stream;
}
