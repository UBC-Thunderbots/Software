#include "software/ai/passing/pass.h"


Pass::Pass(Point receiver_point, double pass_speed_m_per_s)
    : receiver_point(receiver_point),
      pass_speed_m_per_s(pass_speed_m_per_s),
      pass_array({receiver_point.x(), receiver_point.y(), pass_speed_m_per_s})
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

Angle Pass::receiverOrientation(const Point& ball_position) const
{
    return (ball_position - receiverPoint()).orientation();
}

double Pass::speed() const
{
    return pass_speed_m_per_s;
}

Pass Pass::fromPassArray(const std::array<double, 3>& array)
{
    return Pass(Point(array.at(0), array.at(1)), array.at(2));
}

std::array<double, 3> Pass::toPassArray() const
{
    return pass_array;
}

std::ostream& operator<<(std::ostream& output_stream, const Pass& pass)
{
    output_stream << "Receiver Point: " << pass.receiver_point
                  << " w/ Speed (m/s): " << pass.pass_speed_m_per_s;

    return output_stream;
}
