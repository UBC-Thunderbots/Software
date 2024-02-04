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

Pass Pass::fromPassArray(Point passer_point,
                         const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& array,
                         double pass_speed_m_per_s)
{
    return Pass(passer_point, Point(array.at(0), array.at(1)), pass_speed_m_per_s);
}

Pass Pass::fromDestReceiveSpeed(const Point& passer_point,
                                const Point& receiver_point,
                                double receive_speed_m_per_s,
                                double min_pass_speed_m_per_s,
                                double max_pass_speed_m_per_s)
{
    auto pass_speed_m_per_s = Pass::getPassSpeed(passer_point, receiver_point, receive_speed_m_per_s, min_pass_speed_m_per_s, max_pass_speed_m_per_s);

    return Pass(passer_point, receiver_point, pass_speed_m_per_s);
}

double Pass::getPassSpeed(const Point& ball_position,
                          const Point& pass_destination,
                          double dest_speed_m_per_s,
                          double min_pass_speed_m_per_s,
                          double max_pass_speed_m_per_s)
{
    // We have
    //      - destination speed (m/s)       -> vf
    //      - rolling deceleration (m/s^2)  -> r
    //      - sliding deceleration (m/s^2)  -> s
    //      - length of pass (m)            -> D
    //      - friction transition factor    -> c
    //          - this dictates at what speed friction goes from sliding to rolling
    // We want to find
    //      - initial starting speed (m/s) -> x
    //
    // Ball decelerates with sliding from x -> cx, then with rolling from cx -> vf
    // Slide Distance (m) -> d1
    // Roll Distance (m) -> d2
    // d2 = D - d1
    //
    // (cx)^2 = x^2 + 2sd1
    // d1 = (c^2 - 1)x^2 / 2s
    // d2 = D + (1 - c^2)x^2 / 2s
    //
    // vf^2 = (cx)^2 + 2rd2 = (cx)^2 + 2r(D + (1 - c^2)x^2 / 2s)
    // Simplify to get
    // x = sqrt((vf^2 - 2rD) / (c^2 - rc^2/s + b/s))
    Vector pass_distance          = Vector(pass_destination.x() - ball_position.x(),
                                  pass_destination.y() - ball_position.y());
    double pass_distance_length_m = pass_distance.length();

    // calculating the constant value used in determining pass speed
    double sq_friction_trans_factor = pow(FRICTION_TRANSITION_FACTOR, 2);
    double pass_speed_calc_constant =
        sq_friction_trans_factor -
        ((BALL_ROLLING_FRICTION_DECELERATION_METERS_PER_SECOND_SQUARED *
          sq_friction_trans_factor) /
         BALL_SLIDING_FRICTION_DECELERATION_METERS_PER_SECOND_SQUARED) +
        (BALL_ROLLING_FRICTION_DECELERATION_METERS_PER_SECOND_SQUARED /
         BALL_SLIDING_FRICTION_DECELERATION_METERS_PER_SECOND_SQUARED);

    double squared_pass_speed =
        (pow(dest_speed_m_per_s, 2) -
         2 * -BALL_ROLLING_FRICTION_DECELERATION_METERS_PER_SECOND_SQUARED *
             pass_distance_length_m) /
        pass_speed_calc_constant;
    double pass_speed_m_per_s = sqrt(squared_pass_speed);

    double clamped_pass_speed_m_per_s =
        std::max(min_pass_speed_m_per_s, std::min(max_pass_speed_m_per_s, pass_speed_m_per_s));
    return clamped_pass_speed_m_per_s;
}

std::array<double, NUM_PARAMS_TO_OPTIMIZE> Pass::toPassArray() const
{
    return {receiver_point.x(), receiver_point.y()};
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

bool Pass::operator==(const Pass& other) const
{
    return this->passer_point == other.passer_point &&
           this->receiver_point == other.receiver_point &&
           this->pass_speed_m_per_s == other.pass_speed_m_per_s;
}
