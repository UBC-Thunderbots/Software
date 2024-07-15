#include "software/ai/passing/chip_pass.h"

ChipPass::ChipPass(Point passer_point, Point receiver_point)
    : BasePass(passer_point, receiver_point),
      skip_area({Point(0, 0), Point(0, 0), Point(0, 0), Point(0, 0)})
{   
    pass_length = length();
    first_bounce_range_m = calcFirstBounceRange();
    skip_area = calcSkipArea(); 
    std::cout << "SKIP: " << skip_area << std::endl;
}

bool ChipPass::isSkipped(const Point& point) const
{
    return contains(skip_area, point);
}

Polygon ChipPass::calcSkipArea()
{
    std::vector<std::pair<double, double>>pass_info (bounce_heights_and_ranges.rbegin(), bounce_heights_and_ranges.rend());
    double distance_until_intercept = 0;
    for (auto &height_and_range: pass_info)
    {
        std::cout << "HEIGHT: " << height_and_range.first << " RANGE: " << height_and_range.second << std::endl;
        if (height_and_range.first < ROBOT_MAX_HEIGHT_METERS)
        {
            break;
        }

        distance_until_intercept += height_and_range.second;
    }

    Vector pass_direction(receiver_point.x() - passer_point.x(), receiver_point.y() - passer_point.y());

    Vector vec_on_pass = pass_direction.normalize(distance_until_intercept);
    Point point_on_pass(passer_point.x() + vec_on_pass.x(), passer_point.y() + vec_on_pass.y());

    Vector perpendicular = pass_direction.perpendicular().normalize(1.5);
    Vector perpendicular_opp = perpendicular.rotate(Angle::fromDegrees(180));

    return Polygon({
        Point(passer_point.x() + perpendicular.x(), passer_point.y() + perpendicular.y()),
        Point(passer_point.x() + perpendicular_opp.x(), passer_point.y() + perpendicular_opp.y()),
        Point(point_on_pass.x() + perpendicular.x(), point_on_pass.y() + perpendicular.y()),
        Point(point_on_pass.x() + perpendicular_opp.x(), point_on_pass.y() + perpendicular_opp.y())
    });
}

double ChipPass::calcFirstBounceRange() 
{
    double last_bounce_range = 0.0;
    double length_to_go = length();

    // if we cover more than 90% of the pass, stop calculating
    while(length_to_go >= pass_length * 0.2)
    {
        double bounce_height = getBounceHeightFromDistanceTraveled(length_to_go);
        last_bounce_range = getBounceRangeFromBounceHeight(bounce_height);
        bounce_heights_and_ranges.push_back(std::make_pair(bounce_height, last_bounce_range));

        std::cout << "HEIGHT: " << bounce_height << " RANGE: " << last_bounce_range << std::endl;
        length_to_go -= last_bounce_range;
    }

    return last_bounce_range;
}

double ChipPass::getBounceHeightFromDistanceTraveled(double distance_traveled)
{
    return 2 * std::exp(-1.61 * (distance_traveled + 2.82 - pass_length));
}

double ChipPass::getBounceRangeFromBounceHeight(double bounce_height)
{
    return 4 * bounce_height * (std::cos(ROBOT_CHIP_ANGLE_RADIANS) / std::sin(ROBOT_CHIP_ANGLE_RADIANS));
}

double ChipPass::firstBounceRange()
{
    return first_bounce_range_m;
}

Duration ChipPass::estimatePassDuration() const
{
    return Duration::fromSeconds(0);
}

Duration ChipPass::estimateTimeToPoint(Point& point) const
{
    return Duration::fromSeconds(0);
}

PassType Chipass::type() const
{
    return PassType::CHIP_PASS;
}

std::ostream& operator<<(std::ostream& output_stream, const ChipPass& pass)
{
   output_stream << "Pass from " << pass.passer_point 
                  << " to: " << pass.receiver_point
                  << " w/ First Bounce Range (m/s): " << pass.first_bounce_range_m;

    return output_stream;
}

bool ChipPass::operator==(const ChipPass& other) const
{
    return BasePass::operator==(other) &&
           this->first_bounce_range_m == other.first_bounce_range_m;
}