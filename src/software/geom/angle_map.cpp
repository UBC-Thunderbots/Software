#include "software/geom/angle_map.h"

AngleMap::AngleMap(Angle top_angle, Angle bottom_angle, size_t max_num_obstacles)
    : AngleMap(AngleSegment(top_angle, bottom_angle), max_num_obstacles)
{
}

AngleMap::AngleMap(AngleSegment angle_seg, size_t max_num_obstacles)
    : angle_seg(angle_seg)
{
    this->taken_angle_segments.reserve(max_num_obstacles);
}

const AngleSegment &AngleMap::getAngleSegment() const
{
    return this->angle_seg;
}

void AngleMap::addNonViableAngleSegment(AngleSegment &angle_seg)
{
    for (AngleSegment &taken_angle_seg : taken_angle_segments)
    {
        if (!(angle_seg.getAngleBottom() > taken_angle_seg.getAngleTop() ||
              angle_seg.getAngleTop() < taken_angle_seg.getAngleBottom()))
        {
            taken_angle_seg.setAngleTop(
                std::max(taken_angle_seg.getAngleTop(), angle_seg.getAngleTop()));
            taken_angle_seg.setAngleBottom(
                std::min(taken_angle_seg.getAngleBottom(), angle_seg.getAngleBottom()));
            return;
        }
    }

    this->taken_angle_segments.emplace_back(angle_seg);
}

AngleSegment AngleMap::getBiggestViableAngleSegment()
{
    AngleSegment biggest_viable_angle_seg = AngleSegment(Angle::zero(), Angle::zero());
    if (this->taken_angle_segments.empty())
    {
        biggest_viable_angle_seg =
            AngleSegment(this->angle_seg.getAngleTop(), this->angle_seg.getAngleBottom());
        return biggest_viable_angle_seg;
    }

    AngleSegment first_taken_angle_seg = this->taken_angle_segments.front();
    if (first_taken_angle_seg.getAngleTop() < this->angle_seg.getAngleTop())
    {
        biggest_viable_angle_seg = AngleSegment(this->angle_seg.getAngleTop(),
                                                first_taken_angle_seg.getAngleTop());
    }

    AngleSegment last_taken_angle_seg = this->taken_angle_segments.back();
    if (last_taken_angle_seg.getAngleBottom() > this->angle_seg.getAngleBottom())
    {
        AngleSegment viable_angle_seg = AngleSegment(
            last_taken_angle_seg.getAngleBottom(), this->angle_seg.getAngleBottom());
        if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta())
        {
            biggest_viable_angle_seg = viable_angle_seg;
        }
    }

    for (auto i = this->taken_angle_segments.begin();
         i < this->taken_angle_segments.end() - 1; i++)
    {
        AngleSegment taken_angle_seg      = i[0];
        AngleSegment next_taken_angle_seg = i[1];
        AngleSegment viable_angle_seg     = AngleSegment(taken_angle_seg.getAngleBottom(),
                                                     next_taken_angle_seg.getAngleTop());
        if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta())
        {
            biggest_viable_angle_seg = viable_angle_seg;
        }
    }

    return biggest_viable_angle_seg;
}
