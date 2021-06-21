
#include "software/geom/angle_map.h"

AngleMap::AngleMap(Angle top_angle, Angle bottom_angle, size_t reserved_num_obstacles)
    : AngleMap(AngleSegment(top_angle, bottom_angle), reserved_num_obstacles)
{
}

AngleMap::AngleMap(AngleSegment angle_seg, size_t reserved_num_obstacles)
    : angle_seg(angle_seg)
{
    taken_angle_segments.reserve(reserved_num_obstacles);
}

const AngleSegment &AngleMap::getAngleSegment() const
{
    return angle_seg;
}

void AngleMap::addNonViableAngleSegment(AngleSegment &obstacle_angle_seg)
{
    for (AngleSegment &taken_angle_seg : taken_angle_segments)
    {
        if (!(obstacle_angle_seg.getAngleBottom() > taken_angle_seg.getAngleTop() ||
              obstacle_angle_seg.getAngleTop() < taken_angle_seg.getAngleBottom()))
        {
            taken_angle_seg.setAngleTop(std::max(taken_angle_seg.getAngleTop(),
                                                 obstacle_angle_seg.getAngleTop()));
            taken_angle_seg.setAngleBottom(std::min(taken_angle_seg.getAngleBottom(),
                                                    obstacle_angle_seg.getAngleBottom()));
            return;
        }
    }

    taken_angle_segments.emplace_back(obstacle_angle_seg);
}

AngleSegment AngleMap::getBiggestViableAngleSegment()
{
    AngleSegment biggest_viable_angle_seg = AngleSegment(Angle::zero(), Angle::zero());
    if (taken_angle_segments.empty())
    {
        biggest_viable_angle_seg =
            AngleSegment(angle_seg.getAngleTop(), angle_seg.getAngleBottom());
        return biggest_viable_angle_seg;
    }

    AngleSegment first_taken_angle_seg = taken_angle_segments.front();
    if (first_taken_angle_seg.getAngleTop() < angle_seg.getAngleTop())
    {
        biggest_viable_angle_seg =
            AngleSegment(angle_seg.getAngleTop(), first_taken_angle_seg.getAngleTop());
    }

    AngleSegment last_taken_angle_seg = taken_angle_segments.back();
    if (last_taken_angle_seg.getAngleBottom() > angle_seg.getAngleBottom())
    {
        AngleSegment viable_angle_seg = AngleSegment(
            last_taken_angle_seg.getAngleBottom(), angle_seg.getAngleBottom());
        if (viable_angle_seg.getDeltaInDegrees() >
            biggest_viable_angle_seg.getDeltaInDegrees())
        {
            biggest_viable_angle_seg = viable_angle_seg;
        }
    }

    for (auto i = taken_angle_segments.begin(); i < taken_angle_segments.end() - 1; i++)
    {
        AngleSegment taken_angle_seg      = i[0];
        AngleSegment next_taken_angle_seg = i[1];
        AngleSegment viable_angle_seg     = AngleSegment(taken_angle_seg.getAngleBottom(),
                                                     next_taken_angle_seg.getAngleTop());
        if (viable_angle_seg.getDeltaInDegrees() >
            biggest_viable_angle_seg.getDeltaInDegrees())
        {
            biggest_viable_angle_seg = viable_angle_seg;
        }
    }

    return biggest_viable_angle_seg;
}
