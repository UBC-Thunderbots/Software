#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/shot.h"
#include "software/geom/angle_segment.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/field.h"
#include "software/world/robot.h"
#include "software/world/team.h"
#include "software/world/world.h"


class ObstacleAngleSegment : public AngleSegment
{
   public:
    ObstacleAngleSegment(Angle angle_top, Angle angle_bottom)
        : AngleSegment(angle_top, angle_bottom)
    {
    }

    /**
     * Compares another ObstacleAngleSegment's top angle
     * Used to make sorting more performant
     *
     * @param other the other ObstacleAngleSegment to compare to
     * @return true if both ObstacleAngleSegment's top angles are equal
     */
    bool operator==(const ObstacleAngleSegment &other) const
    {
        return getAngleTop() == other.getAngleTop();
    }

    /**
     * Compares another ObstacleAngleSegment's top angle
     * Used to make sorting more performant
     *
     * @param other the other ObstacleAngleSegment to compare to
     * @return true if this top angle is less than the other's top angle
     */
    bool operator<(const ObstacleAngleSegment &other) const
    {
        return getAngleTop() < other.getAngleTop();
    }

    /**
     * Compares another ObstacleAngleSegment's top angle
     * Used to make sorting more performant
     *
     * @param other the other ObstacleAngleSegment to compare to
     * @return true if this top angle is greater than the other's top angle
     */
    bool operator>(const ObstacleAngleSegment &other) const
    {
        return getAngleTop() > other.getAngleTop();
    }
};

/**
 * Represents an AngleMap that is confined to a top and bottom angle
 */
class AngleMap
{
public:
    /**
     * Constructs an AngleMap with a specified top angle, bottom angle, and max number of
     * possible occupied AngleSegments within this map
     *
     * @param top_angle the top angle (most north) of the AngleSegment this map occupies
     * @param bottom_angle the bottom angle (most south) of the AngleSegment this map
     * occupies
     * @param max_num_obstacles the max number of possible occupied AngleSegments within
     * this map
     */
    AngleMap(Angle top_angle, Angle bottom_angle, size_t max_num_obstacles)
            : AngleMap(AngleSegment(top_angle, bottom_angle), max_num_obstacles)
    {
    }

    /**
     * Constructs an AngleMap with a specified AngleSegment and max number of possible
     * occupied AngleSegments within this map
     *
     * @param angle_seg the AngleSegment this map occupies
     * @param max_num_obstacles the max number of possible occupied AngleSegments within
     * this map
     */
    AngleMap(AngleSegment angle_seg, size_t max_num_obstacles) : angle_seg(angle_seg)
    {
        this->taken_angle_segments.reserve(max_num_obstacles);
    }

    /**
     * Gets the AngleSegment that this AngleMap occupies
     *
     * @return the AngleSegment that this AngleMap occupies
     */
    const AngleSegment &getAngleSegment() const
    {
        return this->angle_seg;
    }

    /**
     * Adds an AngleSegment to this map and marks it as occupied
     *
     * @param angleSegment the AngleSegment to mark as occupied
     */
    virtual void addNonViableAngleSegment(ObstacleAngleSegment &angleSegment) = 0;

    /**
     * Gets the biggest AngleSegment within the map that isn't occupied
     *
     * @return the biggest AngleSegment within the map that isn't occupied
     */
    virtual AngleSegment getBiggestViableAngleSegment() = 0;

protected:
    AngleSegment angle_seg;
    std::vector<ObstacleAngleSegment> taken_angle_segments;
};

/**
 * Represents an AngleMap that is for shooting on the enemy goal. The angles are described
 * as going from pi -> 0 -> -pi
 */
class EnemyAngleMap : public AngleMap
{
public:
    EnemyAngleMap(AngleSegment angle_seg, size_t max_num_obstacles)
            : AngleMap(angle_seg, max_num_obstacles)
    {
    }

    EnemyAngleMap(Angle top_angle, Angle bottom_angle, size_t max_num_obstacles)
            : AngleMap(top_angle, bottom_angle, max_num_obstacles)
    {
    }

    void addNonViableAngleSegment(ObstacleAngleSegment &angle_seg) override
    {
        for (ObstacleAngleSegment &taken_angle_seg : taken_angle_segments)
        {
            if (!(angle_seg.getAngleBottom() > taken_angle_seg.getAngleTop() ||
                  angle_seg.getAngleTop() < taken_angle_seg.getAngleBottom()))
            {
                taken_angle_seg.setAngleTop(
                        std::max(taken_angle_seg.getAngleTop(), angle_seg.getAngleTop()));
                taken_angle_seg.setAngleBottom(std::min(taken_angle_seg.getAngleBottom(),
                                                        angle_seg.getAngleBottom()));
                return;
            }
        }

        this->taken_angle_segments.emplace_back(angle_seg);
    }

    AngleSegment getBiggestViableAngleSegment() override
    {
        std::sort(this->taken_angle_segments.begin(), this->taken_angle_segments.end(),
                  [](AngleSegment a, AngleSegment b) -> bool {
                      Angle a_angle = a.getAngleTop();
                      Angle b_angle = b.getAngleTop();

                      if (a_angle < b_angle)
                      {
                          return false;
                      }
                      else if (a_angle > b_angle)
                      {
                          return true;
                      }

                      return false;
                  });


        AngleSegment biggest_viable_angle_seg =
                AngleSegment(Angle::zero(), Angle::zero());
        if (this->taken_angle_segments.empty())
        {
            biggest_viable_angle_seg = AngleSegment(this->angle_seg.getAngleTop(),
                                                    this->angle_seg.getAngleBottom());
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
            AngleSegment viable_angle_seg     = AngleSegment(
                    taken_angle_seg.getAngleBottom(), next_taken_angle_seg.getAngleTop());
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta())
            {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        return biggest_viable_angle_seg;
    }
};

/**
 * Represents an AngleMap that is for shooting on the friendly goal. The angles are
 * described as going from 0 -> 2 * pi
 */
class FriendlyAngleMap : public AngleMap
{
public:
    FriendlyAngleMap(AngleSegment angle_seg, size_t max_num_obstacles)
            : AngleMap(angle_seg, max_num_obstacles)
    {
    }

    FriendlyAngleMap(Angle top_angle, Angle bottom_angle, size_t max_num_obstacles)
            : AngleMap(top_angle, bottom_angle, max_num_obstacles)
    {
    }

    void addNonViableAngleSegment(ObstacleAngleSegment &angle_seg) override
    {
        for (AngleSegment &taken_angle_seg : taken_angle_segments)
        {
            if (!(angle_seg.getAngleBottom() < taken_angle_seg.getAngleTop() ||
                  angle_seg.getAngleTop() > taken_angle_seg.getAngleTop()))
            {
                taken_angle_seg.setAngleTop(
                        std::min(taken_angle_seg.getAngleTop(), angle_seg.getAngleTop()));
                taken_angle_seg.setAngleBottom(std::max(taken_angle_seg.getAngleBottom(),
                                                        angle_seg.getAngleBottom()));
                return;
            }
        }

        this->taken_angle_segments.emplace_back(angle_seg);
    }

    AngleSegment getBiggestViableAngleSegment() override
    {
        std::sort(this->taken_angle_segments.begin(), this->taken_angle_segments.end(),
                  [](AngleSegment a, AngleSegment b) -> bool {
                      Angle a_angle = a.getAngleTop();
                      Angle b_angle = b.getAngleTop();

                      if (a_angle > b_angle)
                      {
                          return false;
                      }
                      else if (a_angle < b_angle)
                      {
                          return true;
                      }

                      return false;
                  });


        AngleSegment biggest_viable_angle_seg =
                AngleSegment(Angle::zero(), Angle::zero());
        if (this->taken_angle_segments.empty())
        {
            biggest_viable_angle_seg = AngleSegment(this->angle_seg.getAngleTop(),
                                                    this->angle_seg.getAngleBottom());
            return biggest_viable_angle_seg;
        }

        AngleSegment first_taken_angle_seg = this->taken_angle_segments.front();
        if (first_taken_angle_seg.getAngleTop() > this->angle_seg.getAngleTop())
        {
            biggest_viable_angle_seg = AngleSegment(this->angle_seg.getAngleTop(),
                                                    first_taken_angle_seg.getAngleTop());
        }

        AngleSegment last_taken_angle_seg = this->taken_angle_segments.back();
        if (last_taken_angle_seg.getAngleBottom() < this->angle_seg.getAngleBottom())
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
            AngleSegment viable_angle_seg     = AngleSegment(
                    taken_angle_seg.getAngleBottom(), next_taken_angle_seg.getAngleTop());
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta())
            {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        return biggest_viable_angle_seg;
    }
};

/**
 * Finds the best shot on the given goal, and returns the best target to shoot at and
 * the largest open angle interval for the shot (this is the total angle between the
 * obstacles on either side of the shot vector).
 *
 * @param goal_post The goal post of the net by the y-coordinate
 * @param shot_origin The point that the shot will be taken from
 * @param robot_obstacles The locations of any robots on the field that may obstruct
 * the shot. These are treated as circular obstacles, and are usually used to
 * represent robots on the field.
 * @param goal The goal to shoot at
 * @param radius The radius for the robot obstacles
 *
 * @return the best target to shoot at and the largest open angle interval for the
 * shot (this is the total angle between the obstacles on either side of the shot
 * vector). If no shot is possible, returns `std::nullopt`
 */
std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles,
                                       TeamType goal,
                                       double radius = ROBOT_MAX_RADIUS_METERS);
/**
 * Finds the best shot on the specified goal, and returns the best target to shoot at
 * and the largest open angle interval for the shot (this is the total angle between
 * the obstacles on either side of the shot vector).
 *
 * This function will treat all robots on the field as obstacles, except for the
 * robots_to_ignore
 *
 * @param field The field
 * @param friendly_team The friendly team
 * @param enemy_team The enemy team
 * @param shot_origin The point that the shot will be taken from
 * @param goal The goal to shoot at
 * @param robots_to_ignore The robots to ignore
 * @param radius The radius for the robot obstacles
 *
 * @return the best target to shoot at and the largest open angle interval for the
 * shot (this is the total angle between the obstacles on either side of the shot
 * vector). If no shot can be found, returns std::nullopt
 */
std::optional<Shot> calcBestShotOnGoal(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore = {},
                                       double radius = ROBOT_MAX_RADIUS_METERS);
