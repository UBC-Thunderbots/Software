#pragma once

#include "shared/constants.h"
#include "software/ai/evaluation/shot.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/util/make_enum/make_enum.h"
#include "software/world/field.h"
#include "software/world/robot.h"
#include "software/world/team.h"
#include "software/world/world.h"

class AngleSegment {
public:
    explicit AngleSegment(Angle angle_top, Angle angle_bottom) : angle_top_(angle_top), angle_bottom_(angle_bottom) {

    }

    Angle getAngleTop() {
        return angle_top_;
    }

    void setAngleTop(Angle angle_top) {
        this->angle_top_ = angle_top;
    }

    Angle getAngleBottom() {
        return angle_bottom_;
    }

    void setAngleBottom(Angle angle_bottom) {
        this->angle_bottom_ = angle_bottom;
    }

    double getDelta() {
        return (angle_bottom_ - angle_top_).abs().toDegrees();
    }

    bool operator==(const AngleSegment & other) const {
        return other.angle_top_ == angle_top_;
    }

    bool operator<(const AngleSegment & other) const{
        return angle_top_ < other.angle_top_;
    }

    bool operator>(const AngleSegment & other) const{
        return angle_top_ > other.angle_top_;
    }

private:
    Angle angle_top_;
    Angle angle_bottom_;
};

class EnemyAngleMap {
public:
    explicit EnemyAngleMap(Angle angle_top, Angle angle_bottom, size_t max_num_obstacles) : angle_top(angle_top), angle_bottom(angle_bottom) {
        this->taken_angle_segments.reserve(max_num_obstacles);
    }

    Angle getAngleTop() {
        return this->angle_top;
    }

    Angle getAngleBottom() {
        return this->angle_bottom;
    }

    void addNonViableAngleSegment(AngleSegment &angle_seg)
    {
        for (AngleSegment &taken_angle_seg : taken_angle_segments) {
            if (!(angle_seg.getAngleBottom() > taken_angle_seg.getAngleTop() || angle_seg.getAngleTop() < taken_angle_seg.getAngleBottom())) {
                taken_angle_seg.setAngleTop(std::max(taken_angle_seg.getAngleTop(), angle_seg.getAngleTop()));
                taken_angle_seg.setAngleBottom(std::min(taken_angle_seg.getAngleBottom(), angle_seg.getAngleBottom()));
                return;
            }
        }

        this->taken_angle_segments.emplace_back(angle_seg);
    }

    AngleSegment getBiggestViableAngleSegment() {

        std::sort(this->taken_angle_segments.begin(), this->taken_angle_segments.end(), [](AngleSegment a, AngleSegment b) -> bool {
            Angle a_angle = a.getAngleTop();
            Angle b_angle = b.getAngleTop();

            if (a_angle < b_angle) {
                return false;
            }
            else if (a_angle > b_angle) {
                return true;
            }

            return false;
        });


        AngleSegment biggest_viable_angle_seg = AngleSegment(Angle::zero(), Angle::zero());
        if (this->taken_angle_segments.empty()) {
            biggest_viable_angle_seg = AngleSegment(this->angle_top, this->angle_bottom);
            return biggest_viable_angle_seg;
        }

        AngleSegment first_taken_angle_seg = this->taken_angle_segments.front();
        if (first_taken_angle_seg.getAngleTop() < this->angle_top) {
            biggest_viable_angle_seg = AngleSegment(this->angle_top, first_taken_angle_seg.getAngleTop());
        }

        AngleSegment last_taken_angle_seg = this->taken_angle_segments.back();
        if (last_taken_angle_seg.getAngleBottom() > this->angle_bottom) {
            AngleSegment viable_angle_seg = AngleSegment(last_taken_angle_seg.getAngleBottom(), this->angle_bottom);
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        for (auto i = this->taken_angle_segments.begin(); i < this->taken_angle_segments.end() - 1; i++) {
            AngleSegment taken_angle_seg = i[0];
            AngleSegment next_taken_angle_seg = i[1];
            AngleSegment viable_angle_seg = AngleSegment(taken_angle_seg.getAngleBottom(), next_taken_angle_seg.getAngleTop());
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        return biggest_viable_angle_seg;
    }

private:
    Angle angle_top;
    Angle angle_bottom;
    std::vector<AngleSegment> taken_angle_segments;
};

class FriendlyAngleMap {
public:
    explicit FriendlyAngleMap(Angle angle_top, Angle angle_bottom, size_t max_num_obstacles) : angle_top(angle_top), angle_bottom(angle_bottom) {
        this->taken_angle_segments.reserve(max_num_obstacles);
    }

    //smaller than bottom angle
    Angle getAngleTop() {
        return this->angle_top;
    }

    //larger than top angle
    Angle getAngleBottom() {
        return this->angle_bottom;
    }

    void addNonViableAngleSegment(AngleSegment &angle_seg)
    {
        for (AngleSegment &taken_angle_seg : taken_angle_segments) {
            if (!(angle_seg.getAngleBottom() < taken_angle_seg.getAngleTop() || angle_seg.getAngleTop() > taken_angle_seg.getAngleTop())) {
                taken_angle_seg.setAngleTop(std::min(taken_angle_seg.getAngleTop(), angle_seg.getAngleTop()));
                taken_angle_seg.setAngleBottom(std::max(taken_angle_seg.getAngleBottom(), angle_seg.getAngleBottom()));
                return;
            }
        }

        this->taken_angle_segments.emplace_back(angle_seg);
    }

    AngleSegment getBiggestViableAngleSegment() {

        std::sort(this->taken_angle_segments.begin(), this->taken_angle_segments.end(), [](AngleSegment a, AngleSegment b) -> bool {
            Angle a_angle = a.getAngleTop();
            Angle b_angle = b.getAngleTop();

            if (a_angle > b_angle) {
                return false;
            }
            else if (a_angle < b_angle) {
                return true;
            }

            return false;
        });


        AngleSegment biggest_viable_angle_seg = AngleSegment(Angle::zero(), Angle::zero());
        if (this->taken_angle_segments.empty()) {
            biggest_viable_angle_seg = AngleSegment(this->angle_top, this->angle_bottom);
            return biggest_viable_angle_seg;
        }

        AngleSegment first_taken_angle_seg = this->taken_angle_segments.front();
        if (first_taken_angle_seg.getAngleTop() > this->angle_top) {
            biggest_viable_angle_seg = AngleSegment(this->angle_top, first_taken_angle_seg.getAngleTop());
        }

        AngleSegment last_taken_angle_seg = this->taken_angle_segments.back();
        if (last_taken_angle_seg.getAngleBottom() < this->angle_bottom) {
            AngleSegment viable_angle_seg = AngleSegment(last_taken_angle_seg.getAngleBottom(), this->angle_bottom);
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        for (auto i = this->taken_angle_segments.begin(); i < this->taken_angle_segments.end() - 1; i++) {
            AngleSegment taken_angle_seg = i[0];
            AngleSegment next_taken_angle_seg = i[1];
            AngleSegment viable_angle_seg = AngleSegment(taken_angle_seg.getAngleBottom(), next_taken_angle_seg.getAngleTop());
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        return biggest_viable_angle_seg;
    }

private:
    Angle angle_top;
    Angle angle_bottom;
    std::vector<AngleSegment> taken_angle_segments;
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
 *
 * @return the best target to shoot at and the largest open angle interval for the
 * shot (this is the total angle between the obstacles on either side of the shot
 * vector). If no shot is possible, returns `std::nullopt`
 */
std::optional<Shot> calcBestShotOnGoal(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles,
                                       TeamType goal, double radius = ROBOT_MAX_RADIUS_METERS);
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