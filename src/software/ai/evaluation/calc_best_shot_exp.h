#pragma once

#include <utility>

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
    explicit AngleSegment(Angle angle_max, Angle angle_min) : angle_max_(angle_max), angle_min_(angle_min) {

    }

    Angle getAngleMax() {
        return angle_max_;
    }

    void setAngleMax(Angle angle_max) {
        this->angle_max_ = angle_max;
    }

    Angle getAngleMin() {
        return angle_min_;
    }

    void setAngleMin(Angle angle_min) {
        this->angle_min_ = angle_min;
    }

    double getDelta() {
        return std::abs((angle_min_ - angle_max_).toDegrees());
    }

private:
    Angle angle_max_;
    Angle angle_min_;
};

class AngleMap {
public:
    explicit AngleMap(Angle angle_max, Angle angle_min, Angle angle_0_max, Angle angle_0_min) : angle_max(angle_max), angle_min(angle_min) {
    }

    Angle getAngleMax() {
        return this->angle_max;
    }

    Angle getAngleMin() {
        return this->angle_min;
    }

    void addNonViableAngleSegment(AngleSegment &angle_seg)
    {
        for (AngleSegment &taken_angle_seg : taken_angle_segments) {
            if (!(angle_seg.getAngleMin() > taken_angle_seg.getAngleMax() || angle_seg.getAngleMax() < taken_angle_seg.getAngleMin())) {
                taken_angle_seg.setAngleMax(std::max(taken_angle_seg.getAngleMax(), angle_seg.getAngleMax()));
                taken_angle_seg.setAngleMin(std::min(taken_angle_seg.getAngleMin(), angle_seg.getAngleMin()));
                return;
            }
        }

        this->taken_angle_segments.emplace_back(angle_seg);
    }

    AngleSegment getBiggestViableAngleSegment() {

        AngleSegment biggest_viable_angle_seg = AngleSegment(Angle::zero(), Angle::zero());
        if (this->taken_angle_segments.empty()) {
            biggest_viable_angle_seg = AngleSegment(this->angle_max, this->angle_min);
            return biggest_viable_angle_seg;
        }

        AngleSegment first_taken_angle_seg = this->taken_angle_segments.front();
        if (first_taken_angle_seg.getAngleMax() < this->angle_max) {
            biggest_viable_angle_seg = AngleSegment(this->angle_max, first_taken_angle_seg.getAngleMax());
        }

        AngleSegment last_taken_angle_seg = this->taken_angle_segments.back();
        if (last_taken_angle_seg.getAngleMin() > this->angle_min) {
            AngleSegment viable_angle_seg = AngleSegment(last_taken_angle_seg.getAngleMin(), this->angle_min);
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        for (auto i = this->taken_angle_segments.begin(); i < this->taken_angle_segments.end() - 1; i++) {
            AngleSegment taken_angle_seg = i[0];
            AngleSegment next_taken_angle_seg = i[1];
            AngleSegment viable_angle_seg = AngleSegment(taken_angle_seg.getAngleMin(), next_taken_angle_seg.getAngleMax());

            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        return biggest_viable_angle_seg;
    }

private:
    Angle angle_max;
    Angle angle_min;
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
std::optional<Shot> calcBestShotOnGoalExp(const Segment &goal_post, const Point &shot_origin,
                                       const std::vector<Robot> &robot_obstacles);

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
std::optional<Shot> calcBestShotOnGoalExp(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore = {});

std::pair<AngleSegment, std::pair<AngleMap, std::vector<AngleSegment>>> calcBestShotOnGoalExpTest(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore);
