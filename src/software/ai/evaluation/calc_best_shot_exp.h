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
    explicit AngleSegment(double angle_max, double angle_min, double cartesian_angle_max, double cartesian_angle_min) : angle_max_(angle_max), angle_min_(angle_min), cartesian_angle_max_(cartesian_angle_max), cartesian_angle_min_(cartesian_angle_min) {

    }

    double getAngleMax() {
        return angle_max_;
    }

    void setAngleMax(double angle_max) {
        this->angle_max_ = angle_max;
    }

    double getAngleMin() {
        return angle_min_;
    }

    void setAngleMin(double angle_min) {
        this->angle_min_ = angle_min;
    }

    double getDelta() {
        return std::abs(angle_min_ - angle_max_);
    }

    double getCartesianAngleMax() {
        return cartesian_angle_max_;
    }

    void setCartesianAngleMax(double cartesian_angle_max) {
        this->cartesian_angle_max_ = cartesian_angle_max;
    }

    double getCartesianAngleMin() {
        return cartesian_angle_min_;
    }

    void setCartesianAngleMin(double cartesian_angle_min) {
        this->cartesian_angle_min_ = cartesian_angle_min;
    }

    double getY() {
        return this->y;
    }

    void setY(double y_) {
        this->y = y_;
    }

private:
    double angle_max_;
    double angle_min_;
    double cartesian_angle_max_;
    double cartesian_angle_min_;
    double y;
};

class AngleMap {
public:
    explicit AngleMap(double angle_max, double angle_min, double angle_0_max, double angle_0_min) : angle_max(angle_max), angle_min(angle_min), angle_0_max(angle_0_max), angle_0_min(angle_0_min) {
    }

    double getAngleMax() {
        return this->angle_max;
    }

    double getAngle0Max() {
        return this->angle_0_max;
    }

    double getAngleMin() {
        return this->angle_min;
    }

    double getAngle0Min() {
        return this->angle_0_min;
    }

    void addNonViableAngleSegment(AngleSegment &angle_seg)
    {
        for (AngleSegment &taken_angle_seg : taken_angle_segments) {
            if (
                    (angle_seg.getAngleMax() < taken_angle_seg.getAngleMin() && angle_seg.getAngleMin() > taken_angle_seg.getAngleMax()) ||
                    (angle_seg.getAngleMin() > taken_angle_seg.getAngleMax() && angle_seg.getAngleMin() < taken_angle_seg.getAngleMin())
                    ) {
                if (taken_angle_seg.getAngleMax() > angle_seg.getAngleMax()) {
                    taken_angle_seg.setCartesianAngleMax(angle_seg.getCartesianAngleMax());
                }
                taken_angle_seg.setAngleMax(std::min(taken_angle_seg.getAngleMax(), angle_seg.getAngleMax()));

                if (taken_angle_seg.getAngleMin() < angle_seg.getAngleMin()) {
                    taken_angle_seg.setCartesianAngleMin(angle_seg.getCartesianAngleMin());
                }
                taken_angle_seg.setAngleMin(std::max(taken_angle_seg.getAngleMin(), angle_seg.getAngleMin()));
                return;
            }
        }

        this->taken_angle_segments.emplace_back(angle_seg);
    }

    AngleSegment getBiggestViableAngleSegment() {

        AngleSegment biggest_viable_angle_seg = AngleSegment(0, 0, 0, 0);

        if (this->taken_angle_segments.empty()) {
            biggest_viable_angle_seg = AngleSegment(this->angle_max, this->angle_min, this->angle_0_max, this->angle_0_min);
            std::cout << "TEST2W\n";
            return biggest_viable_angle_seg;
        }

        AngleSegment first_taken_angle_seg = this->taken_angle_segments.front();
        if (first_taken_angle_seg.getAngleMax() > this->angle_max) {
            biggest_viable_angle_seg = AngleSegment(this->angle_max, first_taken_angle_seg.getAngleMax(), this->angle_0_max, first_taken_angle_seg.getCartesianAngleMax());
        }
        else if (first_taken_angle_seg.getAngleMax() < this->angle_max && first_taken_angle_seg.getAngleMin() > this->angle_min) {
            return biggest_viable_angle_seg;
        }

        AngleSegment last_taken_angle_seg = this->taken_angle_segments.back();
        if (last_taken_angle_seg.getAngleMin() < this->angle_min) {
            AngleSegment viable_angle_seg = AngleSegment(last_taken_angle_seg.getAngleMin(), this->angle_min, last_taken_angle_seg.getCartesianAngleMin(), this->angle_0_min);

//            std::cout << viable_angle_seg.getAngleMax() << "\n";
//            std::cout << this->angle_min << "\n";
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

//        std::sort(obstacles.begin(), obstacles.end(), [&field](const Segment &a, const Segment &b) -> int {
//            Point a_start_screen_pos = screenPoint(field, a.getStart());
//            Point b_start_screen_pos = screenPoint(field, b.getStart());
//
//            if (a_start_screen_pos.y() > b_start_screen_pos.y()) {
//                return -1;
//            }
//            else if (a_start_screen_pos.y() < b_start_screen_pos.y()) {
//                return 1;
//            }
//
//            return 0;
//        });

        for (auto i = this->taken_angle_segments.begin(); i < this->taken_angle_segments.end() - 1; i++) {
            AngleSegment taken_angle_seg = i[0];
            AngleSegment next_taken_angle_seg = i[1];

            std::cout << "-----------------\n";
            std::cout << "FIRST ANGLE_MAX: " << taken_angle_seg.getAngleMax() << "\n";
            std::cout << "FIRST ANGLE_MIN: " << taken_angle_seg.getAngleMin() << "\n";
            std::cout << "NEXT ANGLE_MAX: " << next_taken_angle_seg.getAngleMax() << "\n";
            std::cout << "NEXT ANGLE_MIN: " << next_taken_angle_seg.getAngleMin() << "\n";

            AngleSegment viable_angle_seg = AngleSegment(taken_angle_seg.getAngleMin(), next_taken_angle_seg.getAngleMax(), taken_angle_seg.getCartesianAngleMin(), next_taken_angle_seg.getCartesianAngleMax());

            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        return biggest_viable_angle_seg;
    }

private:
    double angle_max;
    double angle_min;
    double angle_0_max;
    double angle_0_min;
    std::vector<AngleSegment> taken_angle_segments;

    std::unique_ptr<AngleSegment> getAngleSegmentOverlap(AngleSegment &angle_seg) {
        for (AngleSegment &taken_angle_seg : taken_angle_segments) {
            if (
                    (angle_seg.getAngleMax() < taken_angle_seg.getAngleMin() && angle_seg.getAngleMin() > taken_angle_seg.getAngleMax()) ||
                    (angle_seg.getAngleMin() > taken_angle_seg.getAngleMax() && angle_seg.getAngleMin() < taken_angle_seg.getAngleMin())
                    ) {
                return std::make_unique<AngleSegment>(taken_angle_seg);
            }
        }

        return nullptr;
    }
};

std::pair<double, double> screenValues(const Field &field, double cartesian_x, double cartesian_y);
Point screenPoint(const Field &field, const Point& cartesian_point);
Vector screenVector(const Field &field, const Vector& cartesian_vector);

std::pair<double, double> cartesianValues(const Field &field, double screen_x, double screen_y);
Point cartesianPoint(const Field &field, const Point& screen_point);
Vector cartesianVector(const Field &field, const Vector& screen_vector);

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

std::pair<AngleSegment, std::pair<AngleMap, std::vector<Segment>>> calcBestShotOnGoalExpTest(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore);
