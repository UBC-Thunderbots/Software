#include <optional>
#include "calc_best_shot_exp.h"
#include "software/geom/ray.h"
#include "software/geom/algorithms/intersection.h"

class AngleSegment {
public:
    explicit AngleSegment(double angle_max, double angle_min, double angle_0_max, double angle_0_min) : angle_max_(angle_max), angle_min_(angle_min), angle_0_max_(angle_0_max), angle_0_min_(angle_0_min) {

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

    double getAngle0Max() {
        return angle_0_max_;
    }

    double getAngle0Min() {
        return angle_0_min_;
    }

private:
    double angle_max_;
    double angle_min_;
    double angle_0_max_;
    double angle_0_min_;
};

class AngleMap {
public:
    explicit AngleMap(double angle_max, double angle_min, double angle_0_max, double angle_0_min) : angle_max(angle_max), angle_min(angle_min), angle_0_max(angle_0_max), angle_0_min(angle_0_min) {
    }

    double getAngleMax() {
        return this->angle_max;
    }

    double getAngleMin() {
        return this->angle_min;
    }

    void addNonViableAngleSegment(AngleSegment &angle_seg) {
        std::optional<AngleSegment> optional_overlap_seg = getAngleSegmentOverlap(angle_seg);
        if (optional_overlap_seg.has_value()) {
            AngleSegment overlap_seg = optional_overlap_seg.value();

            angle_seg.setAngleMax(std::min(overlap_seg.getAngleMax(), angle_seg.getAngleMax()));
            angle_seg.setAngleMin(std::max(overlap_seg.getAngleMin(), angle_seg.getAngleMin()));
        } else {
            this->taken_angle_segments.emplace_back(angle_seg);
        }
    }

    AngleSegment getBiggestViableAngleSegment() {
        AngleSegment biggest_viable_angle_seg = AngleSegment(0, 0, 0, 0);

        AngleSegment first_taken_angle_seg = this->taken_angle_segments.front();
        if (first_taken_angle_seg.getAngleMax() > this->angle_max) {
            biggest_viable_angle_seg = AngleSegment(this->angle_max, first_taken_angle_seg.getAngleMax(), this->angle_0_max, first_taken_angle_seg.getAngle0Max());
        }

        AngleSegment last_taken_angle_seg = this->taken_angle_segments.back();
        if (last_taken_angle_seg.getAngleMin() < this->angle_min) {
            AngleSegment viable_angle_seg = AngleSegment(last_taken_angle_seg.getAngleMin(), this->angle_min, last_taken_angle_seg.getAngle0Min(), this->angle_0_min);
            if (viable_angle_seg.getDelta() > biggest_viable_angle_seg.getDelta()) {
                biggest_viable_angle_seg = viable_angle_seg;
            }
        }

        for (auto i = this->taken_angle_segments.begin(); i < this->taken_angle_segments.end() - 1; i++) {
            AngleSegment taken_angle_seg = i[0];
            AngleSegment next_taken_angle_seg = i[1];

            AngleSegment viable_angle_seg = AngleSegment(taken_angle_seg.getAngleMin(), next_taken_angle_seg.getAngleMax(), taken_angle_seg.getAngle0Min(), next_taken_angle_seg.getAngle0Max());
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

    std::optional<AngleSegment> getAngleSegmentOverlap(AngleSegment &angle_seg) {
        for (AngleSegment taken_angle_seg : taken_angle_segments) {
            if (
                    (angle_seg.getAngleMax() < taken_angle_seg.getAngleMax() && angle_seg.getAngleMin() > taken_angle_seg.getAngleMin()) ||
                    (angle_seg.getAngleMax() < taken_angle_seg.getAngleMin() && angle_seg.getAngleMin() > taken_angle_seg.getAngleMax()) ||
                    (angle_seg.getAngleMin() > taken_angle_seg.getAngleMin() && angle_seg.getAngleMin() < taken_angle_seg.getAngleMax())
            ) {
                return taken_angle_seg;
            }
        }

        return std::nullopt;
    }
};

std::optional<Shot> calcBestShotOnGoalExp(const Segment &goal_post, const Point &shot_origin,
                                          const std::vector<Robot> &robot_obstacles) {
    return std::nullopt;
}

std::optional<Shot> calcBestShotOnGoalExp(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore) {

    Segment goal_seg = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

    std::vector<Segment> obstacles;
    for (const Robot &enemy_robot : enemy_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
            0)
        {
            Vector relative_vector = enemy_robot.position() - shot_origin;
            Vector perpendicular = relative_vector.perpendicular();

            Vector one_end = perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Vector other_end = -perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Segment obstacle = Segment(Point(one_end.x(), one_end.y()), Point(other_end.x(), other_end.y()));

            obstacles.emplace_back(obstacle);
        }
    }
    for (const Robot &friendly_robot : friendly_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                       friendly_robot) == 0)
        {
            Vector relative_vector = friendly_robot.position() - shot_origin;
            Vector perpendicular = relative_vector.perpendicular();

            Vector one_end = perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Vector other_end = -perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Segment obstacle = Segment(Point(one_end.x(), one_end.y()), Point(other_end.x(), other_end.y()));

            obstacles.emplace_back(obstacle);
        }
    }

    Vector top_post_vec = field.enemyGoalpostPos() - shot_origin;
    Vector bottom_post_vec = field.enemyGoalpostNeg() - shot_origin;
    Vector reference = Vector(0, -1);
    Vector reference0 = Vector(0, 0);

    double top_angle = std::abs((180 / M_PI) * top_post_vec.angleBetween(reference));
    double bottom_angle = std::abs((180 / M_PI) * bottom_post_vec.angleBetween(reference));
    double top_angle_0 = (180 / M_PI) * top_post_vec.angleBetween(reference0);
    double bottom_angle_0 = (180 / M_PI) * bottom_post_vec.angleBetween(reference0);


    std::sort(obstacles.begin(), obstacles.end(), [&shot_origin, &reference](const Segment &a, const Segment &b) -> int {
        double a_angle_max = std::abs((180 / M_PI) * (a.getStart() - shot_origin).angleBetween(reference));
        double b_angle_max = std::abs((180 / M_PI) * (b.getStart() - shot_origin).angleBetween(reference));

        if (a_angle_max < b_angle_max) {
            return -1;
        }
        else if (a_angle_max > b_angle_max) {
            return 1;
        }

        return 0;
    });

    AngleMap angleMap = AngleMap(top_angle, bottom_angle, top_angle_0, bottom_angle_0);

    for (const Segment& obstacle : obstacles) {
        Vector top_vec = obstacle.getStart() - shot_origin;
        double top_obstacle_angle = std::abs((180 / M_PI) * top_vec.angleBetween(reference));
        double top_obstacle_angle_0 = (180 / M_PI) * top_vec.angleBetween(reference0);

        Vector bottom_vec = obstacle.getEnd() - shot_origin;
        double bottom_obstacle_angle = std::abs((180 / M_PI) * bottom_vec.angleBetween(reference));
        double bottom_obstacle_angle_0 = (180 / M_PI) * bottom_vec.angleBetween(reference0);

        if (bottom_obstacle_angle > angleMap.getAngleMax() || top_obstacle_angle < angleMap.getAngleMin()) {
            AngleSegment non_viable_angle_seg = AngleSegment(top_obstacle_angle, bottom_obstacle_angle, top_obstacle_angle_0, bottom_obstacle_angle_0);
            angleMap.addNonViableAngleSegment(non_viable_angle_seg);
        }
    }

    AngleSegment biggest_angle_seg = angleMap.getBiggestViableAngleSegment();

    Ray top_ray = Ray(shot_origin, Angle::fromDegrees(biggest_angle_seg.getAngle0Max()));
    Ray bottom_ray = Ray(shot_origin, Angle::fromDegrees(biggest_angle_seg.getAngle0Min()));

    Segment available_shot_seg = Segment(intersection(top_ray, goal_seg).front(), intersection(bottom_ray, goal_seg).front());

    return std::make_optional(Shot(available_shot_seg.midPoint(), Angle::fromDegrees(biggest_angle_seg.getDelta())));
}