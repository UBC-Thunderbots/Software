#include <optional>
#include "calc_best_shot_exp.h"
#include "software/geom/ray.h"
#include "software/geom/algorithms/intersection.h"



std::optional<Shot> calcBestShotOnGoalExp(const Segment &goal_post, const Point &shot_origin,
                                          const std::vector<Robot> &robot_obstacles) {
    return std::nullopt;
}

std::optional<Shot> calcBestShotOnGoalExp(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore) {

    Segment goal_seg = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

    Point shot_origin_screen_pos = screenPoint(field, shot_origin);
    std::vector<Segment> obstacles;
    for (const Robot &enemy_robot : enemy_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
            0)
        {
            Point enemy_robot_screen_pos = screenPoint(field, enemy_robot.position());

            if (enemy_robot_screen_pos.x() < shot_origin_screen_pos.x()) {
                continue;
            }

            Vector relative_vector = enemy_robot.position() - shot_origin;
            Vector perpendicular = relative_vector.perpendicular();

            Vector one_end = perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Vector other_end = -perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Segment obstacle = Segment(Point(enemy_robot.position().x() + one_end.x(), enemy_robot.position().y() + one_end.y()), Point(enemy_robot.position().x() + other_end.x(), enemy_robot.position().y() + other_end.y()));

            obstacles.emplace_back(obstacle);
        }
    }
    for (const Robot &friendly_robot : friendly_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                       friendly_robot) == 0)
        {
            Point friendly_robot_screen_pos = screenPoint(field, friendly_robot.position());

            if (friendly_robot_screen_pos.x() < shot_origin_screen_pos.x()) {
                continue;
            }

            Vector relative_vector = friendly_robot.position() - shot_origin;
            Vector perpendicular = relative_vector.perpendicular();

            Vector one_end = perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Vector other_end = -perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Segment obstacle = Segment(Point(friendly_robot.position().x() + one_end.x(), friendly_robot.position().y() + one_end.y()), Point(friendly_robot.position().x() + other_end.x(), friendly_robot.position().y() + other_end.y()));

            obstacles.emplace_back(obstacle);
        }
    }

    Point enemy_top_post_screen_pos = screenPoint(field, field.enemyGoalpostPos());
    Vector top_post_vec = enemy_top_post_screen_pos - shot_origin_screen_pos;

    Point enemy_bottom_post_screen_pos = screenPoint(field, field.enemyGoalpostNeg());
    Vector bottom_post_vec = enemy_bottom_post_screen_pos - shot_origin_screen_pos;
    Vector reference = Vector(0, -1);

    double top_angle = std::abs((180 / M_PI) * top_post_vec.angleBetween(reference));
    double bottom_angle = std::abs((180 / M_PI) * bottom_post_vec.angleBetween(reference));
    double top_cartesian_angle = (field.enemyGoalpostPos() - shot_origin).orientation().toDegrees();
    double bottom_cartesian_angle = (field.enemyGoalpostNeg() - shot_origin).orientation().toDegrees();

    AngleMap angleMap = AngleMap(top_angle, bottom_angle, top_cartesian_angle, bottom_cartesian_angle);

    std::sort(obstacles.begin(), obstacles.end(), [&reference, &shot_origin_screen_pos, &field](const Segment &a, const Segment &b) -> bool {
        Point a_start_screen_pos = screenPoint(field, a.getStart());
        Vector a_vec = a_start_screen_pos - shot_origin_screen_pos;
        double a_obstacle_angle = std::abs((180 / M_PI) * a_vec.angleBetween(reference));

        Point b_start_screen_pos = screenPoint(field, b.getStart());
        Vector b_vec = b_start_screen_pos - shot_origin_screen_pos;
        double b_obstacle_angle = std::abs((180 / M_PI) * b_vec.angleBetween(reference));

        if (a_obstacle_angle > b_obstacle_angle) {
            return false;
        }
        else if (a_obstacle_angle < b_obstacle_angle) {
            return true;
        }

        return false;
    });

    for (Segment &obstacle : obstacles) {

        const Point& start = obstacle.getStart();
        const Point& end = obstacle.getEnd();

        Point start_screen_pos = screenPoint(field, start);
        Point end_screen_pos = screenPoint(field, end);

        Vector top_vec = start_screen_pos - shot_origin_screen_pos;
        double top_obstacle_angle = std::abs((180 / M_PI) * top_vec.angleBetween(reference));
        double top_obstacle_cartesian_angle = (start - shot_origin).orientation().toDegrees();


        Vector bottom_vec = end_screen_pos - shot_origin_screen_pos;
        double bottom_obstacle_angle = std::abs((180 / M_PI) * bottom_vec.angleBetween(reference));
        double bottom_obstacle_cartesian_angle = (end - shot_origin).orientation().toDegrees();

        if ((bottom_obstacle_angle > angleMap.getAngleMax() && bottom_obstacle_angle < angleMap.getAngleMin())  || (top_obstacle_angle < angleMap.getAngleMin() && top_obstacle_angle > angleMap.getAngleMax())) {
            AngleSegment non_viable_angle_seg = AngleSegment(top_obstacle_angle, bottom_obstacle_angle, top_obstacle_cartesian_angle, bottom_obstacle_cartesian_angle);

            angleMap.addNonViableAngleSegment(non_viable_angle_seg);
        }
        else if (bottom_obstacle_angle > angleMap.getAngleMin() && top_obstacle_angle < angleMap.getAngleMax()) {
            AngleSegment non_viable_angle_seg = AngleSegment(top_obstacle_angle, bottom_obstacle_angle, top_obstacle_cartesian_angle, bottom_obstacle_cartesian_angle);
            angleMap.addNonViableAngleSegment(non_viable_angle_seg);
        }
    }

    AngleSegment biggest_angle_seg = angleMap.getBiggestViableAngleSegment();

    Ray mid_point_ray = Ray(shot_origin, Angle::fromDegrees(((biggest_angle_seg.getCartesianAngleMax() - biggest_angle_seg.getCartesianAngleMin()) /  2) + biggest_angle_seg.getCartesianAngleMax()));

    std::vector<Point> intersections = intersection(mid_point_ray, goal_seg);

    Angle open_angle = Angle::fromDegrees(biggest_angle_seg.getDelta());
    return std::make_optional(Shot(intersections.front(), open_angle));
}

std::pair<AngleSegment, std::pair<AngleMap, std::vector<Segment>>> calcBestShotOnGoalExpTest(const Field &field, const Team &friendly_team,
                                       const Team &enemy_team, const Point &shot_origin,
                                       TeamType goal,
                                       const std::vector<Robot> &robots_to_ignore) {

    //Segment goal_seg = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

    Point shot_origin_screen_pos = screenPoint(field, shot_origin);
    std::vector<Segment> obstacles;
    for (const Robot &enemy_robot : enemy_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(), enemy_robot) ==
            0)
        {
            Point enemy_robot_screen_pos = screenPoint(field, enemy_robot.position());

            if (enemy_robot_screen_pos.x() < shot_origin_screen_pos.x()) {
                continue;
            }

            Vector relative_vector = enemy_robot.position() - shot_origin;
            Vector perpendicular = relative_vector.perpendicular();

            Vector one_end = perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Vector other_end = -perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Segment obstacle = Segment(Point(enemy_robot.position().x() + one_end.x(), enemy_robot.position().y() + one_end.y()), Point(enemy_robot.position().x() + other_end.x(), enemy_robot.position().y() + other_end.y()));

            obstacles.emplace_back(obstacle);
        }
    }
    for (const Robot &friendly_robot : friendly_team.getAllRobots())
    {
        // Only add the robot to the obstacles if it is not ignored
        if (std::count(robots_to_ignore.begin(), robots_to_ignore.end(),
                       friendly_robot) == 0)
        {
            Point friendly_robot_screen_pos = screenPoint(field, friendly_robot.position());

            if (friendly_robot_screen_pos.x() < shot_origin_screen_pos.x()) {
                continue;
            }

            Vector relative_vector = friendly_robot.position() - shot_origin;
            Vector perpendicular = relative_vector.perpendicular();

            Vector one_end = perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Vector other_end = -perpendicular.normalize(ROBOT_MAX_RADIUS_METERS);
            Segment obstacle = Segment(Point(friendly_robot.position().x() + one_end.x(), friendly_robot.position().y() + one_end.y()), Point(friendly_robot.position().x() + other_end.x(), friendly_robot.position().y() + other_end.y()));

            obstacles.emplace_back(obstacle);
        }
    }

    Point enemy_top_post_screen_pos = screenPoint(field, field.enemyGoalpostPos());
    Vector top_post_vec = enemy_top_post_screen_pos - shot_origin_screen_pos;

    Point enemy_bottom_post_screen_pos = screenPoint(field, field.enemyGoalpostNeg());
    Vector bottom_post_vec = enemy_bottom_post_screen_pos - shot_origin_screen_pos;
    Vector reference = Vector(0, -1);

    double top_angle = std::abs((180 / M_PI) * top_post_vec.angleBetween(reference));
    double bottom_angle = std::abs((180 / M_PI) * bottom_post_vec.angleBetween(reference));
    double top_cartesian_angle = (field.enemyGoalpostPos() - shot_origin).orientation().toDegrees();
    double bottom_cartesian_angle = (field.enemyGoalpostNeg() - shot_origin).orientation().toDegrees();

    AngleMap angleMap = AngleMap(top_angle, bottom_angle, top_cartesian_angle, bottom_cartesian_angle);

    std::sort(obstacles.begin(), obstacles.end(), [&reference, &shot_origin_screen_pos, &field](const Segment &a, const Segment &b) -> bool {
        Point a_start_screen_pos = screenPoint(field, a.getStart());
        Vector a_vec = a_start_screen_pos - shot_origin_screen_pos;
        double a_obstacle_angle = std::abs((180 / M_PI) * a_vec.angleBetween(reference));

        Point b_start_screen_pos = screenPoint(field, b.getStart());
        Vector b_vec = b_start_screen_pos - shot_origin_screen_pos;
        double b_obstacle_angle = std::abs((180 / M_PI) * b_vec.angleBetween(reference));

        if (a_obstacle_angle > b_obstacle_angle) {
            return false;
        }
        else if (a_obstacle_angle < b_obstacle_angle) {
            return true;
        }

        return false;
    });

    for (Segment &obstacle : obstacles) {

        const Point& start = obstacle.getStart();
        const Point& end = obstacle.getEnd();

        Point start_screen_pos = screenPoint(field, start);
        Point end_screen_pos = screenPoint(field, end);

        Vector top_vec = start_screen_pos - shot_origin_screen_pos;
        double top_obstacle_angle = std::abs((180 / M_PI) * top_vec.angleBetween(reference));
        double top_obstacle_cartesian_angle = (start - shot_origin).orientation().toDegrees();


        Vector bottom_vec = end_screen_pos - shot_origin_screen_pos;
        double bottom_obstacle_angle = std::abs((180 / M_PI) * bottom_vec.angleBetween(reference));
        double bottom_obstacle_cartesian_angle = (end - shot_origin).orientation().toDegrees();

        if ((bottom_obstacle_angle > angleMap.getAngleMax() && bottom_obstacle_angle < angleMap.getAngleMin())  || (top_obstacle_angle < angleMap.getAngleMin() && top_obstacle_angle > angleMap.getAngleMax())) {
            AngleSegment non_viable_angle_seg = AngleSegment(top_obstacle_angle, bottom_obstacle_angle, top_obstacle_cartesian_angle, bottom_obstacle_cartesian_angle);

            angleMap.addNonViableAngleSegment(non_viable_angle_seg);
        }
        else if (bottom_obstacle_angle > angleMap.getAngleMin() && top_obstacle_angle < angleMap.getAngleMax()) {
            AngleSegment non_viable_angle_seg = AngleSegment(top_obstacle_angle, bottom_obstacle_angle, top_obstacle_cartesian_angle, bottom_obstacle_cartesian_angle);
            angleMap.addNonViableAngleSegment(non_viable_angle_seg);
        }
    }

    return std::make_pair(angleMap.getBiggestViableAngleSegment(), std::make_pair(angleMap, obstacles));
}

std::pair<double, double> screenValues(const Field &field, double cartesian_x, double cartesian_y) {
    Rectangle field_boundary = field.fieldBoundary();

    return std::make_pair((cartesian_x - field_boundary.xMin()) * field_boundary.xLength() / (field_boundary.xMax() - field_boundary.xMin()), (cartesian_y - field_boundary.yMax()) * -field_boundary.yLength() / (field_boundary.yMax() - field_boundary.yMin()));
}

Point screenPoint(const Field &field, const Point& cartesian_point) {
    std::pair<double, double> screen_transform = screenValues(field, cartesian_point.x(), cartesian_point.y());
    return Point(screen_transform.first, screen_transform.second);
}