#include "software/ai/passing/cost_function.h"

#include <numeric>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "software/../shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/time_to_travel.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"

double ratePass(const World& world, const Pass& pass,
                TbotsProto::PassingConfig passing_config)
{
    double static_pass_quality =
        getStaticPositionQuality(world.field(), pass.receiverPoint(), passing_config);

    double friendly_pass_rating =
        ratePassFriendlyCapability(world.friendlyTeam(), pass, passing_config);

    double pass_forward_rating = ratePassForwardQuality(pass, passing_config);

    double enemy_pass_rating =
        ratePassEnemyRisk(world.enemyTeam(), pass,
                          passing_config.enemy_proximity_importance());

    double shoot_pass_rating =
        ratePassShootScore(world.field(), world.enemyTeam(), pass, passing_config);

    return static_pass_quality * friendly_pass_rating * enemy_pass_rating *
           pass_forward_rating * shoot_pass_rating;
}

double ratePass(const World& world, const Pass& pass, const Rectangle& zone,
                TbotsProto::PassingConfig passing_config)
{
    double in_region_quality = rectangleSigmoid(zone, pass.receiverPoint(), 0.2);

    return ratePass(world, pass, passing_config) * in_region_quality;
}

double ratePassForwardQuality(const Pass& pass,
                              const TbotsProto::PassingConfig& passing_config)
{
    // Rate receiving positions up the field higher and discourage passes back to
    // friendly half if the passer is in the enemy half
    return sigmoid(pass.receiverPoint().x(),
                   std::min(0.0, pass.passerPoint().x() +
                                     passing_config.backwards_pass_distance_meters()),
                   4.0);
}

double rateZone(const Field& field, const Team& enemy_team, const Rectangle& zone,
                const Point& ball_position, TbotsProto::PassingConfig passing_config)
{
    // TODO (#2021) improve and implement tests
    // Zones with their centers in bad positions are not good
    double static_pass_quality =
        getStaticPositionQuality(field, zone.centre(), passing_config);

    // Rate zones that are up the field higher to encourage progress up the field
    double pass_up_field_rating = zone.centre().x() / field.xLength();

    double enemy_proximity_importance = passing_config.enemy_proximity_importance();

    double enemy_risk_rating =
        (ratePassEnemyRisk(enemy_team,
                           Pass(ball_position, zone.negXNegYCorner(),
                                passing_config.max_pass_speed_m_per_s()),
                           enemy_proximity_importance) +
         ratePassEnemyRisk(enemy_team,
                           Pass(ball_position, zone.negXPosYCorner(),
                                passing_config.max_pass_speed_m_per_s()),
                           enemy_proximity_importance) +
         ratePassEnemyRisk(enemy_team,
                           Pass(ball_position, zone.posXNegYCorner(),
                                passing_config.max_pass_speed_m_per_s()),
                           enemy_proximity_importance) +
         ratePassEnemyRisk(enemy_team,
                           Pass(ball_position, zone.posXPosYCorner(),
                                passing_config.max_pass_speed_m_per_s()),
                           enemy_proximity_importance) +
         ratePassEnemyRisk(
             enemy_team,
             Pass(ball_position, zone.centre(), passing_config.max_pass_speed_m_per_s()),
             enemy_proximity_importance)) /
        5.0;

    return pass_up_field_rating * static_pass_quality * enemy_risk_rating;
}

double rateReceivingPosition(const World& world, const Pass& pass,
                             const TbotsProto::PassingConfig& passing_config)
{
    double static_recv_quality =
        getStaticPositionQuality(world.field(), pass.receiverPoint(), passing_config);

    double receiver_up_field_rating = ratePassForwardQuality(pass, passing_config);

    // We want to encourage passes that are not too far away from the passer
    // to stop the robots from trying to pass across the field
    double receiver_not_too_far_rating = circleSigmoid(
        Circle(pass.passerPoint(), passing_config.receiver_ideal_max_distance_meters()),
        pass.receiverPoint(), 2.0);
    double receiver_not_too_close_rating =
        1 - circleSigmoid(Circle(pass.passerPoint(),
                                 passing_config.receiver_ideal_min_distance_meters()),
                          pass.receiverPoint(), 2.0);

    double enemy_proximity_importance = passing_config.enemy_proximity_importance();
    double enemy_risk_rating          = ratePassEnemyRisk(
        world.enemyTeam(), pass, enemy_proximity_importance);

    double pass_shoot_rating =
        ratePassShootScore(world.field(), world.enemyTeam(), pass, passing_config);

    return static_recv_quality * receiver_up_field_rating * receiver_not_too_far_rating *
           receiver_not_too_close_rating * enemy_risk_rating * pass_shoot_rating;
}

double ratePassShootScore(const Field& field, const Team& enemy_team, const Pass& pass,
                          TbotsProto::PassingConfig passing_config)
{
    // TODO (NIMA):
    //  This function should also take give worst scores to really shart shooting angles.
    //  It should also use open angle, rather than net open angle?!
    //  Consider taking into account the deflection angle as well


    //    double ideal_max_rotation_to_shoot_degrees =
    //        passing_config.ideal_max_rotation_to_shoot_degrees();

    // Figure out the range of angles for which we have an open shot to the goal after
    // receiving the pass
    auto shot_opt = calcBestShotOnGoal(
        Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()), pass.receiverPoint(),
        enemy_team.getAllRobots(), TeamType::ENEMY);

    Angle open_angle_to_goal = Angle::zero();
    //    Point shot_target        = field.enemyGoalCenter();
    if (shot_opt && shot_opt.value().getOpenAngle().abs() > Angle::fromDegrees(0))
    {
        open_angle_to_goal = shot_opt.value().getOpenAngle();
        //        shot_target = shot_opt.value().getPointToShootAt();
    }

    // Figure out what the maximum open angle of the goal could be from the receiver pos.
    Angle goal_angle        = convexAngle(field.enemyGoalpostNeg(), pass.receiverPoint(),
                                   field.enemyGoalpostPos());
    double net_percent_open = 0;
    if (goal_angle > Angle::zero())
    {
        net_percent_open = open_angle_to_goal.toDegrees() / goal_angle.toDegrees();
    }

    double open_angle_to_goal_score = open_angle_to_goal.toDegrees();
    // Clamp to [0, 10]
    open_angle_to_goal_score = std::clamp(open_angle_to_goal_score, 0.0, 10.0); // TODO (NIMA): Make 30 a cosntants
    // Linearly scale to [0.0, 1.0]
    open_angle_to_goal_score = open_angle_to_goal_score / 10;

    // Create the shoot score by creating a sigmoid that goes to a large value as
    // the section of net we're shooting on approaches 100% (ie. completely open)
    double shot_openness_score = sigmoid(net_percent_open, 0.45, 0.1);

    // Prefer angles where the robot does not have to turn much after receiving the
    // pass to take the shot (or equivalently the shot deflection angle)
    //
    // Receiver robots on the friendly side, almost always, need to rotate a full 180
    // degrees to shoot on net. So we relax that requirement for both receiver and ball
    // locations on the friendly side
    //
    // TODO (#1987) This creates a very steep slope, find a better way to do this
    //    if (pass.receiverPoint().x() < 0 || pass.passerPoint().x() < 0)
    //    {
    //        ideal_max_rotation_to_shoot_degrees = 180;
    //    }
    //    Angle rotation_to_shot_target_after_pass = pass.receiverOrientation().minDiff(
    //        (shot_target - pass.receiverPoint()).orientation());
    double required_rotation_for_shot_score =
        1.0;  // TODO (NIMA): I don't know if we should take into account the orientation
              //        1 - sigmoid(rotation_to_shot_target_after_pass.abs().toDegrees(),
    //                    ideal_max_rotation_to_shoot_degrees, 4); // 150) * 0.8; // TODO
    //                    (NIMA): Add to config: lowerst 0.8

    double rating = shot_openness_score * required_rotation_for_shot_score * open_angle_to_goal_score;
    return scaleRating(rating, passing_config.min_pass_shoot_score(), 1.0);
}

double ratePassEnemyRisk(const Team& enemy_team, const Pass& pass,
                         double enemy_proximity_importance)
{
    double enemy_receiver_proximity_risk = calculateProximityRisk(
        pass.receiverPoint(), enemy_team, enemy_proximity_importance);
    double intercept_risk = calculateInterceptRisk(enemy_team, pass);

    // We want to rate a pass more highly if it is lower risk, so subtract from 1
    return 1 - std::max(intercept_risk, enemy_receiver_proximity_risk);
}

double calculateInterceptRisk(const Team& enemy_team, const Pass& pass)
{
    // Return the highest risk for all the enemy robots, if there are any
    const std::vector<Robot>& enemy_robots = enemy_team.getAllRobots();
    if (enemy_robots.empty())
    {
        return 0;
    }
    std::vector<double> enemy_intercept_risks(enemy_robots.size());
    std::transform(enemy_robots.begin(), enemy_robots.end(),
                   enemy_intercept_risks.begin(), [&](const Robot& robot) {
                       return calculateInterceptRisk(robot, pass);
                   });
    return *std::max_element(enemy_intercept_risks.begin(), enemy_intercept_risks.end());
}

double calculateInterceptRisk(const Robot& enemy_robot, const Pass& pass)
{
    // Return early to avoid division by zero
    if (pass.speed() == 0)
    {
        return 1.0;
    }

    // We estimate the intercept by the risk that the enemy robot will get to the closest
    // point on the pass before the ball
    Point closest_interception_point = closestPoint(
        enemy_robot.position(), Segment(pass.passerPoint(), pass.receiverPoint()));
    Vector enemy_interception_vector =
        closest_interception_point - enemy_robot.position();
    // Take into account the enemy robot's radius for minimum min_interception_distance
    // required to travel to intercept the pass.
    double min_interception_distance = std::max(
        0.0, enemy_interception_vector.length() -
                 ROBOT_MAX_RADIUS_METERS);  // TODO (NIMA): It is potentially faster to
                                            // travel to +radius than -radius

    const double ENEMY_ROBOT_INTERCEPTION_SPEED_METERS_PER_SECOND = 0.5;
    double signed_1d_enemy_vel =
        enemy_robot.velocity().dot(enemy_interception_vector.normalize());
    Duration enemy_robot_time_to_interception_point = getTimeToTravelDistance(
        min_interception_distance, ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
        ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, signed_1d_enemy_vel,
        ENEMY_ROBOT_INTERCEPTION_SPEED_METERS_PER_SECOND);

    Duration ball_time_to_interception_point = Duration::fromSeconds(
        distance(pass.passerPoint(), closest_interception_point) / pass.speed());

    Duration interception_delta_time =
        ball_time_to_interception_point - enemy_robot_time_to_interception_point;

    // Whether or not the enemy will be able to intercept the pass can be determined
    // by whether or not they will be able to reach the pass receive position before
    // the pass does. As such, we place the time difference between the robot and ball
    // on a sigmoid that is centered at 0, and goes to 1 at positive values, 0 at
    // negative values.
    return std::clamp(interception_delta_time.toSeconds() * 4.0, 0.0,
                      1.0);  // 1 - std::min(min_interception_distance * 5, 1.0);  // 1 -
                             // sigmoid(min_time_to_interception, 0, 0.1); // TODO (NIMA):
                             // Test this value
}

double ratePassFriendlyCapability(const Team& friendly_team, const Pass& pass,
                                  TbotsProto::PassingConfig passing_config)
{
    // We need at least one robot to pass to
    if (friendly_team.getAllRobots().empty())
    {
        return 0;
    }

    // Special case where pass speed is 0
    if (pass.speed() == 0)
    {
        return 0;
    }

    // Get the robot that is closest to where the pass would be received
    Robot best_receiver = friendly_team.getAllRobots()[0];
    for (const Robot& robot : friendly_team.getAllRobots())
    {
        double distance = (robot.position() - pass.receiverPoint()).length();
        double curr_best_distance =
            (best_receiver.position() - pass.receiverPoint()).length();
        if (distance < curr_best_distance)
        {
            best_receiver = robot;
        }
    }

    // Figure out what time the robot would have to receive the ball at
    // TODO (#2988): We should generate a more realistic ball trajectory
    Duration ball_travel_time = Duration::fromSeconds(
        (pass.receiverPoint() - pass.passerPoint()).length() / pass.speed());
    Timestamp receive_time = best_receiver.timestamp() + ball_travel_time;

    // Figure out how long it would take our robot to get there
    Duration min_robot_travel_time =
        best_receiver.getTimeToPosition(pass.receiverPoint());
    Timestamp earliest_time_to_receive_point =
        best_receiver.timestamp() + min_robot_travel_time;

    // Figure out what angle the robot would have to be at to receive the ball
    Angle receive_angle = (pass.passerPoint() - best_receiver.position()).orientation();
    Duration time_to_receive_angle = best_receiver.getTimeToOrientation(receive_angle);
    Timestamp earliest_time_to_receive_angle =
        best_receiver.timestamp() + time_to_receive_angle;

    // Figure out if rotation or moving will take us longer
    Timestamp latest_time_to_reciever_state =
        std::max(earliest_time_to_receive_angle, earliest_time_to_receive_point);

    // Create a sigmoid that goes to 0 as the time required to get to the reception
    // point exceeds the time we would need to get there by
    double sigmoid_width                  = 0.4;
    double time_to_receiver_state_slack_s = 0.25;

    return sigmoid(
        receive_time.toSeconds(),
        latest_time_to_reciever_state.toSeconds() + time_to_receiver_state_slack_s,
        sigmoid_width);
}

double getStaticPositionQuality(const Field& field, const Point& position,
                                TbotsProto::PassingConfig passing_config)
{
    // This constant is used to determine how steep the sigmoid slopes below are
    static const double sig_width = 0.1;

    // The offset from the sides of the field for the center of the sigmoid functions
    double x_offset = passing_config.static_field_position_quality_x_offset();
    double y_offset = passing_config.static_field_position_quality_y_offset();
    double friendly_goal_weight =
        passing_config.static_field_position_quality_friendly_goal_distance_weight();

    // Make a slightly smaller field, and positive weight values in this reduced field
    double half_field_length = field.xLength() / 2;
    double half_field_width  = field.yLength() / 2;
    Rectangle reduced_size_field(
        Point(-half_field_length + x_offset, -half_field_width + y_offset),
        Point(half_field_length - x_offset, half_field_width - y_offset));
    double on_field_quality = rectangleSigmoid(reduced_size_field, position, sig_width);

    // Add a negative weight for positions closer to our goal
    Vector vec_to_friendly_goal = Vector(field.friendlyGoalCenter().x() - position.x(),
                                         field.friendlyGoalCenter().y() - position.y());
    double distance_to_friendly_goal = vec_to_friendly_goal.length();
    double near_friendly_goal_quality =
        (1 -
         std::exp(-friendly_goal_weight * (std::pow(5, -2 + distance_to_friendly_goal))));

    // Add a strong negative weight for positions within the enemy defense area, as we
    // cannot pass there
    double in_enemy_defense_area_quality =
        1 - rectangleSigmoid(field.enemyDefenseArea(), position, sig_width);

    return on_field_quality * near_friendly_goal_quality * in_enemy_defense_area_quality;
}

double calculateProximityRisk(const Point& point, const Team& enemy_team,
                              double enemy_proximity_importance)
{
    // Calculate a risk score based on the distance of the enemy robots from the receive
    // point, based on an exponential function of the distance of each robot from the
    // receiver point
    auto enemy_robots                 = enemy_team.getAllRobots();
    double point_enemy_proximity_risk = 1;
    for (const Robot& enemy : enemy_team.getAllRobots())
    {
        double dist = (point - enemy.position()).length();
        point_enemy_proximity_risk *= enemy_proximity_importance * std::exp(-dist * dist);
    }
    if (enemy_robots.empty())
    {
        point_enemy_proximity_risk = 0;
    }
    return point_enemy_proximity_risk;
}

double ratePasserPointForKeepAway(const Pass& pass, const Team& enemy_team)
{
    // the default values for these passing parameters
    // TODO: cleanup passing parameters as part of #1987
    static constexpr double PASSER_ENEMY_PROXIMITY_IMPORTANCE   = 1.5;
    static constexpr double RECEIVER_ENEMY_PROXIMITY_IMPORTANCE = 0.;

    return ratePassEnemyRisk(enemy_team, pass,
                             RECEIVER_ENEMY_PROXIMITY_IMPORTANCE) *
           (1 - calculateProximityRisk(pass.passerPoint(), enemy_team,
                                       PASSER_ENEMY_PROXIMITY_IMPORTANCE));
}

double ratePasserPosition(const World& world, const Pass& pass,
                          const Rectangle& dribbling_bounds)
{
    static constexpr auto KEEPAWAY_SEARCH_CIRCLE_RADIUS = 0.5;

    // the width of both the field boundary sigmoid and the circular search region sigmoid
    static constexpr auto SIGMOID_WIDTH = 0.1;

    // the region to which the optimization is (effectively) constrained to
    Circle keepaway_search_region(world.ball().position(), KEEPAWAY_SEARCH_CIRCLE_RADIUS);

    return ratePasserPointForKeepAway(pass, world.enemyTeam()) *
           // constrain the optimization to a circular area around the ball
           circleSigmoid(keepaway_search_region, pass.passerPoint(), SIGMOID_WIDTH) *
           // don't try to dribble the ball off the field
           rectangleSigmoid(dribbling_bounds, pass.passerPoint(), SIGMOID_WIDTH);
}

double scaleRating(double rating, double min, double max)
{
    return rating * (max - min) + min;
}

void samplePassesForVisualization(const World& world,
                                  const TbotsProto::PassingConfig& passing_config,
                                  const std::optional<Pass>& best_pass_so_far)
{
    // number of rows and columns are configured in parameters.proto
    int num_cols = passing_config.cost_vis_config().num_cols();
    // this is for DivB field, for DivA, it would be num_cols * 3 / 4 as specified in
    // field.cpp
    int num_rows  = num_cols * 2 / 3;
    double width  = world.field().xLength() / num_cols;
    double height = world.field().yLength() / num_rows;

    std::vector<double> costs;
    double static_pos_quality_costs;
    double pass_friendly_capability_costs;
    double pass_enemy_risk_costs;
    double pass_shoot_score_costs;
    double receiver_position_costs;

    // We loop column wise (in the same order as how zones are defined)
    for (int i = 0; i < num_cols; i++)
    {
        // x coordinate of the centre of the column
        double x = width * i + width / 2 - world.field().xLength() / 2;
        for (int j = 0; j < num_rows; j++)
        {
            // y coordinate of the centre of the row
            double y = height * j + height / 2 - world.field().yLength() / 2;
            Point curr_point(x, y);
            auto pass = Pass::fromDestReceiveSpeed(world.ball().position(), curr_point,
                                                   passing_config);

            // default values
            static_pos_quality_costs       = 1;
            pass_friendly_capability_costs = 1;
            pass_enemy_risk_costs          = 1;
            pass_shoot_score_costs         = 1;
            receiver_position_costs        = 1;

            // getStaticPositionQuality
            if (passing_config.cost_vis_config().static_position_quality())
            {
                static_pos_quality_costs = getStaticPositionQuality(
                    world.field(), pass.receiverPoint(), passing_config);
            }

            // ratePassFriendlyCapability
            if (passing_config.cost_vis_config().pass_friendly_capability())
            {
                pass_friendly_capability_costs = ratePassFriendlyCapability(
                    world.friendlyTeam(), pass, passing_config);
            }

            // ratePassEnemyRisk
            if (passing_config.cost_vis_config().pass_enemy_risk())
            {
                pass_enemy_risk_costs = ratePassEnemyRisk(
                    world.enemyTeam(), pass,
                    passing_config.enemy_proximity_importance());
            }

            // ratePassShootScore
            if (passing_config.cost_vis_config().pass_shoot_score())
            {
                pass_shoot_score_costs = ratePassShootScore(
                    world.field(), world.enemyTeam(), pass, passing_config);
            }

            // rateReceivingPosition
            if (passing_config.cost_vis_config().receiver_position_score())
            {
                receiver_position_costs =
                    rateReceivingPosition(world, pass, passing_config);
            }

            // ratePasserPosition
            if (passing_config.cost_vis_config().passer_position_score() &&
                best_pass_so_far.has_value())
            {
                Pass updated_best_pass = Pass::fromDestReceiveSpeed(
                    curr_point, best_pass_so_far->receiverPoint(), passing_config);
                receiver_position_costs = ratePasserPosition(
                    world, updated_best_pass, world.field().fieldBoundary());
            }

            costs.push_back(static_pos_quality_costs * pass_friendly_capability_costs *
                            pass_enemy_risk_costs * pass_shoot_score_costs *
                            receiver_position_costs);
        }
    }

    LOG(VISUALIZE) << *createCostVisualization(costs, num_rows, num_cols);
}
