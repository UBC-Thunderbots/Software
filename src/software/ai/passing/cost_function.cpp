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
                const TbotsProto::PassingConfig& passing_config)
{
    double static_pass_quality =
        getStaticPositionQuality(world.field(), pass.receiverPoint(), passing_config);

    double receiver_not_too_close_rating = ratePassNotTooClose(pass, passing_config);

    double friendly_pass_rating =
        ratePassFriendlyCapability(world.friendlyTeam(), pass, passing_config);

    double pass_forward_rating = ratePassForwardQuality(pass, passing_config);

    double enemy_pass_rating = ratePassEnemyRisk(world.enemyTeam(), pass, passing_config);

    double shoot_pass_rating =
        ratePassShootScore(world.field(), world.enemyTeam(), pass, passing_config);

    return static_pass_quality * receiver_not_too_close_rating * friendly_pass_rating *
           enemy_pass_rating * pass_forward_rating * shoot_pass_rating;
}

double ratePassForwardQuality(const Pass& pass,
                              const TbotsProto::PassingConfig& passing_config)
{
    // Rate receiving positions up the field higher and discourage passes back to
    // friendly half, if the passer is in the enemy half
    return sigmoid(pass.receiverPoint().x(),
                   std::min(0.0, pass.passerPoint().x()) +
                       passing_config.backwards_pass_distance_meters(),
                   4.0);
}

double ratePassNotTooClose(const Pass& pass,
                           const TbotsProto::PassingConfig& passing_config)
{
    // Encourage passes that are not too close to the passer
    return 1 - circleSigmoid(Circle(pass.passerPoint(),
                                    passing_config.receiver_ideal_min_distance_meters()),
                             pass.receiverPoint(), 2.0);
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
    double receiver_not_too_close_rating = ratePassNotTooClose(pass, passing_config);

    double enemy_risk_rating = ratePassEnemyRisk(world.enemyTeam(), pass, passing_config);

    double pass_shoot_rating =
        ratePassShootScore(world.field(), world.enemyTeam(), pass, passing_config);

    return static_recv_quality * receiver_up_field_rating * receiver_not_too_far_rating *
           receiver_not_too_close_rating * enemy_risk_rating * pass_shoot_rating;
}

double rateShot(const Point& shot_origin, const Field& field, const Team& enemy_team,
                const TbotsProto::PassingConfig& passing_config)
{
    auto shot_opt =
        calcBestShotOnGoal(Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()),
                           shot_origin, enemy_team.getAllRobots(), TeamType::ENEMY);

    Angle open_angle_to_goal = Angle::zero();
    if (shot_opt && shot_opt.value().getOpenAngle().abs() > Angle::fromDegrees(0))
    {
        open_angle_to_goal = shot_opt.value().getOpenAngle();
    }

    const double min_ideal_angle =
        passing_config.min_ideal_pass_shoot_goal_open_angle_deg();
    double open_angle_to_goal_score = open_angle_to_goal.toDegrees();

    // Clamp angle to [0, min_ideal_angle], where all angle >=min_ideal_angle are given
    // a score of 1.0.
    open_angle_to_goal_score = std::clamp(open_angle_to_goal_score, 0.0, min_ideal_angle);

    // Linearly scale score to [0.0, 1.0]
    open_angle_to_goal_score = open_angle_to_goal_score / min_ideal_angle;

    // Linearly scale score to [min_pass_shoot_score, 1.0] to stop this cost function
    // from returning a very low score, causing the other cost functions to be ignored.
    return open_angle_to_goal_score;
}

double ratePassShootScore(const Field& field, const Team& enemy_team, const Pass& pass,
                          const TbotsProto::PassingConfig& passing_config)
{
    double shot_score = rateShot(pass.receiverPoint(), field, enemy_team, passing_config);

    // Linearly scale score to [min_pass_shoot_score, 1.0] to stop this cost function
    // from returning a very low score, causing the other cost functions to be ignored.
    return scaleNormalizedRating(shot_score, passing_config.min_pass_shoot_score(), 1.0);
}

double ratePassEnemyRisk(const Team& enemy_team, const Pass& pass,
                         const TbotsProto::PassingConfig& passing_config)
{
    double enemy_receiver_proximity_risk =
        calculateProximityRisk(pass.receiverPoint(), enemy_team, passing_config);
    double intercept_risk = calculateInterceptRisk(enemy_team, pass, passing_config);

    // We want to rate a pass more highly if it is lower risk, so subtract from 1
    return 1 - std::max(intercept_risk, enemy_receiver_proximity_risk);
}

double calculateInterceptRisk(const Team& enemy_team, const Pass& pass,
                              const TbotsProto::PassingConfig& passing_config)
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
                       return calculateInterceptRisk(robot, pass, passing_config);
                   });
    return *std::max_element(enemy_intercept_risks.begin(), enemy_intercept_risks.end());
}

double calculateInterceptRisk(const Robot& enemy_robot, const Pass& pass,
                              const TbotsProto::PassingConfig& passing_config)
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
    double min_interception_distance =
        std::max(0.0, enemy_interception_vector.length() - ROBOT_MAX_RADIUS_METERS);

    const double ENEMY_ROBOT_INTERCEPTION_SPEED_METERS_PER_SECOND = 0.5;
    double signed_1d_enemy_vel =
        enemy_robot.velocity().dot(enemy_interception_vector.normalize());
    double enemy_robot_time_to_interception_point_sec =
        getTimeToTravelDistance(
            min_interception_distance, ENEMY_ROBOT_MAX_SPEED_METERS_PER_SECOND,
            ENEMY_ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, signed_1d_enemy_vel,
            ENEMY_ROBOT_INTERCEPTION_SPEED_METERS_PER_SECOND)
            .toSeconds();
    // Scale the time to interception point by the enemy robot's interception capability
    Duration enemy_robot_time_to_interception_point =
        Duration::fromSeconds(enemy_robot_time_to_interception_point_sec *
                              passing_config.enemy_interception_capability());

    Duration ball_time_to_interception_point =
        Duration::fromSeconds(distance(pass.passerPoint(), closest_interception_point) /
                              pass.speed()) +
        Duration::fromSeconds(passing_config.pass_delay_sec());

    Duration interception_delta_time =
        ball_time_to_interception_point - enemy_robot_time_to_interception_point;

    // Whether or not the enemy will be able to intercept the pass can be determined
    // by whether or not they will be able to reach the pass receive position before
    // the pass does.
    return std::clamp(interception_delta_time.toSeconds() *
                          passing_config.enemy_interception_risk_importance(),
                      0.0, 1.0);
}

double ratePassFriendlyCapability(const Team& friendly_team, const Pass& pass,
                                  const TbotsProto::PassingConfig& passing_config)
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
    Duration ball_travel_time =
        Duration::fromSeconds((pass.receiverPoint() - pass.passerPoint()).length() /
                              pass.speed()) +
        Duration::fromSeconds(passing_config.pass_delay_sec());
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
    Timestamp latest_time_to_receiver_state =
        std::max(earliest_time_to_receive_angle, earliest_time_to_receive_point);

    // Create a sigmoid that goes to 0 as the time required to get to the reception
    // point exceeds the time we would need to get there by
    double sigmoid_width = 0.4;
    double time_to_receiver_state_slack_s =
        passing_config.friendly_time_to_receive_slack_sec();

    return sigmoid(
        receive_time.toSeconds(),
        latest_time_to_receiver_state.toSeconds() + time_to_receiver_state_slack_s,
        sigmoid_width);
}

double getStaticPositionQuality(const Field& field, const Point& position,
                                const TbotsProto::PassingConfig& passing_config)
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
                              const TbotsProto::PassingConfig& passing_config)
{
    // Calculate a risk score based on the distance of the enemy robots from the given
    // point, based on an exponential function of the distance of each robot from the
    // receiver point
    if (enemy_team.getAllRobots().empty())
    {
        return 0;
    }

    double risk = 0;
    for (const Robot& enemy : enemy_team.getAllRobots())
    {
        double dist_to_enemy =
            std::max(0.0, distance(point, enemy.position()) - ROBOT_MAX_RADIUS_METERS);
        risk += std::exp((-dist_to_enemy * dist_to_enemy) /
                         passing_config.enemy_proximity_importance());
    }
    return sigmoid(risk, 1, 2);
}

double rateKeepAwayPosition(const Point& keep_away_position, const World& world,
                            const Pass& best_pass_so_far,
                            const Rectangle& dribbling_bounds,
                            const TbotsProto::PassingConfig& passing_config)
{
    static constexpr auto KEEPAWAY_SEARCH_CIRCLE_RADIUS = 0.5;

    // the width of both the field boundary sigmoid and the circular search region sigmoid
    static constexpr auto SIGMOID_WIDTH = 0.1;

    // the region to which the optimization is (effectively) constrained to
    Circle keepaway_search_region(world.ball().position(), KEEPAWAY_SEARCH_CIRCLE_RADIUS);

    Pass updated_best_pass(keep_away_position, best_pass_so_far.receiverPoint(),
                           best_pass_so_far.speed());

    double enemy_receiver_proximity_risk =
        calculateProximityRisk(keep_away_position, world.enemyTeam(), passing_config);
    double intercept_risk =
        calculateInterceptRisk(world.enemyTeam(), updated_best_pass, passing_config);
    // We want to rate a keep away position more highly if it is lower risk, so subtract
    // from 1
    double combined_score = 1 - std::max(intercept_risk, enemy_receiver_proximity_risk);

    return combined_score *
           // constrain the optimization to a circular area around the ball
           circleSigmoid(keepaway_search_region, keep_away_position, SIGMOID_WIDTH) *
           // don't try to dribble the ball off the field
           rectangleSigmoid(dribbling_bounds, keep_away_position, SIGMOID_WIDTH);
}

double scaleNormalizedRating(double rating, double min, double max)
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
    double enemy_proximity_cost;
    double pass_shoot_score_costs;
    double receiver_position_costs;
    double keep_away_position_costs;
    double pass_forward_costs;

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
            enemy_proximity_cost           = 1;
            pass_shoot_score_costs         = 1;
            receiver_position_costs        = 1;
            keep_away_position_costs       = 1;
            pass_forward_costs             = 1;

            // getStaticPositionQuality
            if (passing_config.cost_vis_config().static_position_quality())
            {
                static_pos_quality_costs = getStaticPositionQuality(
                    world.field(), pass.receiverPoint(), passing_config);
            }

            // ratePassForwardQuality
            if (passing_config.cost_vis_config().pass_forward_quality())
            {
                pass_forward_costs = ratePassForwardQuality(pass, passing_config);
            }

            // ratePassNotTooClose
            if (passing_config.cost_vis_config().pass_not_too_close_quality())
            {
                pass_forward_costs = ratePassNotTooClose(pass, passing_config);
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
                pass_enemy_risk_costs =
                    ratePassEnemyRisk(world.enemyTeam(), pass, passing_config);
            }

            // ratePassShootScore
            if (passing_config.cost_vis_config().pass_shoot_score())
            {
                pass_shoot_score_costs = ratePassShootScore(
                    world.field(), world.enemyTeam(), pass, passing_config);
            }

            // calculateProximityRisk
            if (passing_config.cost_vis_config().enemy_proximity_risk())
            {
                enemy_proximity_cost =
                    calculateProximityRisk(curr_point, world.enemyTeam(), passing_config);
            }

            // rateReceivingPosition
            if (passing_config.cost_vis_config().receiver_position_score())
            {
                receiver_position_costs =
                    rateReceivingPosition(world, pass, passing_config);
            }

            // rateKeepAwayPosition
            if (passing_config.cost_vis_config().passer_position_score() &&
                best_pass_so_far.has_value())
            {
                keep_away_position_costs =
                    rateKeepAwayPosition(curr_point, world, best_pass_so_far.value(),
                                         world.field().fieldBoundary(), passing_config);
            }

            costs.push_back(static_pos_quality_costs * pass_friendly_capability_costs *
                            pass_enemy_risk_costs * enemy_proximity_cost *
                            pass_shoot_score_costs * receiver_position_costs *
                            keep_away_position_costs * pass_forward_costs);
        }
    }

    LOG(VISUALIZE) << *createCostVisualization(costs, num_rows, num_cols);
}
