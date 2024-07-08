#pragma once

#include "proto/parameters.pb.h"
#include "proto/tactic.pb.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/convex_angle.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/ray.h"
#include "software/logger/logger.h"

struct CreaseDefenderFSM
{
   public:
    // this struct defines the unique control parameters that the CreaseDefenderFSM
    // requires in its update
    struct ControlParams
    {
        // The origin point of the enemy threat
        Point enemy_threat_origin;
        // The crease defender alignment with respect to the enemy threat
        TbotsProto::CreaseDefenderAlignment crease_defender_alignment;
        // The maximum allowed speed mode
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode;
        // The crease defender's aggressiveness towards the ball
        TbotsProto::BallStealMode ball_steal_mode;
    };

    // this struct defines the only event that the CreaseDefenderFSM responds to
    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Finds the point to block the threat
     *
     * @param field The field
     * @param enemy_threat_origin The origin of the threat to defend against
     * @param crease_defender_alignment alignment of the crease defender
     * @param robot_obstacle_inflation_factor The robot obstacle inflation factor
     *
     * @return The best point to block the threat if it exists
     */
    static std::optional<Point> findBlockThreatPoint(
        const Field& field, const Point& enemy_threat_origin,
        const TbotsProto::CreaseDefenderAlignment& crease_defender_alignment,
        double robot_obstacle_inflation_factor);

    /**
     * Constructor for CreaseDefenderFSM struct
     *
     * @param ai_config The ai config for this struct
     */
    explicit CreaseDefenderFSM(const TbotsProto::AiConfig& ai_config)
        : robot_navigation_obstacle_config(ai_config.robot_navigation_obstacle_config()),
          crease_defender_config(ai_config.crease_defender_config())
    {
    }

    /**
     * Guard that checks if the ball is on friendly side, nearby, and unguarded by the
     * enemy (only if stealing is enabled).
     *
     * @param event CreaseDefenderFSM::Update event
     *
     * @return true if the ball is on friendly side, nearby, unguarded by the enemy up,
     *          and within a max get possession threshold
     */
    bool ballNearbyWithoutThreat(const Update& event);

    /**
     * This is the Action that prepares for getting possession of the ball
     * @param event CreaseDefenderFSM::Update event
     * @param processEvent processes the DribbleFSM::Update
     */
    void prepareGetPossession(const Update& event,
                              boost::sml::back::process<DribbleFSM::Update> processEvent);

    /**
     * This is an Action that blocks the threat
     *
     * @param event CreaseDefenderFSM::Update event
     */
    void blockThreat(const Update& event,
                     boost::sml::back::process<MoveFSM::Update> processEvent);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveFSM)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(blockThreat, MoveFSM)
        DEFINE_SML_STATE(DribbleFSM)
        DEFINE_SML_GUARD(ballNearbyWithoutThreat)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(prepareGetPossession, DribbleFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveFSM_S + Update_E[ballNearbyWithoutThreat_G] / prepareGetPossession_A =
                DribbleFSM_S,
            MoveFSM_S + Update_E / blockThreat_A, MoveFSM_S = X,
            DribbleFSM_S + Update_E[!ballNearbyWithoutThreat_G] / blockThreat_A =
                MoveFSM_S,
            DribbleFSM_S + Update_E / prepareGetPossession_A,
            X + Update_E[ballNearbyWithoutThreat_G] / prepareGetPossession_A =
                DribbleFSM_S,
            X + Update_E / blockThreat_A = MoveFSM_S);
    }

   private:
    static constexpr double DETECT_THREAT_AHEAD_SHAPE_LENGTH_M = 1;
    static constexpr double DETECT_THREAT_AHEAD_SHAPE_RADIUS_M = 0.25;
    /**
     * Finds the intersection with the front or sides of the defense area with the given
     * ray
     *
     * @param field The field that has the friendly defense area
     * @param ray The ray to intersect
     * @param robot_obstacle_inflation_factor The robot obstacle inflation factor
     *
     * @return the intersection with the front or sides of the defense area, returns
     * std::nullopt if there is no intersection or if the start point of the ray is inside
     * or behind the defense area
     */
    static std::optional<Point> findDefenseAreaIntersection(
        const Field& field, const Ray& ray, double robot_obstacle_inflation_factor);

    /**
     * Returns true if any enemy robot is within the given zone
     *
     * @param event CreaseDefenderFSM::Update event
     * @param zone a stadium shape that defines the zone
     * @return true if any enemy robot is within the given zone, else false
     */
    static bool isAnyEnemyInZone(const Update& event, const Stadium& zone);

    TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config;
    TbotsProto::CreaseDefenderConfig crease_defender_config;
};
