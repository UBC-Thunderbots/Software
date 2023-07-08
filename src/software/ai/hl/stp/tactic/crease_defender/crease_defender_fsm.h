#pragma once

#include "proto/parameters.pb.h"
#include "proto/tactic.pb.h"
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
    class ChipAway;
    class Control;

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
        bool is_currently_in_possession;
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
     * @param robot_navigation_obstacle_config The config
     */
    explicit CreaseDefenderFSM(
        TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config)
        : robot_navigation_obstacle_config(robot_navigation_obstacle_config)
    {
    }

    /**
     * This is an Action that blocks the threat
     *
     * @param event CreaseDefenderFSM::Update event
     */
    void blockThreat(const Update& event,
                     boost::sml::back::process<MoveFSM::Update> processEvent);

    bool shouldChipAway(const Update& event);

    bool shouldControl(const Update& event);

    void control(const Update& event);

    void chipAway(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveFSM)
        DEFINE_SML_STATE(ChipAway)
        DEFINE_SML_STATE(Control)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(shouldChipAway)
        DEFINE_SML_GUARD(shouldControl)

        DEFINE_SML_ACTION(control)
        DEFINE_SML_ACTION(chipAway)

        DEFINE_SML_SUB_FSM_UPDATE_ACTION(blockThreat, MoveFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveFSM_S + Update_E[shouldChipAway_G] / chipAway_A = ChipAway_S,
            MoveFSM_S + Update_E[shouldControl_G] / control_A    = Control_S,
            MoveFSM_S + Update_E / blockThreat_A, MoveFSM_S = X,
            ChipAway_S + Update_E[shouldChipAway_G] / chipAway_A = ChipAway_S,
            ChipAway_S + Update_E / blockThreat_A                = MoveFSM_S,
            Control_S + Update_E[shouldChipAway_G] / chipAway_A  = ChipAway_S,
            Control_S + Update_E[shouldControl_G] / control_A    = Control_S,
            Control_S + Update_E / blockThreat_A                 = MoveFSM_S,
            X + Update_E / blockThreat_A                         = MoveFSM_S);
    }

   private:
    /**
     * Finds the intersection with the front or sides of the defense area with the given
     * ray
     *
     * @param field The field that has the friendly defense area
     * @param ray The ray to intersect
     * @param inflated_defense_area The inflated friendly defense area
     *
     * @return the intersection with the front or sides of the defense area, returns
     * std::nullopt if there is no intersection or if the start point of the ray is inside
     * or behind the defense area
     */
    static std::optional<Point> findDefenseAreaIntersection(
        const Field& field, const Ray& ray, const Rectangle& inflated_defense_area);

   private:
    bool enemyCloseToBall(const Update& event);

    TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config;

    static constexpr double ENEMY_THREATS_CLOSE_THRESHOLD_M = 0.15;
    static constexpr double BALL_CLOSE_THRESHOLD_M          = 0.05;

    bool isSafeToChipForward(const Update& event);
};
