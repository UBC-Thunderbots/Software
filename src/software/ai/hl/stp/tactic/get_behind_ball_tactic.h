#pragma once

#include "software/ai/hl/stp/action/move_action.h"  // TODO (#1888): remove this dependency
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/triangle.h"

struct GetBehindBallFSM
{
    class idle_state;
    class get_behind_ball_state;

    struct ControlParams
    {
        // The location where the chick will be taken, i.e. where we expect the ball to be
        // when we chip or kick it
        Point ball_location;
        // The direction the Robot will chick in
        Angle chick_direction;
    };

    struct Update
    {
        /* event */
        ControlParams control_params;
        TacticUpdate common;
    };

    auto operator()()
    {
        using namespace boost::sml;

        const auto idle_s            = state<idle_state>;
        const auto get_behind_ball_s = state<get_behind_ball_state>;
        const auto update_e          = event<Update>;

        // How large the triangle is that defines the region where the robot is
        // behind the chick origin and ready to chip or kick.
        // We want to keep the region small enough that we won't use the
        // Chip/KickIntent from too far away (since the Chip/KickIntent doesn't avoid
        // obstacles and we risk colliding with something), but large enough we can
        // reasonably get in the region and chip/kick the ball successfully. This
        // value is 'X' in the ASCII art below
        double size_of_region_behind_ball = 4 * ROBOT_MAX_RADIUS_METERS;

        // ASCII art showing the region behind the ball
        // Diagram not to scale
        //
        //                 X
        //          v-------------v
        //
        //       >  B-------------C
        //       |   \           /
        //       |    \         /
        //       |     \       /     <- Region considered "behind chick origin"
        //     X |      \     /
        //       |       \   /
        //       |        \ /
        //                 A    <  The chick origin is at A
        //                 |
        //                 V
        //         direction of chip/kick

        const auto update_move = [size_of_region_behind_ball](auto event) {
            Vector behind_ball = Vector::createFromAngle(
                event.control_params.chick_direction + Angle::half());
            Point point_behind_ball =
                event.control_params.ball_location +
                behind_ball.normalize(size_of_region_behind_ball * 3 / 4);
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), point_behind_ball,
                event.control_params.chick_direction, 0.0, DribblerMode::OFF,
                BallCollisionType::AVOID));
        };

        const auto behind_ball = [size_of_region_behind_ball](auto event) {
            // A vector in the direction opposite the chip (behind the ball)
            Vector behind_ball = Vector::createFromAngle(
                event.control_params.chick_direction + Angle::half());


            // The points below make up the triangle that defines the region we treat as
            // "behind the ball". They correspond to the vertices labeled 'A', 'B', and
            // 'C' in the ASCII diagram

            // We make the region close enough to the ball so that the robot will still be
            // inside it when taking the chip.
            Point behind_ball_vertex_A = event.control_params.ball_location;
            Point behind_ball_vertex_B =
                behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) +
                behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2);
            Point behind_ball_vertex_C =
                behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) -
                behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2);

            Triangle behind_ball_region = Triangle(
                behind_ball_vertex_A, behind_ball_vertex_B, behind_ball_vertex_C);

            return contains(behind_ball_region, event.common.robot.position());
        };

        return make_transition_table(
            *idle_s + update_e / update_move = get_behind_ball_s,
            get_behind_ball_s + update_e[!behind_ball] / update_move,
            get_behind_ball_s + update_e[behind_ball] / update_move = X,
            X + update_e[behind_ball] / update_move);
    }

   private:
};

/**
 * The GetBehindBallTactic will move the assigned robot to the given destination and
 * arrive with the specified final orientation and speed
 */
class GetBehindBallTactic : public Tactic
{
   public:
    /**
     * Creates a new GetBehindBallTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit GetBehindBallTactic(bool loop_forever);

    GetBehindBallTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the control parameters for this GetBehindBallTactic.
     *
     * @param ball_location The location of the ball when it will be chipped or kicked
     * @param chick_direction The direction to kick or chip
     */
    void updateControlParams(const Point& ball_location, Angle chick_direction);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    void accept(TacticVisitor& visitor) const override;
    bool done() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;
    void updateIntent(const TacticUpdate& tactic_update) override;

    boost::sml::sm<GetBehindBallFSM> fsm;

    GetBehindBallFSM::ControlParams control_params;
};
