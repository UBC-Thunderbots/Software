#pragma once

#include "software/ai/hl/stp/action/move_action.h"  // TODO (#1888): remove this dependency
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

struct MoveFSM
{
    /* state classes */
    class idle_state;
    class move_state;

    /* update structs */
    struct ControlParams
    {
        // The point the robot is trying to move to
        Point destination;
        // The orientation the robot should have when it arrives at its destination
        Angle final_orientation;
        // The speed the robot should have when it arrives at its destination
        double final_speed;
    };

    struct Update
    {
        ControlParams control_params;
        TacticUpdate common;
    };

    auto operator()()
    {
        using namespace boost::sml;

        const auto idle_s   = state<idle_state>;
        const auto move_s   = state<move_state>;
        const auto update_e = event<Update>;

        const auto update_move = [](auto event) {
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), event.control_params.destination,
                event.control_params.final_orientation, event.control_params.final_speed,
                DribblerMode::OFF, BallCollisionType::AVOID));
        };

        const auto movement_done = [](auto event) {
            return robotReachedDestination(event.common.robot,
                                           event.control_params.destination,
                                           event.control_params.final_orientation);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *idle_s + update_e / update_move                = move_s,
            move_s + update_e[!movement_done] / update_move = move_s,
            move_s + update_e[movement_done] / update_move  = X,
            X + update_e[movement_done] / update_move       = X);
    }
};

/**
 * The MoveTactic will move the assigned robot to the given destination and arrive
 * with the specified final orientation and speed
 */
class MoveTactic : public Tactic
{
   public:
    /**
     * Creates a new MoveTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit MoveTactic(bool loop_forever);

    MoveTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the control parameters for this MoveTactic.
     *
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     */
    void updateControlParams(Point destination, Angle final_orientation,
                             double final_speed);

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

    boost::sml::sm<MoveFSM> fsm;

    MoveFSM::ControlParams control_params;
};
