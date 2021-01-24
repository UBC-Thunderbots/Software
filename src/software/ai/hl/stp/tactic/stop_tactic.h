#pragma once

#include "software/ai/hl/stp/action/move_action.h"  // TODO (#1888): remove this dependency
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"

struct StopFSM
{
    /* state structs */
    struct Stop
    {
    };

    /* update structs */
    struct Update
    {
        bool coast;
        TacticUpdate common;
    };

    auto operator()()
    {
        using namespace boost::sml;

        const auto update_stop_intent = [](auto event) {
            event.common.set_intent(
                std::make_unique<StopIntent>(event.common.robot.id(), event.coast));
        };

        const auto stop_done = [](auto event) {
            return robotStopped(event.common.robot);
        };

        return make_transition_table(
            *"idle"_s + event<Update> / update_stop_intent               = state<Stop>,
            state<Stop> + event<Update>[!stop_done] / update_stop_intent = state<Stop>,
            state<Stop> + event<Update>[stop_done] / update_stop_intent  = X,
            X + event<Update>[!stop_done] / update_stop_intent           = X);
    }
};

/**
 * The StopTactic will stop the robot from moving. The robot will actively try and brake
 * to come to a halt unless is it told to coast, in which case it will coast to a stop.
 */
class StopTactic : public Tactic
{
   public:
    /**
     * Creates a new StopTactic
     t
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit StopTactic(bool coast);

    StopTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers all robots
     * equally
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

    boost::sml::sm<StopFSM> fsm;

    // Whether or not the robot should coast to a stop
    bool coast;
};
