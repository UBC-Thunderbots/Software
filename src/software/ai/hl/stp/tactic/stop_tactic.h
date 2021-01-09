#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"

struct StopTacticFSM
{
    struct Stop
    { /* state */
    };

    struct Update
    {
        bool coast;
        TacticUpdate common;
    };

    auto operator()()
    {
        using namespace boost::sml;

        const auto update_move_intent = [](auto event) {
            event.common.set_intent(
                std::make_unique<StopIntent>(event.common.robot.id(), event.coast));
        };

        const auto stop_done = [](auto event) {
            return robotStopped(event.common.robot);
        };

        return make_transition_table(
            *"idle"_s + event<Update> / update_move_intent = state<Stop>,
            state<Stop> + event<Update>[!stop_done] / update_move_intent,
            state<Stop> + event<Update>[stop_done] = X);
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

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers all robots
     * equally
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double cost(const Robot& robot, const World& world) const override;
    void accept(TacticVisitor& visitor) const override;
    bool done() const override;

   private:
    void updateIntent(const TacticUpdate& tactic_update) override;

    boost::sml::sm<StopTacticFSM> fsm;

    // Whether or not the robot should coast to a stop
    bool coast;
};
