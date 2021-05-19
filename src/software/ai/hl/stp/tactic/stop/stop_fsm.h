#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"

struct StopFSM
{
   public:
    class StopState;

    struct ControlParams
    {
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Constructor for StopFSM struct
     *
     * @param coast whether or not the StopFSM should coast
     */
    explicit StopFSM(bool coast) : coast(coast) {}

    auto operator()()
    {
        using namespace boost::sml;

        const auto stop_s   = state<StopState>;
        const auto update_e = event<Update>;

        /**
         * Action to set the StopIntent
         *
         * @param event StopFSM::Update
         */
        const auto update_stop = [this](auto event) {
            event.common.set_intent(
                std::make_unique<StopIntent>(event.common.robot.id(), coast));
        };

        /**
         * Guard if the stop is done
         *
         * @param event StopFSM::Update
         *
         * @return if the robot has stopped
         */
        const auto stop_done = [](auto event) {
            return robotStopped(event.common.robot);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *stop_s + update_e[!stop_done] / update_stop = stop_s,
            stop_s + update_e[stop_done] / update_stop   = X,
            X + update_e[!stop_done] / update_stop       = stop_s,
            X + update_e[stop_done] / update_stop        = X);
    }

   private:
    // Whether or not the robot should coast to a stop
    bool coast;
};
