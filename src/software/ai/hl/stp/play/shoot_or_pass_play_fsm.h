#pragma once

#include <include/boost/sml.hpp>

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/offensive_play_fsm.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/logger/logger.h"

struct ShootOrPassPlayFSM
{
    class ShootOrPassState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    explicit ShootOrPassPlayFSM(std::shared_ptr<const PlayConfig> play_config)
        : play_config(play_config),
          crease_defender_tactics({
              std::make_shared<CreaseDefenderTactic>(
                  play_config->getRobotNavigationObstacleConfig()),
              std::make_shared<CreaseDefenderTactic>(
                  play_config->getRobotNavigationObstacleConfig()),
          }),
          offensive_fsm(
              std::make_shared<FSM<OffensivePlayFSM>>(OffensivePlayFSM{play_config}))
    {
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto shoot_or_pass_s = state<ShootOrPassState>;
        const auto update_e        = event<Update>;

        const auto update_shoot_or_pass = [this](auto event) {
            std::get<0>(crease_defender_tactics)
                ->updateControlParams(event.common.world.ball().position(),
                                      CreaseDefenderAlignment::LEFT);
            std::get<1>(crease_defender_tactics)
                ->updateControlParams(event.common.world.ball().position(),
                                      CreaseDefenderAlignment::RIGHT);

            PriorityTacticVector tactics_to_return;

            offensive_fsm->process_event(OffensivePlayFSM::Update(
                OffensivePlayFSM::ControlParams{.num_additional_offensive_tactics = 2},
                PlayUpdate(event.common.world,
                           [&tactics_to_return](PriorityTacticVector new_tactics) {
                               tactics_to_return = new_tactics;
                           })));

            tactics_to_return.emplace_back(
                TacticVector({std::get<0>(crease_defender_tactics),
                              std::get<1>(crease_defender_tactics)}));

            event.common.set_tactics(tactics_to_return);
        };

        const auto done_shoot_or_pass = [this](auto event) {
            return offensive_fsm->is(boost::sml::X);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *shoot_or_pass_s + update_e[done_shoot_or_pass] / update_shoot_or_pass = X,
            shoot_or_pass_s + update_e[!done_shoot_or_pass] / update_shoot_or_pass =
                shoot_or_pass_s);
    }

   private:
    std::shared_ptr<const PlayConfig> play_config;
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics;
    std::shared_ptr<FSM<OffensivePlayFSM>> offensive_fsm;
};
