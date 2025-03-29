#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/evaluation/keep_away.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"

struct KeepAwayFSM
{
    /**
   * Constructor for KeepAwayFSM
   *
   * @param ai_config The config to fetch parameters from
   */
    explicit KeepAwayFSM(const TbotsProto::AiConfig& ai_config) : ai_config(ai_config) {};

    struct ControlParams
    {
      std::optional<Pass> best_pass_so_far;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
    * Action that updates the DribbleFSM to keep the ball away
    *
    * @param event KeepAwayFSM::Update event
    * @param processEvent processes the DribbleFSM::Update
    */
    void keepAway(const Update& event,
                  boost::sml::back::process<DribbleFSM::Update> processEvent);

    auto operator ()() {
      using namespace boost:: sml;
      DEFINE_SML_EVENT(Update)
      DEFINE_SML_STATE(DribbleFSM)
      DEFINE_SML_SUB_FSM_UPDATE_ACTION(keepAway, DribbleFSM)

      return make_transition_table(
       *DribbleFSM_S + Update_E / keepAway_A,
       DribbleFSM_S = X,
       X + Update_E / SET_STOP_PRIMITIVE_ACTION = X
      );
    }

    private:
        TbotsProto::AiConfig ai_config;
};
