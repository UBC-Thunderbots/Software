#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

struct MoveTestPlayFSM : PlayFSM<MoveTestPlayFSM>
{
    struct ControlParams
    {
    };

    class MoveTestState;

    explicit MoveTestPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
        : PlayFSM<MoveTestPlayFSM>(ai_config_ptr),
          move_test_tactic_friendly_goal(std::make_shared<MoveTactic>(ai_config_ptr)),
          move_test_tactic_enemy_goal(std::make_shared<MoveTactic>(ai_config_ptr)),
          move_test_tactic_center_field(std::make_shared<MoveTactic>(ai_config_ptr))
    {
    }

    void updateMove(const Update& event)
    {
        move_test_tactic_friendly_goal->updateControlParams(
            event.common.world_ptr->field().friendlyGoalCenter(), Angle::zero(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);
        move_test_tactic_enemy_goal->updateControlParams(
            event.common.world_ptr->field().enemyGoalCenter(), Angle::zero(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);
        move_test_tactic_center_field->updateControlParams(
            Point(0, 0), Angle::zero(), TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);

        event.common.set_tactics(
            {{move_test_tactic_center_field, move_test_tactic_friendly_goal,
              move_test_tactic_enemy_goal}});
    }

    bool moveDone(const Update& event)
    {
        return move_test_tactic_center_field->done();
    }


    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveTestState)
        DEFINE_SML_EVENT(Update)
        DEFINE_SML_ACTION(updateMove)
        DEFINE_SML_GUARD(moveDone)

        return make_transition_table(
            *MoveTestState_S + Update_E[!moveDone_G] / updateMove_A = MoveTestState_S,
            MoveTestState_S + Update_E[moveDone_G] / updateMove_A   = X,
            X + Update_E / updateMove_A                             = X);
    }

   private:
    std::shared_ptr<MoveTactic> move_test_tactic_friendly_goal;
    std::shared_ptr<MoveTactic> move_test_tactic_enemy_goal;
    std::shared_ptr<MoveTactic> move_test_tactic_center_field;
};
