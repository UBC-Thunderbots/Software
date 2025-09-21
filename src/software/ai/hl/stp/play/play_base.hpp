#pragma once

#include "software/ai/hl/stp/play/play.h"

/**
 * A base play class for all other plays based on FSMs to extend from
 *
 * @tparam PlayFsm the Play FSM to base this class off of (e.g. HaltPlay needs HaltFSM)
 * @tparam PlaySubFsms the sub FSMs this play uses (none use this yet)
 */
template <class PlayFsm, class... PlaySubFsms>
class PlayBase : public Play
{
   public:
    /**
     * The constructor for a PlayBase
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit PlayBase(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr,
                      bool requires_goalie)
            : Play(ai_config_ptr, requires_goalie),
              logger(),
              fsm{PlayFsm{ai_config_ptr}, PlaySubFsms{ai_config_ptr}..., logger},
              control_params()
    {
    }

    PlayBase() = delete;

    void updateTactics(const PlayUpdate &play_update) override = 0;

   protected:
    FSMLogger logger;
    FSM<PlayFsm> fsm;
    PlayFsm::ControlParams control_params;
};
