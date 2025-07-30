#include "software/ai/hl/stp/play/play_base.h"

template<class PlayFsm, class... PlaySubFsms>
PlayBase<PlayFsm, PlaySubFsms...>::PlayBase(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr, bool requires_goalie)
: Play(ai_config_ptr, requires_goalie),
fsm{PlayFsm{ai_config_ptr}, PlaySubFsms{ai_config_ptr}...},
control_params(){}

