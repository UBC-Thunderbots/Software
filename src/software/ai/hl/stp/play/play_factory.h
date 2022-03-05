#pragma once

#include "proto/play.pb.h"
#include "software/ai/hl/stp/play/play.h"

std::unique_ptr<Play> createPlay(const TbotsProto::Play& play_proto,
                                 std::shared_ptr<const AiConfig> ai_config);
