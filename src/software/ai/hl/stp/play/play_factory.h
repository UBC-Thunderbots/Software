#pragma once

#include "proto/play.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Creates a play given a play proto
 *
 * @param play_proto the play proto
 * @param ai_config The AI config
 *
 * @return a pointer to the play
 */
std::unique_ptr<Play> createPlay(const TbotsProto::Play& play_proto,
                                 std::shared_ptr<const AiConfig> ai_config);
