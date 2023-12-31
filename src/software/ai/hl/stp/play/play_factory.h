#pragma once

#include "proto/play.pb.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Creates a play given a play proto
 *
 * @param play_proto the play proto
 * @param ai_config  The AI config
 * @param strategy   to get and store shared calculations 
 *
 * @return a pointer to the play
 */
std::unique_ptr<Play> createPlay(const TbotsProto::Play& play_proto,
                                 TbotsProto::AiConfig ai_config,
                                 std::shared_ptr<Strategy> strategy = std::make_shared<Strategy>());
