#include "software/ai/hl/stp/play/play_factory.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

std::unique_ptr<Play> createPlay(const TbotsProto::Play& play_proto,
                                 TbotsProto::AiConfig ai_config)
{
    std::unique_ptr<Play> play = GenericFactory<std::string, Play, TbotsProto::AiConfig>::create(
        TbotsProto::PlayName_Name(play_proto.name()));
    play.reset(ai_config);
}
