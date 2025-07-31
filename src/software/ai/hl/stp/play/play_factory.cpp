#include "software/ai/hl/stp/play/play_factory.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

std::unique_ptr<Play> createPlay(const TbotsProto::Play& play_proto,
                                 std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
{
    return GenericFactory<std::string, Play, std::shared_ptr<TbotsProto::AiConfig>>::create(
        TbotsProto::PlayName_Name(play_proto.name()), ai_config_ptr);
}
