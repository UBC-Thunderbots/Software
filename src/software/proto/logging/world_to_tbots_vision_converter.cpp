#include "software/proto/logging/world_to_tbots_vision_converter.h"

#include "software/proto/message_translation/tbots_protobuf.h"

void WorldToTbotsVisionConverter::onValueReceived(World world)
{
    sendValueToObservers(*createVision(world));
}
