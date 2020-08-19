#include "software/proto/message_translation/defending_side.h"

std::unique_ptr<DefendingSideProto> createDefendingSideProto(bool defending_positive_side) {
    auto msg                    = std::make_unique<DefendingSideProto>();
    msg->set_defending_positive_side(defending_positive_side);

    return msg;
}
