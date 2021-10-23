#include "proto/message_translation/defending_side.h"

std::unique_ptr<DefendingSideProto> createDefendingSide(const FieldSide& defending_side)
{
    auto msg = std::make_unique<DefendingSideProto>();
    switch (defending_side)
    {
        case FieldSide::NEG_X:
            msg->set_defending_side(
                DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_NEG_X);
            break;
        case FieldSide::POS_X:
            msg->set_defending_side(
                DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X);
            break;
    }

    return msg;
}
