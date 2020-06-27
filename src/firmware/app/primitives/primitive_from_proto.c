#include "firmware/app/primitives/primitive_from_proto.h"
#include "firmware/app/primitives/move_primitive.h"
#include "firmware/app/primitives/stop_primitive.h"

bool create_firmware_primitive_from_proto(PrimitiveMsg proto, primitive_t* created_primitive)
{
    switch (proto.which_primitive)
    {
        case PrimitiveMsg_move_primitive_tag:
            *created_primitive = MOVE_PRIMITIVE;
            created_primitive->start(
                proto.primitive.move_primitive.destination.x,
                proto.primitive.move_primitive.destination.y,
                proto.primitive.move_primitive.final_orientation_rad,
                proto.primitive.move_primitive.final_speed_m_per_s,
                )
            return true;
        case PrimitiveMsg_stop_primitive_tag:
            break;
    }
    return false;
}
