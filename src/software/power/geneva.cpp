#include "geneva.h"

TbotsProto_Geneva_Slot Geneva::current_slot = TbotsProto_Geneva_Slot_CENTRE;

Geneva::Geneva()
{
}

TbotsProto_Geneva_Slot Geneva::getCurrentSlot()
{
    return current_slot;
}
