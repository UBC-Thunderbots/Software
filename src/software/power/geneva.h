#pragma once

#include <array>

#include "pins.h"
#include "proto/tbots_nanopb_proto_nanopb_gen/proto/geneva_slot.pb.h"

/**
 * Represents the geneva motor on the power board
 */
class Geneva
{
   public:
    /**
     * Creates a Geneva setting up relevant pins and attaching interrupts
     */
    Geneva();
    /**
     * Returns the current slot of the geneva motor
     *
     * @returns the current slot of the geneva motor
     */
    TbotsProto_Geneva_Slot getCurrentSlot();

    static TbotsProto_Geneva_Slot current_slot;
};
