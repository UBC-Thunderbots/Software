#pragma once

#include <array>

#include "pins.h"
#include "proto/geneva_slot.nanopb.h"

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
