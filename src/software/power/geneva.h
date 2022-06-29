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

    /**
     * Sets the angle of the geneva motor.
     * Also attaches a pulse_timer interrupt to asynchronously wait a fixed amount of time
     * (proportional to how much the geneva motor needs to rotate) before running the
     * given isr
     *
     * @param slot slot to set the geneva motor to
     */
    void setSlot(TbotsProto_Geneva_Slot slot);

    void setRotationDoneCallbackOnce(void (*volatile rotation_done_callback)());

    int32_t getEncoderValueA();
    int32_t getEncoderValueB();

   private:
    static void pulseEncoderA();
    static void pulseEncoderB();
    static void onTimer();
    void rotateLeft();
    void rotateRight();
    static volatile int dir;
    static volatile int count_a;
    static volatile int count_b;
    static volatile int prev_count_a;
    static volatile int prev_count_b;
    static void (*volatile rotation_done_callback)();
    static hw_timer_t* timer;
    static TbotsProto_Geneva_Slot homingSlot;
    static TbotsProto_Geneva_Slot current_slot;

    // TODO: Calibration for centering geneva
    static constexpr int CENTERING_VALUE_FROM_LEFT  = 170;
    static constexpr int CENTERING_VALUE_FROM_RIGHT = -130;
    static constexpr int RIGHTING_VALUE_FROM_LEFT = 0;
    static constexpr int LEFTING_VALUE_FROM_RIGHT = 0;
    static std::array<int, _TbotsProto_Geneva_Slot_ARRAYSIZE> VALUE_FROM_LEFT;
    static std::array<int, _TbotsProto_Geneva_Slot_ARRAYSIZE> VALUE_FROM_RIGHT;
};
