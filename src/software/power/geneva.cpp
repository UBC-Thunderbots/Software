//
// Created by cody on 2022-04-28.
//

#include "geneva.h"

hw_timer_t* Geneva::timer = nullptr;

Geneva::Geneva()
{
    timer = timerBegin(0, 80, true);
}

float Geneva::getCurrentAngle()
{
    // TODO: convert from EncoderValue to angle
    return 0;
}

void Geneva::performWhenDone(void (*isr)(void))
{
    timerRestart(timer);

    timerAttachInterrupt(timer, isr, true);
    // TODO: calculate delay based on start position and end position
    // TODO: take max of time to setChargeDoneCallbackOnce once known
    timerAlarmWrite(timer, 1000000 /* 1s */, false);
    timerAlarmEnable(timer);
}

void Geneva::setAngle(float angle_deg, void (*isr)(void))
{
    // TODO: Set angle through encoderValue
    // TODO: Set up values for isr (can't add args but can access static members)
    performWhenDone(isr);
}
