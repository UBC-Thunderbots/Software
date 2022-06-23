#include "geneva.h"

hw_timer_t* Geneva::timer                                   = nullptr;
volatile int Geneva::dir                                    = 0;
volatile int Geneva::count_a                                = 0;
volatile int Geneva::count_b                                = 0;
volatile int Geneva::prev_count_a                           = 0;
volatile int Geneva::prev_count_b                           = 0;
Slot Geneva::slot                                           = Slot::CENTER;
void (*volatile IRAM_ATTR Geneva::rotation_done_callback)() = NULL;

Geneva::Geneva()
{
    pinMode(DIR, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(GENEVA_ENCODER_A_PIN, INPUT_PULLDOWN);
    pinMode(GENEVA_ENCODER_B_PIN, INPUT_PULLDOWN);

    attachInterrupt(digitalPinToInterrupt(GENEVA_ENCODER_A_PIN), pulseEncoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(GENEVA_ENCODER_B_PIN), pulseEncoderB, RISING);

    timer = timerBegin(GENEVA_TIMER, 80, true);

    timerAttachInterrupt(timer, &onTimer, true);
    // call onTimer every 0.1 seconds
    timerAlarmWrite(timer, 100000, true);
    timerAlarmEnable(timer);
}

void IRAM_ATTR Geneva::pulseEncoderA()
{
    count_a = count_a + dir;
}

void IRAM_ATTR Geneva::pulseEncoderB()
{
    count_b = count_b + dir;
}

void IRAM_ATTR Geneva::onTimer()
{
    // stop motor when it stalls aka in side slots
    // or if its centering it should be a fixed amount
    if (((slot == Slot::LEFT || slot == Slot::RIGHT) &&
         ((count_a != 0 && prev_count_a == count_a) ||
          (count_b != 0 && prev_count_b == count_b))) ||
        (slot == Slot::CENTER && ((dir == 1 && count_a >= CENTERING_VALUE_FROM_LEFT) ||
                                  (dir == -1 && count_a <= CENTERING_VALUE_FROM_RIGHT))))
    {
        digitalWrite(PWM, LOW);
        count_a = 0;
        count_b = 0;
        if (rotation_done_callback)
        {
            rotation_done_callback();
            rotation_done_callback = NULL;
        }
    }

    prev_count_a = count_a;
    prev_count_b = count_b;
}

float Geneva::getCurrentAngle()
{
    switch (slot)
    {
        case Slot::LEFT:
            return 30;
        case Slot::CENTER:
            return 0;
        case Slot::RIGHT:
            return -30;
    }
    return 0;
}
void Geneva::setRotationDoneCallbackOnce(void (*rotation_done_callback)())
{
    Geneva::rotation_done_callback = rotation_done_callback;
}

void Geneva::rotateLeft()
{
    dir = 1;
    digitalWrite(DIR, LOW);
    digitalWrite(PWM, HIGH);
}

void Geneva::rotateRight()
{
    dir = -1;
    digitalWrite(DIR, HIGH);
    digitalWrite(PWM, HIGH);
}

void Geneva::setAngle(float angle_deg)
{
    if (angle_deg > 0)
    {
        slot = Slot::LEFT;
        rotateLeft();
    }
    else if (angle_deg == 0)
    {
        if (angle_deg > getCurrentAngle())
        {
            slot = Slot::CENTER;
            rotateLeft();
        }
        else if (angle_deg < getCurrentAngle())
        {
            slot = Slot::CENTER;
            rotateRight();
        }
    }
    else
    {
        slot = Slot::RIGHT;
        rotateRight();
    }
}
