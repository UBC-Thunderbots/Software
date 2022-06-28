#include "geneva.h"

hw_timer_t* Geneva::timer                                   = nullptr;
volatile int Geneva::dir                                    = 0;
volatile int Geneva::count_a                                = 0;
volatile int Geneva::count_b                                = 0;
volatile int Geneva::prev_count_a                           = 0;
volatile int Geneva::prev_count_b                           = 0;
TbotsProto_Geneva_Slot Geneva::currentSlot                         = TbotsProto_Geneva_Slot_CENTRE;
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
    if (((currentSlot == TbotsProto_Geneva_Slot_LEFT || currentSlot == TbotsProto_Geneva_Slot_RIGHT) &&
         ((count_a != 0 && prev_count_a == count_a) ||
          (count_b != 0 && prev_count_b == count_b))) ||
        (currentSlot == TbotsProto_Geneva_Slot_CENTRE && ((dir == 1 && count_a >= CENTERING_VALUE_FROM_LEFT) ||
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

TbotsProto_Geneva_Slot Geneva::getCurrentSlot()
{
    return currentSlot;
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

void Geneva::setSlot(TbotsProto_Geneva_Slot slot)
{
    switch (slot) {
        case TbotsProto_Geneva_Slot_LEFT:
            if (getCurrentSlot() != TbotsProto_Geneva_Slot_LEFT) {
                rotateLeft();
            }
            break;
        case TbotsProto_Geneva_Slot_CENTRE:
            switch (getCurrentSlot()) {
                case TbotsProto_Geneva_Slot_LEFT:
                    rotateRight();
                    break;
                case TbotsProto_Geneva_Slot_CENTRE:
                    break;
                case TbotsProto_Geneva_Slot_RIGHT:
                    rotateLeft();
                    break;
                case TbotsProto_Geneva_Slot_CENTRE_LEFT:
                case TbotsProto_Geneva_Slot_CENTRE_RIGHT:
                default:
                    break;
            }
            break;
        case TbotsProto_Geneva_Slot_RIGHT:
            if (getCurrentSlot() != TbotsProto_Geneva_Slot_RIGHT) {
                rotateRight();
            }
            break;
        case TbotsProto_Geneva_Slot_CENTRE_LEFT:
        case TbotsProto_Geneva_Slot_CENTRE_RIGHT:
        default:
            break;
    }
    currentSlot = slot;
}
