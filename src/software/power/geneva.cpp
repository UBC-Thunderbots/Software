#include "geneva.h"

hw_timer_t* Geneva::timer                  = nullptr;
volatile int Geneva::dir                   = 0;
volatile int Geneva::count_a               = 0;
volatile int Geneva::count_b               = 0;
volatile int Geneva::prev_count_a          = 0;
volatile int Geneva::prev_count_b          = 0;
TbotsProto_Geneva_Slot Geneva::current_slot = TbotsProto_Geneva_Slot_CENTRE;
TbotsProto_Geneva_Slot Geneva::homingSlot = TbotsProto_Geneva_Slot_CENTRE;
void (*volatile IRAM_ATTR Geneva::rotation_done_callback)() = NULL;
std::array<int, _TbotsProto_Geneva_Slot_ARRAYSIZE> Geneva::VALUE_FROM_LEFT;
std::array<int, _TbotsProto_Geneva_Slot_ARRAYSIZE> Geneva::VALUE_FROM_RIGHT;

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

    VALUE_FROM_LEFT[TbotsProto_Geneva_Slot_CENTRE] = CENTERING_VALUE_FROM_LEFT;
    VALUE_FROM_LEFT[TbotsProto_Geneva_Slot_RIGHT]  = RIGHTING_VALUE_FROM_LEFT;
    VALUE_FROM_RIGHT[TbotsProto_Geneva_Slot_CENTRE] = CENTERING_VALUE_FROM_RIGHT;
    VALUE_FROM_RIGHT[TbotsProto_Geneva_Slot_LEFT]  = LEFTING_VALUE_FROM_RIGHT;

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

    bool geneva_motor_stalled = ((count_a != 0 && prev_count_a == count_a) || (count_b != 0 && prev_count_b == count_b));

    bool homed_to_slot = false;
    if (homingSlot == TbotsProto_Geneva_Slot_LEFT) {
        homed_to_slot = count_a >= VALUE_FROM_LEFT[current_slot];
    }
    if (homingSlot == TbotsProto_Geneva_Slot_RIGHT) {
        homed_to_slot = count_b <= VALUE_FROM_RIGHT[current_slot];
    }

    if (homed_to_slot || geneva_motor_stalled) {
        digitalWrite(PWM, LOW);
        if (rotation_done_callback)
        {
            rotation_done_callback();
            rotation_done_callback = NULL;
        }
    }

    if (geneva_motor_stalled) {
        count_a = 0;
        count_b = 0;
        if (homingSlot == TbotsProto_Geneva_Slot_CENTRE) {
            homingSlot = current_slot;
        }
    }

    prev_count_a = count_a;
    prev_count_b = count_b;
}

TbotsProto_Geneva_Slot Geneva::getCurrentSlot()
{
    return current_slot;
}
void Geneva::setRotationDoneCallbackOnce(void (*rotation_done_callback)())
{
    Geneva::rotation_done_callback = rotation_done_callback;
}

int32_t Geneva::getEncoderValueA() {
    return count_a;
}

int32_t Geneva::getEncoderValueB() {
    return count_b;
}

void Geneva::rotateLeft()
{
    dir = LEFT_DIR;
    digitalWrite(DIR, LOW);
    digitalWrite(PWM, HIGH);
}

void Geneva::rotateRight()
{
    dir = RIGHT_DIR;
    digitalWrite(DIR, HIGH);
    digitalWrite(PWM, HIGH);
}

void Geneva::setSlot(TbotsProto_Geneva_Slot slot)
{
    switch (slot)
    {
        case TbotsProto_Geneva_Slot_LEFT:
            if (getCurrentSlot() != TbotsProto_Geneva_Slot_LEFT)
            {
                rotateLeft();
            }
            break;
        case TbotsProto_Geneva_Slot_CENTRE:
            switch (getCurrentSlot())
            {
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
            if (getCurrentSlot() != TbotsProto_Geneva_Slot_RIGHT)
            {
                rotateRight();
            }
            break;
        case TbotsProto_Geneva_Slot_CENTRE_LEFT:
        case TbotsProto_Geneva_Slot_CENTRE_RIGHT:
        default:
            break;
    }
    current_slot = slot;
}
