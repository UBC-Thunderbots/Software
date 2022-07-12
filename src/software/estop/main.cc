
#ifdef PLATFORMIO_BUILD
#include <constants_platformio.h>  // PlatformIO sees and includes the library based on the bazel rule name ONLY
#else
#include <constants_platformio.h>

#include "shared/constants.h"  //actual path to the file
#endif

#include <Arduino.h>

int ESTOP_BUTTON_PIN = 7;

void setup()
{
    Serial.begin(ARDUINO_BAUD_RATE);  // opens serial port, sets data rate
    pinMode(ESTOP_BUTTON_PIN, INPUT_PULLUP);
}

void loop()
{
    // When the estop is disabled (button released)
    //    - circuit is closed
    //    - the input is pulled down, we send ESTOP_PLAY_MSG
    // When the estop is enabled (button pushed down)
    //    - circuit is broken
    //    - the input is pulled up, we send ESTOP_STOP_MSG
    unsigned char estop_val =
        digitalRead(ESTOP_BUTTON_PIN) == 1 ? ESTOP_STOP_MSG : ESTOP_PLAY_MSG;

    Serial.write(estop_val);
}
