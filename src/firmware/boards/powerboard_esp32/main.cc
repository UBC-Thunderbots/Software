#ifdef PLATFORMIO_BUILD
#include <constants_platformio.h>   // PlatformIO sees and includes the library based on the bazel rule name ONLY
#include "echo.h"
#else
#include "shared/constants.h"
#include "echo.h"
#include <constants_platformio.h>
#endif

#include <Arduino.h>

/**
* Main code to run on the powerboard.
*/

void setup()
{
    Serial.begin(ARDUINO_BAUD_RATE);
}

void loop()
{
    Serial.print("hello!\n");
}