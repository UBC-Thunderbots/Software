#ifdef PLATFORMIO_BUILD
#include <constants_platformio.h>   // PlatformIO sees and includes the library based on the bazel rule name ONLY
#include "echo.h"
#else
#include "shared/constants.h"
#include "echo.h"
#include <constants_platformio.h>
#endif

#include <Arduino.h>

#define LED_BUILTIN 13

/**
* Main code to run on the powerboard.
*/

void setup()
{
    Serial.begin(ARDUINO_BAUD_RATE);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    // turn LED on/off with 1000ms pause
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}