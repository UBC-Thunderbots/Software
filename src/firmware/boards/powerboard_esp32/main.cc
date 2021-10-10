#ifdef PLATFORMIO_BUILD
#include <constants_platformio.h>   // PlatformIO sees and includes the library based on the bazel rule name ONLY
#include "echo_library.h"
#else
#include "echo_library.h"
#include "shared/constants.h"
#include <constants_platformio.h>
#endif

#include <Arduino.h>

#define LED_BLINK 13

/**
* Main code to run on the powerboard.
*/

void setup()
{
    Serial.begin(ESP32_BAUD_RATE);
    pinMode(LED_BLINK, OUTPUT);
}

void loop()
{
    // echo message sent over serial monitor
    echo();

    // turn LED on/off with 1000ms pause
    digitalWrite(LED_BLINK, HIGH);
    delay(1000);
    digitalWrite(LED_BLINK, LOW);
    delay(1000);
}