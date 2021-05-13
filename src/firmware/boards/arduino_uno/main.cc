
#ifdef PLATFORMIO_BUILD
#include <constants_platformio.h>  // PlatformIO sees and includes the library based on the bazel rule name ONLY
#else
#include "shared/constants.h" //actual path to the file
#endif

#include <Arduino.h>

/**
 * main code to be run on Arduino
 */

int input_pin = 7;

void setup() {
    Serial.begin(ARDUINO_BAUD_RATE);    // opens serial port, sets data rate to 9600 bps
    pinMode(input_pin, INPUT_PULLUP);
}

void loop() {

    //todo (#1953) write firmware based on estop board

    //PLAY when connected to gnd, STOP when yanked out
    unsigned char estopVal = digitalRead(input_pin)==0 ? ESTOP_PLAY_MSG : ESTOP_STOP_MSG;

    Serial.print(estopVal);

}
