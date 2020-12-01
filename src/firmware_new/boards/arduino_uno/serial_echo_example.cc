/*
 * Echoes what is sent back through the serial port.
 *
 */



#ifdef PLATFORMIO_BUILD
#include "echo_library.h"  // This is how PlatformIO sees and includes the library.
#else
#include "echo_library.h"  // actual path to file (in this example its the same)
#endif

#include <Arduino.h>

void setup() {
    Serial.begin(9600);    // opens serial port, sets data rate to 9600 bps
}

void loop() {
    echo();
}
