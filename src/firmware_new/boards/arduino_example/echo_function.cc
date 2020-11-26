//
// Created by amr on 2020-11-21.
//

#include <Arduino.h>


#ifdef PLATFORMIO_BUILD
#include "echo_function.h" // This is how PlatformIO sees and includes the library.
#else
#endif

int incomingByte = 0;

void echo() {
    // send data only when you receive data:
    if (Serial.available() > 0) {

        // read the incoming byte:
        incomingByte = Serial.read();

        // say what you got:
        Serial.print((char)incomingByte);
    }
}
