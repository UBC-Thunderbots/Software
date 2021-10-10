//
// Created by liamb on 2021-10-09.
//
#ifdef PLATFORMIO_BUILD
#include "echo_library.h"
#else
#include "echo_library.h"
#endif

int incomingByte = 0;

void echo() {
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        Serial.print((char)incomingByte);
        Serial.println("h");
    }
}

