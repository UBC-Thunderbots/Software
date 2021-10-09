//
// Created by liamb on 2021-10-09.
//
#ifdef PLATFORMIO_BUILD
#include "echo.h"
#else
#include "echo.h"
#endif

int incomingByte = 0;

void echo() {
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        Serial.print((char)incomingByte);
        Serial.print('h');
    }
}

