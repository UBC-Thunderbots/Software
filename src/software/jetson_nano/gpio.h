// Copyright (c) 2019, Alex Mous
// Licensed under the Creative Commons Attribution-ShareAlike 4.0 International
// (CC-BY-4.0)

// Header file for GPIO.cpp

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

class GPIO
{
   private:
    int _direction;       // Pin direction; either 1 for output or 0 for input
    int _level;           // Current pin level (in direction="out" mode)
    const char *pin_num;  // Holds the pin number

   public:
    GPIO(const char *pin);
    int setupPin(int create);  // Setup pin (ARGS create: whether to create or destroy pin
                               // (1 for create, 0 for destroy))
    int setDirection(int direction);  // Set pin direction (ARGS direction: either 1 for
                                      // output or 0 for input)
    int readValue(
        std::string *level);  // Read current value of the pin (ARGS *level: Address of
                              // string type variable, run command with
                              // readValue(&STRING_VAR) and the result will be stored in
                              // STRING_VAR) (only works if _direction is 0 (input))
    int writeValue(
        int level);  // Write pin (ARGS level: level to set pin, either 0 for LOW or 1 for
                     // HIGH) (only works if _direction is 1 (output))
};
