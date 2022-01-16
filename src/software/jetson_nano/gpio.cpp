// Copyright (c) 2019, Alex Mous
// Licensed under the Creative Commons Attribution-ShareAlike 4.0 International
// (CC-BY-4.0)

// C++ library to control the GPIO pins on the Raspberry Pi.
// Overview of functions:
//	setupPin	--	creates/destroys pin (1 to create and init, 0 to destroy)
//	setDirection	--	set pin direction (input/output)
//	writeValue	--	set the output voltage of the GPIO pin (1 for high, 0 for low)
//	readValue	--	read the current value of the GPIO pin (sets input pointer *output to 1
//for high or 0 for low)

#include "gpio.h"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

GPIO::GPIO(const char *pin)
{
    // Run class setup
    pin_num    = pin;
    _direction = -1;  // Direction not set
    _level     = -1;  // Current pin level not set
}

int GPIO::setupPin(int create)
{  // Set create to true for creation or false to destroy pin
    // Export pin (BCM number pin_num) for use as IO, NOTE: pin_num is char type
    const char *path;

    switch (create)
    {  // Set path
        case 1:
            path = "/sys/class/gpio/export";  // export path
            break;
        case 0:
            path = "/sys/class/gpio/unexport";  // unexport path
            break;
        default:
            std::cout
                << "ERR (GPIO.h:setupPin): Incorrect value for create variable (must be either 0 or 1)\n";
            break;
    }

    std::ofstream file(path);  // Open file
    if (file.is_open())
    {                     // Check that file opened
        file << pin_num;  // Write pin number to file
        file.close();     // Close file
    }
    else
    {
        std::cout
            << "ERR (GPIO.h:setupPin): Failed to export pin (could not open 'export' file), TRY running code as root and checking /sys/class/gpio/export exists\n";
        return 1;
    }

    char pin_path[30];  // Path for pin
    sprintf(pin_path, "/sys/class/gpio/gpio%s", pin_num);
    int status = access(pin_path, F_OK);  // Access directory on system to check exists
    switch (create)
    {
        case 1:
            if (status < 0)
            {  // Check for errors
                std::cout
                    << "ERR (GPIO.h:setupPin): Failed to export GPIO pin (could not write to 'export' file), TRY running code as root and checking that /sys/class/gpio/export exists\n";
                return 1;
            }
            break;
        case 0:
            if (status >= 0)
            {  // Check for errors (ensure that file does not exist)
                std::cout
                    << "ERR (GPIO.h:setupPin): Failed to unexport GPIO pin (could not write to 'unexport' file), TRY running code as root and checking that /sys/class/gpio/export exists\n";
                return 1;
            }
            break;
        default:
            std::cout
                << "ERR (GPIO.h:setupPin): Incorrect value for create variable (must be either 0 or 1)\n";
            break;
    }

    return 0;
}

int GPIO::setDirection(int direction)
{
    // Run setup for input/output (variable direction)
    char pin_path[35];                                               // Path for pin
    sprintf(pin_path, "/sys/class/gpio/gpio%s/direction", pin_num);  // Format path
    std::ofstream file(pin_path);                                    // Open file
    if (file.is_open())
    {  // Check that file opened
        if (direction == 1)
        {                    // Check to make sure correct formatting
            file << "out";   // Write
            _direction = 1;  // Set _direction variable to prevent write on read pin
        }
        else if (direction == 0)
        {
            file << 0;
            _direction = 0;  // Set _direction variable to prevent write on read pin
        }
        else
        {
            std::cout
                << "ERR (GPIO.h:setDirection): Incorrect value for direction variable (must be either 1 for output or 0 of input)\n";
            return 1;
        }
        file.close();  // Close
    }
    else
    {
        std::cout
            << "ERR (GPIO.h:setDirection): Failed to set direction of pin (could not open 'direction' file), TRY running code as root and checking that /sys/class/gpio/gpioN/direction exists\n";
        return 1;
    }

    return 0;
}

int GPIO::writeValue(int level)
{
    // Set level of pin
    if (_direction == 1)
    {                       // Check that the direction of the pin is set as an output
        char pin_path[35];  // Path for pin
        sprintf(pin_path, "/sys/class/gpio/gpio%s/value", pin_num);  // Format path
        std::ofstream file(pin_path);                                // Open file
        if (file.is_open())
        {  // Check that file opened
            if (level == 1)
            {                 // Check to make sure correct formatting
                file << "1";  // Write
            }
            else if (level == 0)
            {
                file << "0";
            }
            else
            {
                std::cout
                    << "ERR (GPIO.h:writeValue): Incorrect value for direction variable (must be either 'out' or 'in')\n";
                return 1;
            }
            file.close();  // Close
        }
        else
        {
            std::cout
                << "ERR (GPIO.h:writeValue): Failed to set value of pin (could not open 'value' file), TRY running code as root and checking that /sys/class/gpio/gpioN/value exists\n";
            return 1;
        }
    }
    else
    {
        std::cout
            << "ERR (GPIO.h:writeValue): Cannot write pin when direction is set as input (direction must be 1 to write level)\n";
    }
    usleep(10000);
    return 0;
}

int GPIO::readValue(std::string *level)
{
    if (_direction == 0)
    {                       // Check that the direction of the pin is set as an input
        char pin_path[35];  // Path for pin
        sprintf(pin_path, "/sys/class/gpio/gpio%s/value", pin_num);  // Format path
        std::ifstream ifile(pin_path);                               // Open file
        if (ifile.is_open())
        {  // Check that file opened
            std::getline(ifile, *level);
        }
        else
        {
            std::cout
                << "ERR (GPIO.h:readValue): Failed to read pin value (could not open 'value' file), TRY running code as root and checking that /sys/class/gpio/gpioN/value exists\n";
            return 1;
        }
    }
    else
    {
        std::cout
            << "ERR (GPIO.h:readValue): Cannot read pin when direction is set as output (direction must be 0 to read level)\n";
    }
    return 0;
}
