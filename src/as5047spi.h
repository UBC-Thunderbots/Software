/** @file AS5X47Spi.h
 *
 * @brief A library for Arduino boards that reads angles from AS5047 and AS5147 sensors.
 * 		  Also support configuration of the sensor parameters.
 *
 * @par
 * COPYRIGHT NOTICE: MIT License
 *
 * 	Copyright (c) 2020 Adrien Legrand <contact@adrien-legrand.com>
 *
 * 	Permission is hereby granted, free of charge, to any person obtaining a copy
 * 	of this software and associated documentation files (the "Software"), to deal
 * 	in the Software without restriction, including without limitation the rights
 * 	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * 	copies of the Software, and to permit persons to whom the Software is
 * 	furnished to do so, subject to the following conditions:
 *
 * 	The above copyright notice and this permission notice shall be included in all
 * 	copies or substantial portions of the Software.
 *
 * 	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * 	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * 	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * 	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * 	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * 	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * 	SOFTWARE.
 *
*/

#ifndef AS5X47COMMUNICATION_h
#define AS5X47COMMUNICATION_h

//#include "Arduino.h"
#include <SPI.h>

/**
 * This wrapper is called from the main code to send/receive data
 * to/from the SPI port. This way other users can easily adapt this library
 * to other platforms just modifying this wrapper.
 *
 */
class AS5X47Spi {
  public:

    AS5X47Spi(uint8_t chipSelectPin);
    void writeData(uint16_t command, uint16_t value);
    uint16_t readData(uint16_t command, uint16_t nopCommand);

  private:
    uint8_t chipSelectPin;
};


#endif // #AS5X47COMMUNICATION_h
