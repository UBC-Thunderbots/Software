/** @file AS5X47Spi.cpp
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

#include "software/jetson_nano/encoder/as5047spi.h"

AS5X47Spi::AS5X47Spi(uint8_t _chipSelectPin) {
	// Initialize SPI Communication
	chipSelectPin = _chipSelectPin;
	pinMode(chipSelectPin, OUTPUT);
	digitalWrite(chipSelectPin, HIGH);
	SPI.begin();
}


void AS5X47Spi::writeData(uint16_t command, uint16_t value) {
	// @todo Expose the SPI Maximum Frequency in library interface.
	SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
	// Send command
	digitalWrite(chipSelectPin, LOW);
	SPI.transfer16(command);
	digitalWrite(chipSelectPin, HIGH);
	delayMicroseconds(1);
	// Read data
	digitalWrite(chipSelectPin, LOW);
	SPI.transfer16(value);
	digitalWrite(chipSelectPin, HIGH);
	SPI.endTransaction();
	delayMicroseconds(1);

}

uint16_t AS5X47Spi::readData(uint16_t command, uint16_t nopCommand) {
	SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));

	// Send Read Command
	digitalWrite(chipSelectPin, LOW);
	SPI.transfer16(command);
	digitalWrite(chipSelectPin, HIGH);
	delayMicroseconds(1);
	// Send Nop Command while receiving data
	digitalWrite(chipSelectPin, LOW);
	uint16_t receivedData = SPI.transfer16(nopCommand);
	digitalWrite(chipSelectPin, HIGH);
	SPI.endTransaction();
	delayMicroseconds(1);
	return receivedData;
}
