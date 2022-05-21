#ifndef LTC4151_H
#define LTC4151_H

#include <Arduino.h>

/*
 * LTC4151 Library
 * Kerry D. Wong
 * http://www.kerrywong.com
 * 4/2014
 */

class LTC4151 {
public:
	static const byte L = 0; //low
	static const byte H = 1; //high
	static const byte F = 2; //float

	static const byte REG_SENSE_H = 0x0;
	static const byte REG_SENSE_L = 0x1;
	static const byte REG_VIN_H = 0x2;
	static const byte REG_VIN_L = 0x3;
	static const byte REG_ADIN_H = 0x4;
	static const byte REG_ADIN_L = 0x5;
	static const byte REG_CTRL = 0x6;

	static const byte CTRL_BIT_SNAPSHOT_ENABLE = 0x7;
	
	//G6 G5
	//0  0  sense (default)
	//0  1  Vin
	//1  0  ADIN
	static const byte CTRL_BIT_ADC_CHN_SNAPSHOT_MODE = 0x5; 
	
	//1  enable test mode
	//0  disable test mode (default)
	static const byte CTRL_BIT_TEST_MODE_ENABLE = 0x4;

	//1  enable page RW (default)
	//0  disable page RW
	static const byte CTRL_BIT_PAGE_RW_ENABLE = 0x3;

	//1  enable (default)
	//0  disable
	static const byte CTRL_BIT_STUCK_BUS_TIMER_ENABLE = 0x2;

	static const byte SNAPSHOT_CHANNEL_SENSE = 0x0;
	static const byte SNAPSHOT_CHANNEL_VIN = 0x1;
	static const byte SNAPSHOT_CHANNEL_ADIN = 0x2;

	void init(byte A0, byte A1);
	
	//get load current (result in Ampere)
	//r is the sense resistor value in Ohm
	double getLoadCurrent(double r);
	double getInputVoltage();
	double getADCInVoltage();

	double getSnapshotLoadCurrent(double r);
	double getSnapshotInputVoltage();
	double getSnapshotADCInVoltage();

	byte getControlRegister();
	void setControlRegister(byte ctrlReg);
	
private:
	byte I2C_ADDRESS;
	long readADC(byte reg, byte numOfBytes);
	long readADCSnapshot(byte reg);
	void disableSnapshotMode(byte ctrlReg);
};
#endif
