#include "software/jetson_nano/encoder/encoder.h"

Encoder::Encoder(uint8_t chipSelectPin) : 
	chip_select_pin(chipSelectPin),
	spi(chipSelectPin)
{
}

ReadDataFrame Encoder::readRegister(uint16_t registerAddress) {
	CommandFrame command;
	command.values.rw = READ;
	command.values.commandFrame = static_cast<uint16_t>(registerAddress & 0x3FFF);
	command.values.parc = isEven(command.raw);

	CommandFrame nopCommand;
	nopCommand.values.rw = READ;
	nopCommand.values.commandFrame = NOP_REG;
	nopCommand.values.parc = isEven(nopCommand.raw);

	ReadDataFrame receivedFrame;
	receivedFrame.raw = spi.readData(command.raw, nopCommand.raw);
	return receivedFrame;
}



void Encoder::writeRegister(uint16_t registerAddress, uint16_t registerValue) {
	CommandFrame command;
	command.values.rw = WRITE;
	command.values.commandFrame = static_cast<uint16_t>(registerAddress & 0x3FFF);
	command.values.parc = isEven(command.raw);

	WriteDataFrame contentFrame;
	contentFrame.values.data = static_cast<uint16_t>(registerValue & 0x3FFF);
	contentFrame.values.low = 0;
	contentFrame.values.pard = isEven(contentFrame.raw);
	spi.writeData(command.raw, contentFrame.raw);
}

float Encoder::readAngle() {
	ReadDataFrame readDataFrame = readRegister(ANGLE_REG);
	EncoderAngle angle;
	angle.raw = readDataFrame.values.data;
	return static_cast<float>(angle.values.cordicang/16384.*360.);
}

void Encoder::writeSettings1(Settings1 values) {
	writeRegister(SETTINGS1_REG, values.raw);
}
void Encoder::writeSettings2(Settings2 values){
	writeRegister(SETTINGS2_REG, values.raw);
}
void Encoder::writeZeroPosition(Zposm zposm, Zposl zposl){
	writeRegister(ZPOSM_REG, zposm.raw);
	writeRegister(ZPOSL_REG, zposl.raw);
}

//void Encoder::printDebugString() {
//	ReadDataFrame readDataFrame;
//	readDataFrame = readRegister(ERRFL_REG);
//	Errfl errfl;
//	errfl.raw = readDataFrame.values.data;
//	Serial.println("======== Encoder Debug ========");
//	Serial.println("------- ERRFL Register :");
//	Serial.print("   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("   FRERR: ");
//	Serial.println(errfl.values.frerr);
//	Serial.print("   INVCOMM: ");
//	Serial.println(errfl.values.invcomm);
//	Serial.print("   PARERR: ");
//	Serial.println(errfl.values.parerr);
//	Serial.println("");
//
//	readDataFrame = readRegister(PROG_REG);
//	Prog prog;
//	prog.raw = readDataFrame.values.data;
//	Serial.println("------- PROG Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   PROGEN: ");
//	Serial.println(prog.values.progen);
//	Serial.print("|   OTPREF: ");
//	Serial.println(prog.values.otpref);
//	Serial.print("|   PROGOTP: ");
//	Serial.println(prog.values.progotp);
//	Serial.print("|   PROVER: ");
//	Serial.println(prog.values.progver);
//	Serial.println("|");
//
//	readDataFrame = readRegister(DIAGAGC_REG);
//	Diaagc diaagc;
//	diaagc.raw = readDataFrame.values.data;
//	Serial.println("|------- DIAAGC Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   AGC: ");
//	Serial.println(diaagc.values.agc);
//	Serial.print("|   LF: ");
//	Serial.println(diaagc.values.lf);
//	Serial.print("|   COF: ");
//	Serial.println(diaagc.values.cof);
//	Serial.print("|   MAGH: ");
//	Serial.println(diaagc.values.magh);
//	Serial.print("|   MAGL: ");
//	Serial.println(diaagc.values.magl);
//	Serial.println("|");
//
//	readDataFrame = readRegister(MAG_REG);
//	Mag mag;
//	mag.raw = readDataFrame.values.data;
//	Serial.println("|------- MAG Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   CMAG: ");
//	Serial.println(mag.values.cmag);
//	Serial.println("|");
//
//	readDataFrame = readRegister(ANGLE_REG);
//	Angle angle;
//	angle.raw = readDataFrame.values.data;
//	Serial.println("|------- ANGLE Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   CORDICANG: ");
//	Serial.println(angle.values.cordicang);
//	Serial.println("|");
//
//	readDataFrame = readRegister(ANGLECOM_REG);
//	Anglecom anglecom;
//	anglecom.raw = readDataFrame.values.data;
//	Serial.println("|------- ANGLECOM Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   DAECANG: ");
//	Serial.println(anglecom.values.daecang);
//	Serial.println("|");
//
//	readDataFrame = readRegister(ZPOSM_REG);
//	Zposm zposm;
//	zposm.raw = readDataFrame.values.data;
//	Serial.println("|------- ZPOSM Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   ZPOSM: ");
//	Serial.println(zposm.values.zposm);
//	Serial.println("|");
//
//	readDataFrame = readRegister(ZPOSL_REG);
//	Zposl zposl;
//	zposl.raw = readDataFrame.values.data;
//	Serial.println("|------- ZPOSL Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   ZPOSL: ");
//	Serial.println(zposl.values.zposl);
//	Serial.print("|   COMP_L_ERROR_EN: ");
//	Serial.println(zposl.values.compLerrorEn);
//	Serial.print("|   COMP_H_ERROR_EN: ");
//	Serial.println(zposl.values.compHerrorEn);
//	Serial.println("|");
//
//	readDataFrame = readRegister(SETTINGS1_REG);
//	Settings1 settings1;
//	settings1.raw = readDataFrame.values.data;
//	Serial.println("|------- SETTINGS1 Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   NOISESET: ");
//	Serial.println(settings1.values.noiseset);
//	Serial.print("|   DIR: ");
//	Serial.println(settings1.values.dir);
//	Serial.print("|   UVW_ABI: ");
//	Serial.println(settings1.values.uvw_abi);
//	Serial.print("|   DAECDIS: ");
//	Serial.println(settings1.values.daecdis);
//	Serial.print("|   ABIBIN: ");
//	Serial.println(settings1.values.abibin);
//	Serial.print("|   DATASELECT: ");
//	Serial.println(settings1.values.dataselect);
//	Serial.print("|   PWMON: ");
//	Serial.println(settings1.values.pwmon);
//	Serial.println("|");
//
//	readDataFrame = readRegister(SETTINGS2_REG);
//	Settings2 settings2;
//	settings2.raw = readDataFrame.values.data;
//	Serial.println("|------- SETTINGS2 Register: ");
//	Serial.print("|   Reading Error: ");
//	Serial.println(readDataFrame.values.ef);
//	Serial.print("|   UVWPP: ");
//	Serial.println(settings2.values.uvwpp);
//	Serial.print("|   HYS: ");
//	Serial.println(settings2.values.hys);
//	Serial.print("|   ABIRES: ");
//	Serial.println(settings2.values.abires);
//
//	Serial.println("==============================");
//}


bool Encoder::isEven(uint16_t data) {
	int count=0;
	unsigned int b = 1;
	for (unsigned int i=0; i<15; i++) {
		if (data & (b << i)) {
			count++;
		}
	}
	
	if (count%2==0) {
		return false;

	} else {
		return true;
	}
}
