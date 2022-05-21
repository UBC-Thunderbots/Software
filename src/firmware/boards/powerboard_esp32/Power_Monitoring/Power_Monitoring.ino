/*
 * LTC4151 Library Example
 * Kerry D. Wong
 * http://www.kerrywong.com
 * 4/2014
 */

#include <Wire.h>
#include <LTC4151.h>
LTC4151 sensor;

void LTC4151::init(byte A0, byte A1)
{
  if (A0 == L && A1 == H) I2C_ADDRESS = B1100111;
  else if (A0 == H && A1 == F) I2C_ADDRESS = B1101000;
  else if (A0 == H && A1 == H) I2C_ADDRESS = B1101001;
  else if (A0 == F && A1 == F) I2C_ADDRESS = B1101010;
  else if (A0 == L && A1 == F) I2C_ADDRESS = B1101011;
  else if (A0 == H && A1 == L) I2C_ADDRESS = B1101100;
  else if (A0 == F && A1 == H) I2C_ADDRESS = B1101101;
  else if (A0 == F && A1 == L) I2C_ADDRESS = B1101110;
  else if (A0 == L && A1 == L) I2C_ADDRESS = B1101111;
}

long LTC4151::readADC(byte reg, byte numOfBytes)
{
  unsigned int h, l;
  long result;

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(I2C_ADDRESS, numOfBytes);

  if (numOfBytes == 1) 
  {
    result = Wire.read();
  } else if (numOfBytes == 2) 
  {
    h = Wire.read();
    l = Wire.read();
    result = h << 4 | l >> 4;
  }

  return result;
}

long LTC4151::readADCSnapshot(byte reg)
{
  unsigned int h, l;
  long result;
  bool ready = false;
  
  while (!ready) 
  {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDRESS, (byte) 2);
    h = Wire.read();
    l = Wire.read();
    
    ready = ((l & 0x8) == 0);
  }

  result = h << 4 | l >> 4;

  return result;
}

void LTC4151::setControlRegister(byte ctrlReg)
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(REG_CTRL);
  Wire.write(ctrlReg);
  Wire.endTransmission();
}

byte LTC4151::getControlRegister()
{
  return readADC(REG_CTRL, 1);
}

double LTC4151::getLoadCurrent(double r)
{
  return readADC(REG_SENSE_H, 2) * 81.92 / 4096.0 / r;
}

double LTC4151::getInputVoltage()
{
  return readADC(REG_VIN_H, 2) * 102.4 / 4096.0;
}

double LTC4151::getADCInVoltage()
{
  return readADC(REG_ADIN_H, 2) * 2.048 / 4096.0;
}

double LTC4151::getSnapshotLoadCurrent(double r) 
{
  byte ctrlReg = getControlRegister();
  disableSnapshotMode(ctrlReg); 
  

  ctrlReg |= SNAPSHOT_CHANNEL_SENSE << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
  setControlRegister(ctrlReg);

  return readADCSnapshot(REG_SENSE_H) * 81.92 / 4096.0 / r;
}

double LTC4151::getSnapshotInputVoltage()
{
  byte ctrlReg = getControlRegister();
  disableSnapshotMode(ctrlReg); 

  ctrlReg |= SNAPSHOT_CHANNEL_VIN << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
  setControlRegister(ctrlReg);

  return readADCSnapshot(REG_VIN_H) * 102.4 / 4096.0;
}

double LTC4151::getSnapshotADCInVoltage()
{
  byte ctrlReg = getControlRegister();
  disableSnapshotMode(ctrlReg); 

  ctrlReg |= SNAPSHOT_CHANNEL_ADIN << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
  setControlRegister(ctrlReg);

  return readADCSnapshot(REG_ADIN_H) * 2.048 / 4096.0;
}

void LTC4151::disableSnapshotMode(byte ctrlReg)
{
  if (ctrlReg & 1 << CTRL_BIT_SNAPSHOT_ENABLE > 0) 
  {
    //reset snapshot bit
    ctrlReg = ctrlReg & ~(1 << CTRL_BIT_SNAPSHOT_ENABLE);

    //reset adc channel to SENSE
    ctrlReg = ctrlReg & ~(1 << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE);
    ctrlReg = ctrlReg & ~(1 << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE + 1);

    setControlRegister(ctrlReg);    
  }
}

void setup()
{   
    Serial.begin(9600);
    Wire.begin(13,14);
    
    sensor.init(LTC4151::L, LTC4151::L);
}

void loop()
{
    Serial.println("Load Current");
    Serial.println(sensor.getLoadCurrent(0.002));
    Serial.println(sensor.getSnapshotLoadCurrent(0.002)); 
    Serial.print("\n");

    Serial.println("Input Voltage");
    Serial.println(sensor.getInputVoltage());    
    Serial.println(sensor.getSnapshotInputVoltage());  
    Serial.print("\n");

    Serial.println("ADCIn Input Voltage");
    Serial.println(sensor.getADCInVoltage());
    Serial.println(sensor.getSnapshotADCInVoltage());  
    Serial.print("\n");
}
