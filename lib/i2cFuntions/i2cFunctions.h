#ifndef I2CFUNCTIONS_H
#define I2CFUNCTIONS_H

bool i2cPing(uint8_t);
uint8_t i2cGetDataFromSlave(uint8_t*, int);
uint8_t i2cSendDataToSlave(uint8_t*, uint8_t);
uint8_t i2cSetNextRequestSize(uint8_t);
bool i2cGetFullData();
bool i2cGetMotionData();
void i2cSendTrajPoint(uint8_t,uint8_t,uint16_t,uint8_t,int32_t, uint32_t,uint8_t);
void i2cSetPoint(int32_t);
void i2cAbort();
void i2cGo();
void i2cStop();
void i2cSetHardwareConfig(uint32_t,uint32_t,bool,bool);
void i2cSetControlCoef(float,float,float,float,uint16_t);
void i2cMotorOff();
void i2cMotorOn();
void i2cSetSlaveMode(uint8_t);
void i2cSetJogSpeed(float);
void i2cSetSlaveMotorPosition(int32_t);
void i2cSetSlaveEncoderPosition(float);


// eeprom related functions : 
void writeConfigToEeprom();
void readConfigFromEeprom();
void eepromUpdateByte(uint16_t, uint8_t);














#endif