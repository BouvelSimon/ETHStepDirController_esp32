#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <I2C_eeprom.h>

#include <i2cFunctions.h>
#include <defines.h>
#include <types.h>
#include <trajectory.h>

extern uint32_t g_lastI2CdateTime;
extern uint8_t g_nextRequestSize;
extern i32_ui8 g_currentMotorPulses;
extern i32_ui8 g_currentSetPoint;
extern float_ui8 g_currentEncoderPosition;
extern uint8_t g_nTrajSlotsAvailableOnSlave;
extern ui16_ui8 g_curentActiveTraj;
extern bool g_isMotorOn;
extern bool g_closedLoopEnabled;
extern uint8_t g_errorCodesFromSlave;
extern float_ui8 g_Kp;
extern float_ui8 g_Ki;
extern float_ui8 g_Kd;
extern float_ui8 g_maxVelocity;
extern bool g_invertedMotor;
extern bool g_invertedEncoder;
extern i32_ui8 g_motorStepsPerRev;
extern i32_ui8 g_encoderStepsPerRev;
extern ui16_ui8 g_errorLimit;


extern uint8_t g_eepromImage[];
extern I2C_eeprom g_eeprom;

extern struct networkConfiguration g_networkConfig;
extern uint8_t g_boardNumber;
extern struct mqttConfiguration g_mqttConfig;
extern PubSubClient g_mqttClient;

bool i2cPing(uint8_t i2cAddress){
  Wire.beginTransmission(i2cAddress);
  if(Wire.endTransmission()==0)
    return true;
  return false;
}

uint8_t i2cGetDataFromSlave(uint8_t* data, int size){
  while(millis()-g_lastI2CdateTime<MIN_I2C_USAGE_MS){
  }
  g_lastI2CdateTime=millis();
  
  memset(data,0,size);
  uint8_t nBytesReceived=Wire.requestFrom(PICO_ADDRESS,size);
  for(uint8_t i=0;i<nBytesReceived;i++){
    if(Wire.available()){
      data[i]=Wire.read();
    }
  }
  return nBytesReceived;
}

uint8_t i2cSendDataToSlave(uint8_t* data, uint8_t size){
  while(millis()-g_lastI2CdateTime<MIN_I2C_USAGE_MS){
  } // This can be improved. Instead of waiting, a queue could be implemented
  g_lastI2CdateTime=millis();
  Wire.beginTransmission(PICO_ADDRESS);
  for(uint8_t i=0;i<size;i++){
    Wire.write(data[i]);
  }
  return Wire.endTransmission();
}

uint8_t i2cSetNextRequestSize(uint8_t nextRequestSize){
  uint8_t dataToSend[2];
  dataToSend[0]=MSGTYPE_SETNEXTREQUESTSIZE;
  dataToSend[1]=nextRequestSize;
  uint8_t result=i2cSendDataToSlave(dataToSend,2);
  if(result!=0){
    return 0;
  }else{
    return nextRequestSize;
  }
}

bool i2cGetFullData(){
  if(g_nextRequestSize!=MAX_RECEIVE_SIZE){
    i2cSetNextRequestSize(MAX_RECEIVE_SIZE);
  }
  uint8_t receivedData[MAX_RECEIVE_SIZE];
  if(i2cGetDataFromSlave(receivedData, MAX_RECEIVE_SIZE)!=MAX_RECEIVE_SIZE){
    return false;
  }

  g_currentMotorPulses.ui8[0]=receivedData[0];
  g_currentMotorPulses.ui8[1]=receivedData[1];
  g_currentMotorPulses.ui8[2]=receivedData[2];
  g_currentMotorPulses.ui8[3]=receivedData[3];

  g_currentSetPoint.ui8[0]=receivedData[4];
  g_currentSetPoint.ui8[1]=receivedData[5];
  g_currentSetPoint.ui8[2]=receivedData[6];
  g_currentSetPoint.ui8[3]=receivedData[7];

  g_currentEncoderPosition.ui8[0]=receivedData[8];
  g_currentEncoderPosition.ui8[1]=receivedData[9];
  g_currentEncoderPosition.ui8[2]=receivedData[10];
  g_currentEncoderPosition.ui8[3]=receivedData[11];

  g_nTrajSlotsAvailableOnSlave=receivedData[12];
  
  g_curentActiveTraj.ui8[0]=receivedData[13];
  g_curentActiveTraj.ui8[1]=receivedData[14];
  
  g_isMotorOn=receivedData[15]!=0;
  
  g_closedLoopEnabled=receivedData[16]!=0;

  g_errorCodesFromSlave=receivedData[17];
  
  g_Kp.ui8[0]=receivedData[18];
  g_Kp.ui8[1]=receivedData[19];
  g_Kp.ui8[2]=receivedData[20];
  g_Kp.ui8[3]=receivedData[21];
  
  g_Ki.ui8[0]=receivedData[22];
  g_Ki.ui8[1]=receivedData[23];
  g_Ki.ui8[2]=receivedData[24];
  g_Ki.ui8[3]=receivedData[25];
  
  g_Kd.ui8[0]=receivedData[26];
  g_Kd.ui8[1]=receivedData[27];
  g_Kd.ui8[2]=receivedData[28];
  g_Kd.ui8[3]=receivedData[29];

  g_maxVelocity.ui8[0]=receivedData[30];
  g_maxVelocity.ui8[0]=receivedData[31];
  g_maxVelocity.ui8[0]=receivedData[32];
  g_maxVelocity.ui8[0]=receivedData[33];

  g_invertedMotor=receivedData[34]!=0;
  g_invertedEncoder=receivedData[35]!=0;

  g_motorStepsPerRev.ui8[0]=receivedData[36];
  g_motorStepsPerRev.ui8[1]=receivedData[37];
  g_motorStepsPerRev.ui8[2]=receivedData[38];
  g_motorStepsPerRev.ui8[3]=receivedData[39];
  
  g_encoderStepsPerRev.ui8[0]=receivedData[40];
  g_encoderStepsPerRev.ui8[1]=receivedData[41];
  g_encoderStepsPerRev.ui8[2]=receivedData[42];
  g_encoderStepsPerRev.ui8[3]=receivedData[43];

  g_errorLimit.ui8[0]=receivedData[44];
  g_errorLimit.ui8[1]=receivedData[45];

  return true;
}

bool i2cGetMotionData(){ 
  // Updates the following : 
  // g_currentMotorPulses;
  // g_currentSetPoint;
  // g_currentEncoderPosition;
  // g_nTrajSlotsAvailableOnSlave;
  // g_curentActiveTraj;
  // g_isMotorOn;
  // g_closedLoopEnabled;
  // g_errorCodesFromSlave
  
  if(g_nextRequestSize!=18){
    i2cSetNextRequestSize(18);
  }
  uint8_t receivedData[18];
  if(i2cGetDataFromSlave(receivedData, 18)!=18){
    return false;
  }

  g_currentMotorPulses.ui8[0]=receivedData[0];
  g_currentMotorPulses.ui8[1]=receivedData[1];
  g_currentMotorPulses.ui8[2]=receivedData[2];
  g_currentMotorPulses.ui8[3]=receivedData[3];

  g_currentSetPoint.ui8[0]=receivedData[4];
  g_currentSetPoint.ui8[1]=receivedData[5];
  g_currentSetPoint.ui8[2]=receivedData[6];
  g_currentSetPoint.ui8[3]=receivedData[7];

  g_currentEncoderPosition.ui8[0]=receivedData[8];
  g_currentEncoderPosition.ui8[1]=receivedData[9];
  g_currentEncoderPosition.ui8[2]=receivedData[10];
  g_currentEncoderPosition.ui8[3]=receivedData[11];

  g_nTrajSlotsAvailableOnSlave=receivedData[12];
  
  g_curentActiveTraj.ui8[0]=receivedData[13];
  g_curentActiveTraj.ui8[1]=receivedData[14];
  
  g_isMotorOn=receivedData[15]!=0;
  
  g_closedLoopEnabled=receivedData[16]!=0;

  g_errorCodesFromSlave=receivedData[17];

  return true;
}

void i2cSetSlaveMode(uint8_t modeToSet){
  uint8_t dataToTransmit[2];
  dataToTransmit[0]=MSGTYPE_SETMODE;
  dataToTransmit[1]=modeToSet;
  i2cSendDataToSlave(dataToTransmit,2);
}

void i2cMotorOn(){
  uint8_t dataToTransmit[2];
  dataToTransmit[0]=MSGTYPE_MOTORPOWER;
  dataToTransmit[1]=1;
  i2cSendDataToSlave(dataToTransmit,2);
}
void i2cMotorOff(){
    uint8_t dataToTransmit[2];
  dataToTransmit[0]=MSGTYPE_MOTORPOWER;
  dataToTransmit[1]=0;
  i2cSendDataToSlave(dataToTransmit,2);
}

void i2cSetControlCoef(float Kp,float Ki,float Kd,float maxVelocity,uint16_t errorLimit){
  float_ui8 KpToTransmit;
  float_ui8 KiToTransmit;
  float_ui8 KdToTransmit;
  float_ui8 maxVelocityToTransmit;
  ui16_ui8 errorLimitToTransmit;

  KpToTransmit.f=Kp;
  KiToTransmit.f=Ki;
  KdToTransmit.f=Kd;
  maxVelocityToTransmit.f=maxVelocity;
  errorLimitToTransmit.ui16=errorLimit;
  
  uint8_t dataToTransmit[20];

  dataToTransmit[0]=MSGTYPE_CONTROLCOEFS;

  dataToTransmit[1]=KpToTransmit.ui8[0];
  dataToTransmit[2]=KpToTransmit.ui8[1];
  dataToTransmit[3]=KpToTransmit.ui8[2];
  dataToTransmit[4]=KpToTransmit.ui8[3];

  dataToTransmit[5]=KiToTransmit.ui8[0];
  dataToTransmit[6]=KiToTransmit.ui8[1];
  dataToTransmit[7]=KiToTransmit.ui8[2];
  dataToTransmit[8]=KiToTransmit.ui8[3];

  dataToTransmit[9]=KdToTransmit.ui8[0];
  dataToTransmit[10]=KdToTransmit.ui8[1];
  dataToTransmit[11]=KdToTransmit.ui8[2];
  dataToTransmit[12]=KdToTransmit.ui8[3];

  dataToTransmit[13]=maxVelocityToTransmit.ui8[0];
  dataToTransmit[14]=maxVelocityToTransmit.ui8[1];
  dataToTransmit[15]=maxVelocityToTransmit.ui8[2];
  dataToTransmit[16]=maxVelocityToTransmit.ui8[3];

  dataToTransmit[17]=errorLimitToTransmit.ui8[0];
  dataToTransmit[18]=errorLimitToTransmit.ui8[1];

  i2cSendDataToSlave(dataToTransmit,19);
}

void i2cSetHardwareConfig(uint32_t motorStepsPerRev,uint32_t encoderStepsPerRev,bool invertedMotor,bool invertedEncoder){

  ui32_ui8 motorStepsPerRevToTransmit;
  ui32_ui8 encoderStepsPerRevToTransmit;

  motorStepsPerRevToTransmit.ui32=motorStepsPerRev;
  encoderStepsPerRevToTransmit.ui32=encoderStepsPerRev;

  uint8_t dataToTransmit[11];

  dataToTransmit[0]=MSGTYPE_HWCONFIG;

  dataToTransmit[1]=motorStepsPerRevToTransmit.ui8[0];
  dataToTransmit[2]=motorStepsPerRevToTransmit.ui8[1];
  dataToTransmit[3]=motorStepsPerRevToTransmit.ui8[2];
  dataToTransmit[4]=motorStepsPerRevToTransmit.ui8[3];

  dataToTransmit[5]=encoderStepsPerRevToTransmit.ui8[0];
  dataToTransmit[6]=encoderStepsPerRevToTransmit.ui8[1];
  dataToTransmit[7]=encoderStepsPerRevToTransmit.ui8[2];
  dataToTransmit[8]=encoderStepsPerRevToTransmit.ui8[3];

  dataToTransmit[9]=uint8_t(invertedMotor);
  dataToTransmit[10]=uint8_t(invertedEncoder);

  i2cSendDataToSlave(dataToTransmit,11);
}

void i2cGo(){
  uint8_t dataToTransmit[2];
  dataToTransmit[0]=MSGTYPE_MOTION;
  dataToTransmit[1]=MSG_GO;
  i2cSendDataToSlave(dataToTransmit,2);
}

void i2cSetPoint(int32_t setPoint){
  i32_ui8 setPointToTransmit;
  setPointToTransmit.i32=setPoint;

  uint8_t dataToTransmit[5];
  dataToTransmit[0]=MSGTYPE_SETPOINT;
  dataToTransmit[1]=setPointToTransmit.ui8[0];
  dataToTransmit[2]=setPointToTransmit.ui8[1];
  dataToTransmit[3]=setPointToTransmit.ui8[2];
  dataToTransmit[4]=setPointToTransmit.ui8[3];
  i2cSendDataToSlave(dataToTransmit,5);
}

void i2cSendTrajPoint(uint8_t currentPointInTraj,uint8_t totalPointsInTraj,uint16_t trajId,uint8_t trajType,int32_t currentPosition, uint32_t currentTime,uint8_t relativeOrAbsolute){
  ui16_ui8 trajIdToSend;
  i32_ui8 positionToSend;
  ui32_ui8 timeToSend;

  trajIdToSend.ui16=trajId;
  positionToSend.i32=currentPosition;
  timeToSend.ui32=currentTime;

  uint8_t dataToTransmit[15];
  dataToTransmit[0]=MSGTYPE_APPENDTRAJ;
  dataToTransmit[1]=currentPointInTraj;
  dataToTransmit[2]=totalPointsInTraj;
  dataToTransmit[3]=trajIdToSend.ui8[0];
  dataToTransmit[4]=trajIdToSend.ui8[1];
  dataToTransmit[5]=trajType;
  dataToTransmit[6]=positionToSend.ui8[0];
  dataToTransmit[7]=positionToSend.ui8[1];
  dataToTransmit[8]=positionToSend.ui8[2];
  dataToTransmit[9]=positionToSend.ui8[3];
  dataToTransmit[10]=timeToSend.ui8[0];
  dataToTransmit[11]=timeToSend.ui8[1];
  dataToTransmit[12]=timeToSend.ui8[2];
  dataToTransmit[13]=timeToSend.ui8[3];
  dataToTransmit[14]=relativeOrAbsolute;

  i2cSendDataToSlave(dataToTransmit,15);
}

void i2cSetJogSpeed(float jogSpeed){
  uint8_t dataToTransmit[5];
  float_ui8 jogSpeedToTransmit;
  jogSpeedToTransmit.f=jogSpeed;

  dataToTransmit[0]=MSGTYPE_JOG;
  dataToTransmit[1]=jogSpeedToTransmit.ui8[0];
  dataToTransmit[2]=jogSpeedToTransmit.ui8[1];
  dataToTransmit[3]=jogSpeedToTransmit.ui8[2];
  dataToTransmit[4]=jogSpeedToTransmit.ui8[3];
  i2cSendDataToSlave(dataToTransmit,5);
}

void i2cSetSlaveMotorPosition(int32_t motorPosition){
  uint8_t dataToTransmit[5];
  dataToTransmit[0]=MSGTYPE_SETMOTORPOS;
  
  i32_ui8 motorPosToTransmit;
  motorPosToTransmit.i32=motorPosition;
  dataToTransmit[1]=motorPosToTransmit.ui8[0];
  dataToTransmit[2]=motorPosToTransmit.ui8[1];
  dataToTransmit[3]=motorPosToTransmit.ui8[2];
  dataToTransmit[4]=motorPosToTransmit.ui8[3];
  i2cSendDataToSlave(dataToTransmit,5);
}

void i2cSetSlaveEncoderPosition(float encPosition){
  uint8_t dataToTransmit[5];
  dataToTransmit[0]=MSGTYPE_SETENCODERPOS;
  
  float_ui8 encPosToTransmit;
  encPosToTransmit.f=encPosition;
  dataToTransmit[1]=encPosToTransmit.ui8[0];
  dataToTransmit[2]=encPosToTransmit.ui8[1];
  dataToTransmit[3]=encPosToTransmit.ui8[2];
  dataToTransmit[4]=encPosToTransmit.ui8[3];
  i2cSendDataToSlave(dataToTransmit,5);
}

void i2cStop(){
  uint8_t dataToTransmit[2];
  dataToTransmit[0]=MSGTYPE_MOTION;
  dataToTransmit[1]=MSG_STOP;
  i2cSendDataToSlave(dataToTransmit,2);
  initializeTrajQueue();
}

void i2cAbort(){
  uint8_t dataToTransmit[2];
  dataToTransmit[0]=MSGTYPE_MOTION;
  dataToTransmit[1]=MSG_ABORT;
  i2cSendDataToSlave(dataToTransmit,2);
  initializeTrajQueue();
}

void eepromUpdateByte(uint16_t address,uint8_t value){
  if(g_eepromImage[address]!=value){
    g_eeprom.updateByte(address,value);
    g_eepromImage[address]=value;
  }
}

void readConfigFromEeprom(){

  // doing a quick check that all data is not 255 (which would mean this is a brand new eeprom chip) and we don't want to get our config from it
  bool noDataFound=true;
  for(uint8_t i=1;i<71;i++){
    if(g_eeprom.readByte(i)!=255)
      noDataFound=false;
  }
  if(noDataFound){
    return;
  }

  g_eeprom.readBlock(0,g_eepromImage,EEPROM_SIZE);

  float_ui8 tempFloat;
  ui16_ui8 tempUi16;

  g_networkConfig.ip[0]=g_eepromImage[1];
  g_networkConfig.ip[1]=g_eepromImage[2];
  g_networkConfig.ip[2]=g_eepromImage[3];
  g_networkConfig.ip[3]=g_eepromImage[4];

  g_networkConfig.gatewayIp[0]=g_eepromImage[5];
  g_networkConfig.gatewayIp[1]=g_eepromImage[6];
  g_networkConfig.gatewayIp[2]=g_eepromImage[7];
  g_networkConfig.gatewayIp[3]=g_eepromImage[8];

  g_networkConfig.masks[0]=g_eepromImage[9];
  g_networkConfig.masks[1]=g_eepromImage[10];
  g_networkConfig.masks[2]=g_eepromImage[11];
  g_networkConfig.masks[3]=g_eepromImage[12];
  // deprecated :
  g_networkConfig.errorFeedbackIp[0]=g_eepromImage[13]; 
  g_networkConfig.errorFeedbackIp[1]=g_eepromImage[14];
  g_networkConfig.errorFeedbackIp[2]=g_eepromImage[15];
  g_networkConfig.errorFeedbackIp[3]=g_eepromImage[16];

  g_boardNumber=g_eepromImage[17];

  // 18 is unused

  g_Kp.ui8[0]=g_eepromImage[19];
  g_Kp.ui8[1]=g_eepromImage[20];
  g_Kp.ui8[2]=g_eepromImage[21];
  g_Kp.ui8[3]=g_eepromImage[22];

  g_Ki.ui8[0]=g_eepromImage[23];
  g_Ki.ui8[1]=g_eepromImage[24];
  g_Ki.ui8[2]=g_eepromImage[25];
  g_Ki.ui8[3]=g_eepromImage[26];
  
  g_Kd.ui8[0]=g_eepromImage[27];
  g_Kd.ui8[1]=g_eepromImage[28];
  g_Kd.ui8[2]=g_eepromImage[29];
  g_Kd.ui8[3]=g_eepromImage[30];

  g_maxVelocity.ui8[0]=g_eepromImage[31];
  g_maxVelocity.ui8[1]=g_eepromImage[32];
  g_maxVelocity.ui8[2]=g_eepromImage[33];
  g_maxVelocity.ui8[3]=g_eepromImage[34];

  g_errorLimit.ui8[0]=g_eepromImage[35];
  g_errorLimit.ui8[1]=g_eepromImage[36];

  
  g_motorStepsPerRev.ui8[0]=g_eepromImage[41];
  g_motorStepsPerRev.ui8[1]=g_eepromImage[42];
  g_motorStepsPerRev.ui8[2]=g_eepromImage[43];
  g_motorStepsPerRev.ui8[3]=g_eepromImage[44];
    
  g_encoderStepsPerRev.ui8[0]=g_eepromImage[45];
  g_encoderStepsPerRev.ui8[1]=g_eepromImage[46];
  g_encoderStepsPerRev.ui8[2]=g_eepromImage[47];
  g_encoderStepsPerRev.ui8[3]=g_eepromImage[48];

  g_invertedMotor=g_eepromImage[49]!=0;
  
  g_invertedEncoder=g_eepromImage[50]!=0;

  g_closedLoopEnabled=g_eepromImage[51]!=0;

  g_mqttConfig.brokerIp[0]=g_eepromImage[52];
  g_mqttConfig.brokerIp[1]=g_eepromImage[53];
  g_mqttConfig.brokerIp[2]=g_eepromImage[54];
  g_mqttConfig.brokerIp[3]=g_eepromImage[55];
  tempUi16.ui8[0]=g_eepromImage[56];
  tempUi16.ui8[1]=g_eepromImage[57];
  g_mqttConfig.brokerPort=tempUi16.ui16;

  g_mqttConfig.enabled=g_eepromImage[58]!=0;

  g_mqttConfig.pulseFeedbackEnabled=g_eepromImage[59]!=0;

  g_mqttConfig.encoderFeedbackEnabled=g_eepromImage[60]!=0;

  g_mqttConfig.setpointFeedbackEnabled=g_eepromImage[61]!=0;

  g_mqttConfig.timeStampFeedbackEnabled=g_eepromImage[62]!=0;

  tempFloat.ui8[0]=g_eepromImage[63];
  tempFloat.ui8[1]=g_eepromImage[64];
  tempFloat.ui8[2]=g_eepromImage[65];
  tempFloat.ui8[3]=g_eepromImage[66];
  g_mqttConfig.feedbackPublishPeriod=tempFloat.f;
  
  g_mqttConfig.errorFeedbackEnabled=g_eepromImage[69]!=0;

  // Updating slave according to what was read : 
  i2cSetHardwareConfig(g_motorStepsPerRev.i32,g_encoderStepsPerRev.i32,g_invertedMotor,g_invertedEncoder);
  i2cSetControlCoef(g_Kp.f,g_Ki.f,g_Kd.f,g_maxVelocity.f,g_errorLimit.ui16);
  if(g_closedLoopEnabled){
    i2cSetSlaveMode(CLOSEDLOOP_MODE);
  }else{
    i2cSetSlaveMode(OPENLOOP_MODE);
  }

  // MQTT username and passwords :
  uint8_t byteBuffer[MQTT_CRED_MAXSIZE+1];
  char charBuffer[MQTT_CRED_MAXSIZE+1];

  g_eeprom.readBlock(200,byteBuffer,MQTT_CRED_MAXSIZE+1);
  for(uint8_t i=0;i<MQTT_CRED_MAXSIZE+1;i++){
    charBuffer[i]=byteBuffer[i];
  }
  sprintf(g_mqttConfig.userName,charBuffer);

  g_eeprom.readBlock(300,byteBuffer,MQTT_CRED_MAXSIZE+1);
  for(uint8_t i=0;i<MQTT_CRED_MAXSIZE+1;i++){
    charBuffer[i]=byteBuffer[i];
  }
  sprintf(g_mqttConfig.password,charBuffer);
  if(SERIAL_PRINTS)
    Serial.println("config read from EEPROM");
}

void writeConfigToEeprom(){
  float_ui8 tempFloat;
  ui16_ui8 tempUi16;

  
    /* EEPROM data structure : 
    [data]                        [address]
    ip Address	                  1-4
    ip gateway	                  5-8
    network masks	                9-12
    errorFeedbackIp	              13-16 <- Deprecated
    boardNumber	                  17
    UNUSED      	                18
    Kp	                          19-22
    Ki	                          23-26
    Kd	                          27-30
    maxVelocity	                  31-34
    errorLimit	                  35-36
    Unused      	                37-40
    MotorStepsPerRev	            41-44
    EncoderTicksPerRev	          45-48
    InvertedMotor                 49
    InvertedEncoder               50
    controlLoop	                  51
    mqttBrokerIp	                52-55
    mqttBrokerPort	              56-57
    mqttEnabled	                  58
    mqttpulseFeedbackEnabled	  59
    mqttEncoderFeedbackEnabled	  60
    mqttSetPointFeedbackEnabled	  61
    mqttTimeStampFeedbackEnabled	62
    publishPeriod	                63-66
    httpErrorEnabled	            67 <- Deprecated
    httpWarningEnabled	          68 <- Deprecated
    mqttErrorEnabled	            69
    mqttWarningEnabled	          70 <- Not implemented
  */

  
  
  eepromUpdateByte(1,g_networkConfig.ip[0]);
  eepromUpdateByte(2,g_networkConfig.ip[1]);
  eepromUpdateByte(3,g_networkConfig.ip[2]);
  eepromUpdateByte(4,g_networkConfig.ip[3]);

  eepromUpdateByte(5,g_networkConfig.gatewayIp[0]);
  eepromUpdateByte(6,g_networkConfig.gatewayIp[1]);
  eepromUpdateByte(7,g_networkConfig.gatewayIp[2]);
  eepromUpdateByte(8,g_networkConfig.gatewayIp[3]);

  eepromUpdateByte(9,g_networkConfig.masks[0]);
  eepromUpdateByte(10,g_networkConfig.masks[1]);
  eepromUpdateByte(11,g_networkConfig.masks[2]);
  eepromUpdateByte(12,g_networkConfig.masks[3]);

  eepromUpdateByte(13,g_networkConfig.errorFeedbackIp[0]);
  eepromUpdateByte(14,g_networkConfig.errorFeedbackIp[1]);
  eepromUpdateByte(15,g_networkConfig.errorFeedbackIp[2]);
  eepromUpdateByte(16,g_networkConfig.errorFeedbackIp[3]);
  
  eepromUpdateByte(17,g_boardNumber);

  // 18 is unused

  eepromUpdateByte(19,g_Kp.ui8[0]);
  eepromUpdateByte(20,g_Kp.ui8[1]);
  eepromUpdateByte(21,g_Kp.ui8[2]);
  eepromUpdateByte(22,g_Kp.ui8[3]);

  eepromUpdateByte(23,g_Ki.ui8[0]);
  eepromUpdateByte(24,g_Ki.ui8[1]);
  eepromUpdateByte(25,g_Ki.ui8[2]);
  eepromUpdateByte(26,g_Ki.ui8[3]);

  eepromUpdateByte(27,g_Kd.ui8[0]);
  eepromUpdateByte(28,g_Kd.ui8[1]);
  eepromUpdateByte(29,g_Kd.ui8[2]);
  eepromUpdateByte(30,g_Kd.ui8[3]);

  eepromUpdateByte(31,g_maxVelocity.ui8[0]);
  eepromUpdateByte(32,g_maxVelocity.ui8[1]);
  eepromUpdateByte(33,g_maxVelocity.ui8[2]);
  eepromUpdateByte(34,g_maxVelocity.ui8[3]);

  eepromUpdateByte(35,g_errorLimit.ui8[0]);
  eepromUpdateByte(36,g_errorLimit.ui8[1]);

  eepromUpdateByte(41,g_motorStepsPerRev.ui8[0]);
  eepromUpdateByte(42,g_motorStepsPerRev.ui8[1]);
  eepromUpdateByte(43,g_motorStepsPerRev.ui8[2]);
  eepromUpdateByte(44,g_motorStepsPerRev.ui8[3]);

  eepromUpdateByte(45,g_encoderStepsPerRev.ui8[0]);
  eepromUpdateByte(46,g_encoderStepsPerRev.ui8[1]);
  eepromUpdateByte(47,g_encoderStepsPerRev.ui8[2]);
  eepromUpdateByte(48,g_encoderStepsPerRev.ui8[3]);

  eepromUpdateByte(49,g_invertedMotor);
  eepromUpdateByte(50,g_invertedEncoder);

  eepromUpdateByte(51,g_closedLoopEnabled);

  eepromUpdateByte(52,g_mqttConfig.brokerIp[0]);
  eepromUpdateByte(53,g_mqttConfig.brokerIp[1]);
  eepromUpdateByte(54,g_mqttConfig.brokerIp[2]);
  eepromUpdateByte(55,g_mqttConfig.brokerIp[3]);

  tempUi16.ui16=g_mqttConfig.brokerPort;
  eepromUpdateByte(56,tempUi16.ui8[0]);
  eepromUpdateByte(57,tempUi16.ui8[1]);
  
  eepromUpdateByte(58,g_mqttConfig.enabled);
  
  eepromUpdateByte(59,g_mqttConfig.pulseFeedbackEnabled);

  eepromUpdateByte(60,g_mqttConfig.encoderFeedbackEnabled);
  
  eepromUpdateByte(61,g_mqttConfig.setpointFeedbackEnabled);
  
  eepromUpdateByte(62,g_mqttConfig.timeStampFeedbackEnabled);

  tempFloat.f=g_mqttConfig.feedbackPublishPeriod;
  eepromUpdateByte(63,tempFloat.ui8[0]);  
  eepromUpdateByte(64,tempFloat.ui8[1]);  
  eepromUpdateByte(65,tempFloat.ui8[2]);  
  eepromUpdateByte(66,tempFloat.ui8[3]);

  eepromUpdateByte(69,g_mqttConfig.errorFeedbackEnabled);

  if(SERIAL_PRINTS)
    Serial.println("Config written to EEPROM");
}