#include <Arduino.h>
#include <Wire.h>
#include <WebServer_WT32_ETH01.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <I2C_eeprom.h>
#include <analogWrite.h>

// Hardware defines
#define I2C_SCL 2
#define I2C_SDA 14
#define PICO_ADDRESS 35
#define EEPROM_ADDRESS 80
#define PIN_RESET_BUTTON 32
#define PIN_LEDR 33
#define PIN_LEDG 5
#define PIN_LEDB 17

// Messages identifiers for the slave
#define MSGTYPE_SETNEXTREQUESTSIZE 78
#define MSGTYPE_SETMODE 101 // Used for setting config mode : closed loop or open loop
#define MSGTYPE_CONTROLCOEFS 102
#define MSGTYPE_HWCONFIG 103
#define MSGTYPE_APPENDTRAJ 104
#define MSGTYPE_SETPOINT 105
#define MSGTYPE_MOTORPOWER 106
#define MSGTYPE_MOTION 107
#define MSGTYPE_JOG 108
#define MSGTYPE_SETMOTORPOS 109
#define MSGTYPE_SETENCODERPOS 110
#define MSGTYPE_REBOOTSLAVE 66
#define MIN_I2C_USAGE_MS 5 // Minimum number of milliseconds between two usages of the i2c bus

// useful values for readability
#define CLOSEDLOOP_MODE 52
#define OPENLOOP_MODE 51
#define CONFIGURATION_MODE 60
#define MSG_GO 61
#define MSG_ABORT 62
#define MSG_STOP 63
#define LEDBRIGHTNESS 30

// Trajectory types : 
#define TRAJTYPE_DEFAULT 0
#define TRAJTYPE_LINEAR 1
#define TRAJTYPE_SPLINE 2
#define TRAJTYPE_BEZIER 3
#define RELATIVE_POSITION 10
#define ABSOLUTE_POSITION 11

// I2C related defines :
#define MAX_RECEIVE_SIZE 46
#define EEPROM_SIZE 75 // this is the size used for configuration (excluding mqtt credentials), not the whole size
#define MQTT_CRED_MAXSIZE 50

// Trajectory queue related defines : 
#define TRAJ_BUFFER_SIZE 50
#define TRAJ_N_POINTS_MAX 128

// Ethernet and MQTT settings
#define DEBUG_ETHERNET_WEBSERVER_PORT       Serial
#define _ETHERNET_WEBSERVER_LOGLEVEL_       0 // Debug Level from 0 to 4
#define MQTT_STATUS_POST_INTERVAL_MS       5000L

// Error codes : 
#define ERRCODE_LIMITABORT 90

struct networkConfiguration{
  uint8_t ip[4];
  uint8_t gatewayIp[4];
  uint8_t masks[4];
  uint8_t errorFeedbackIp[4];
};
struct networkConfiguration g_networkConfig;

struct mqttConfiguration{
  uint8_t brokerIp[4];
  uint16_t brokerPort;
  bool enabled;

  bool pulseFeedbackEnabled;
  bool encoderFeedbackEnabled;
  bool setpointFeedbackEnabled;
  bool timeStampFeedbackEnabled;

  bool errorFeedbackEnabled;

  float feedbackPublishPeriod;

  char userName[MQTT_CRED_MAXSIZE+1];
  char password[MQTT_CRED_MAXSIZE+1];
};
struct mqttConfiguration g_mqttConfig;

struct mqttTopics{
  char motorsOn[10];
  char motorOn[11];
  char motorsOff[11];
  char motorOff[12];
  char motorsEnableCL[25];
  char motorEnableCL[26];
  char motorsDisableCL[27];
  char motorDisableCL[28];
  char motorsGo[10];
  char motorGo[11];
  char motorsStop[12];
  char motorStop[13];
  char motorsAbort[13];
  char motorAbort[14];
  char feedback[17];
  char errorFeedback[14];
  char queue[14];
};
struct mqttTopics g_mqttTopics;

struct errorFeedbackSettings{
  bool httpErrorEnabled;
  bool httpWarningEnabled;
  bool mqttErrorEnabled;
  bool mqttWarningEnabled;
};
struct errorFeedbackSettings g_errorFeedbackSettings;


struct trajectory{
  uint8_t nPoints;
  uint8_t type;
  uint16_t id;
  int32_t position[TRAJ_N_POINTS_MAX];
  uint32_t time[TRAJ_N_POINTS_MAX];
  uint8_t positionRelativity;
};

struct valuesOffset{
  int32_t pulses;
  float encoder;
  int32_t time_ms;
};
struct valuesOffset g_valuesOffset;


// Useful unions for conversion of variable into bytes arrays
union i32_ui8 {
    int32_t i32;
    uint8_t ui8[4];
};

union float_ui8 {
    float f;
    uint8_t ui8[4];
};

union ui16_ui8 {
    uint16_t ui16;
    uint8_t ui8[2];
};

union ui32_ui8 {
    uint32_t ui32;
    uint8_t ui8[4];
};

// i2c related function declarations
bool i2cPing(uint8_t);
uint8_t i2cGetDataFromSlave(uint8_t*, int);
uint8_t i2cSendDataToSlave(uint8_t*, uint8_t);
uint8_t i2cSetNextRequestSize(uint8_t);
bool i2cGetFullData();
bool i2cGetMotionData();
void i2cSendTrajPoint(uint8_t,uint8_t,uint16_t,uint8_t,int32_t, uint32_t);
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
void processErrorCodes();

// trajectory related functions declaration : 
void initializeTrajQueue();
void addTrajectoryToQueue(trajectory);
void removeTrajectoryFromQueue();
void updateTrajectoryQueue();

// Network functions :
void initializeEthernetConnection();
void initializeMqttConnection();

// default config & button press: 
void setDefaultConfiguration();
void manageButtonPress();

// http server callbacks 
void httpRootCallback();
void httpNotFoundCallback();
void httpFaviconCallback();
void httpNetworkconfigCallback();
void httpControlParamsCallback();
void httpHwConfigCallback();
void httpMotorStatusCallback();
void httpMotorOnCallback();
void httpMotorOffCallback();
void httpEnableClosedLoopCallback();
void httpDisableClosedLoopCallback();
void httpGoCallback();
void httpStopCallback();
void httpSetPointCallback();
void httpPositionCallback();
void httpEncoderCallback();
void httpTimestampCallback();
void httpFeedbackCallback();
void httpSetValuesCallback();
void httpJogCallback();
void httpAbortCallback();
void httpTrajLinearCallback();
void httpMqttConfigCallback();
void httpQueueCallback();
void httpRebootCallback();

// mqtt callback and publication functions : 
void mqttCallback(char*, byte*, unsigned int);
void publishMqttTopics();

// Setters from JSON : 
bool setNetworkConfigFromJson(String);
bool setControlParamsFromJson(String);
bool setHwConfigFromJson(String);

// eeprom related functions : 
void writeConfigToEeprom();
void readConfigFromEeprom();
void eepromUpdateByte(uint16_t, uint8_t);

// Miscellanous functions declaration :
void printData(uint8_t*, uint8_t);
bool stringIpToByte(String,uint8_t[]);
void manageLedColor();

// html related functions : 
String generateHomePage();
String generateNetworkConfigHtmlForm();
String generateControlParamHtmlForm();
String generateHwConfigHtmlForm();
String generateHtmlPowerButtons();
String generateHtmlclosedLoopButtons();
String generateHtmlBasicPosition();
String generateHtmlBasicVelocity();
String generateHtmlMqttconfig();
String generateHtmlReboot();

// Trajectory-related global variables : 
trajectory g_trajectoryQueue[TRAJ_BUFFER_SIZE];
uint16_t g_trajEmptySlotIndex;

// Useful global variables;
uint8_t g_nextRequestSize;
uint32_t g_lastI2CdateTime;
uint8_t g_boardNumber;
char g_deviceName[15];
uint8_t g_eepromImage[EEPROM_SIZE];

// Motion global variables : 
i32_ui8 g_currentMotorPulses;
i32_ui8 g_currentSetPoint;
float_ui8 g_currentEncoderPosition;
uint8_t g_nTrajSlotsAvailableOnSlave;
ui16_ui8 g_curentActiveTraj;
bool g_isMotorOn;
bool g_closedLoopEnabled;
uint8_t g_errorCodesFromSlave;
float_ui8 g_Kp;
float_ui8 g_Ki;
float_ui8 g_Kd;
bool g_invertedMotor;
bool g_invertedEncoder;
i32_ui8 g_motorStepsPerRev;
i32_ui8 g_encoderStepsPerRev;
ui16_ui8 g_errorLimit;
float_ui8 g_maxVelocity;
uint16_t g_lastTrajectoryIdReceived;

// eth & mqtt clients : 
WiFiClient g_ethClient; // Note : the type is WifiClient, but this is effectively a wired eth client
PubSubClient g_mqttClient(g_ethClient);
WebServer g_server(80);

// EEPROM chip : 
I2C_eeprom g_eeprom(EEPROM_ADDRESS);

// Tasks definition for dual core usage : 
TaskHandle_t mainTask;
TaskHandle_t mqttPublishTask;
void mainLoop(void*);
void mqttPublishLoop(void*);

// ************************************************************************************
//                                   setup & loop
// ************************************************************************************
void setup() {
  pinMode(PIN_LEDR,OUTPUT);
  pinMode(PIN_LEDG,OUTPUT);
  pinMode(PIN_LEDB,OUTPUT);
  pinMode(PIN_RESET_BUTTON,INPUT);

  // entering setup : light up the LED blue :
  analogWrite(PIN_LEDB,LEDBRIGHTNESS);

  Serial.begin(9600);
  
  // I2C setup : 
  Serial.println("Setting up I2C communication");
  while(!Wire.begin(I2C_SDA, I2C_SCL,(uint32_t)400000)) {
	  delay(200);
  }
  while(!i2cPing(PICO_ADDRESS)){ // the encoder reader is mandatory : we keep trying until we find it
    Serial.println("could not reach pico, trying again in 0.1s");
    delay(100);
  }
  Serial.println("Pico communication up and running");

  while(!i2cPing(EEPROM_ADDRESS)){
    Serial.println("could not reach EEPROM chip, trying again in 0.1s");
    delay(100);
  }
  if(g_eeprom.readByte(0)!=42){ // This is a trick to detect if this is the first time the board is booted up
    setDefaultConfiguration();
    g_eeprom.writeByte(0,42);
  }else{
    readConfigFromEeprom();
  }
  Serial.println("EEPROM communication up and running");

  // global variables initialization : 
  g_nextRequestSize=0;
  g_lastTrajectoryIdReceived=0;
  g_boardNumber=0;
  sprintf(g_deviceName,"DirStepBoard%02d",g_boardNumber);

  // initializing mqtt topics : 
  sprintf(g_mqttTopics.motorsOn,"motors/on");
  sprintf(g_mqttTopics.motorOn,"motor%02d/on",g_boardNumber);
  sprintf(g_mqttTopics.motorsOff,"motors/off");
  sprintf(g_mqttTopics.motorOff,"motor%02d/off",g_boardNumber);
  sprintf(g_mqttTopics.motorsEnableCL,"motors/closedLoop/enable");
  sprintf(g_mqttTopics.motorEnableCL,"motor%02d/closedLoop/enable",g_boardNumber);
  sprintf(g_mqttTopics.motorsDisableCL,"motors/closedLoop/disable");
  sprintf(g_mqttTopics.motorDisableCL,"motor%02d/closedLoop/disable",g_boardNumber);
  sprintf(g_mqttTopics.motorsGo,"motors/go");
  sprintf(g_mqttTopics.motorGo,"motor%02d/go",g_boardNumber);
  sprintf(g_mqttTopics.motorsStop,"motors/stop");
  sprintf(g_mqttTopics.motorStop,"motor%02d/stop",g_boardNumber);
  sprintf(g_mqttTopics.motorsAbort,"motors/abort");
  sprintf(g_mqttTopics.motorAbort,"motor%02d/abort",g_boardNumber);
  sprintf(g_mqttTopics.feedback,"motor%02d/feedback",g_boardNumber);
  sprintf(g_mqttTopics.errorFeedback,"motor%02d/error",g_boardNumber);
  sprintf(g_mqttTopics.queue,"motor%02d/queue",g_boardNumber);

  initializeTrajQueue();

  // network Setup : 
  initializeEthernetConnection();

  xTaskCreatePinnedToCore(mainLoop,"mainLoop",10000,NULL,0,&mainTask,0);
  xTaskCreatePinnedToCore(mqttPublishLoop,"mqttPublishTask",10000,NULL,1,&mqttPublishTask,1);

}

void loop() {} // We use freertos tasks instead of loop()

void mainLoop(void* pvParameters){ // runs on core 0 
  while(true){
    i2cGetMotionData();
	  processErrorCodes();
    updateTrajectoryQueue();
	  manageButtonPress();
	  manageLedColor();
    g_server.handleClient();
  }
}

void mqttPublishLoop(void* pvParameters){ // runs on core 1
  initializeMqttConnection();
  while(true){
    if(g_mqttConfig.enabled && g_mqttClient.connected()){
      publishMqttTopics();
      g_mqttClient.loop();
    }
    if(g_mqttConfig.enabled && !g_mqttClient.connected()){
      initializeMqttConnection();
    }

  }
}
// ************************************************************************************
//                                i2c functions definitions
// ************************************************************************************
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
// ************************************************************************************
//                           Trajectory related functions
// ************************************************************************************
void initializeTrajQueue(){
  for(uint16_t i=0;i<TRAJ_BUFFER_SIZE;i++){
    g_trajectoryQueue[i].nPoints=0;
    g_trajectoryQueue[i].type=TRAJTYPE_DEFAULT;
    g_trajectoryQueue[i].id=0;
    for(uint16_t j=0;j<TRAJ_N_POINTS_MAX;j++){
      g_trajectoryQueue[i].position[j]=0;
      g_trajectoryQueue[i].time[j]=0;
    }
    g_trajectoryQueue[i].positionRelativity=RELATIVE_POSITION;
  }
  g_trajEmptySlotIndex=0;
}

void addTrajectoryToQueue(trajectory trajToAdd){
  if(g_trajEmptySlotIndex<TRAJ_BUFFER_SIZE){
    g_trajectoryQueue[g_trajEmptySlotIndex++]=trajToAdd;
  }
}

void removeTrajectoryFromQueue(){
  // implicit : will remove the first item (aka g_trajectoryQueue[0])
  for(uint16_t i=0;i<TRAJ_BUFFER_SIZE-1;i++){
    g_trajectoryQueue[i]=g_trajectoryQueue[i+1];
  }
  g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].nPoints=0;
  g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].type=TRAJTYPE_DEFAULT;
  g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].id=0;
  for(uint16_t j=0;j<TRAJ_N_POINTS_MAX;j++){
    g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].position[j]=0;
    g_trajectoryQueue[TRAJ_BUFFER_SIZE-1].time[j]=0;
  }
  if(g_trajEmptySlotIndex>0)
    g_trajEmptySlotIndex--;
}

void updateTrajectoryQueue(){
  // this assumes a call to i2cGetMotionData has been made prior to calling this function, in order to have an accurate view of the
  // number of slots available in the slave (max 2)
  // One call of this function transmits one point of trajectory through the I2C communication
  static uint8_t nextPointToBeTransmitted;

  if(g_trajEmptySlotIndex==0)
    return; // There is nothing to send
  
  if(g_nTrajSlotsAvailableOnSlave==0)
    return; // There is no slot on the slave to receive a trajectory

  i2cSendTrajPoint( nextPointToBeTransmitted,
                    g_trajectoryQueue[0].nPoints,
                    g_trajectoryQueue[0].id,
                    g_trajectoryQueue[0].type,
                    g_trajectoryQueue[0].position[nextPointToBeTransmitted],
                    g_trajectoryQueue[0].time[nextPointToBeTransmitted],
                    g_trajectoryQueue[0].positionRelativity);
  nextPointToBeTransmitted++;
  if(nextPointToBeTransmitted>=g_trajectoryQueue[0].nPoints){
    removeTrajectoryFromQueue();
    nextPointToBeTransmitted=0;
  } 
}

void processErrorCodes(){
  if(g_errorCodesFromSlave==0){
    return;
  }
  Serial.print("Error code from slave : ");
  Serial.println(g_errorCodesFromSlave);
  
  // Do a publish if mqtt feedback is enabled
  String payload="{";
  payload+="\"code\":"+String(g_errorCodesFromSlave)+",";
  payload+="\"timeStamp\":"+String((millis()+g_valuesOffset.time_ms)/1000.)+",";
  payload+="\"description\":\"";
  if(g_errorCodesFromSlave==ERRCODE_LIMITABORT){
    payload+="Error limit triggered, motion aborted. Closed loop disabled, motor off";
  }
  payload+="\"}";
  
  if(g_mqttConfig.enabled && g_mqttConfig.errorFeedbackEnabled){
	  g_mqttClient.publish(g_mqttTopics.errorFeedback,payload.c_str());
  }
  g_errorCodesFromSlave=0;
}

// ************************************************************************************
//                           network related functions
// ************************************************************************************
void initializeEthernetConnection()
{
  WT32_ETH01_onEvent(); // When this line is done, the message "ETH started" is printed on serial port
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER);
  IPAddress ip(g_networkConfig.ip);
  IPAddress gatewayip(g_networkConfig.gatewayIp);
  IPAddress maskip(g_networkConfig.masks);
  ETH.config(ip, gatewayip, maskip);
  
  while (!WT32_ETH01_eth_connected){
    delay(100);
  }
  
  // Starting the web server : 
  g_server.on(F("/"), httpRootCallback);
  g_server.on(F("/favicon.ico"),httpFaviconCallback);
  g_server.on(F("/networkConfig"),httpNetworkconfigCallback);
  g_server.on(F("/controlParameters"),httpControlParamsCallback);
  g_server.on(F("/hardwareConfig"),httpHwConfigCallback);
  g_server.on(F("/motor"),httpMotorStatusCallback);
  g_server.on(F("/motor/on"),httpMotorOnCallback);
  g_server.on(F("/motor/off"),httpMotorOffCallback);
  g_server.on(F("/motor/closedLoop/enable"),httpEnableClosedLoopCallback);
  g_server.on(F("/motor/closedLoop/disable"),httpDisableClosedLoopCallback);
  g_server.on(F("/go"),httpGoCallback);
  g_server.on(F("/stop"),httpStopCallback);
  g_server.on(F("/setPoint"),httpSetPointCallback);
  g_server.on(F("/pulse"),httpPositionCallback);
  g_server.on(F("/encoder"),httpEncoderCallback);
  g_server.on(F("/timeStamp"),httpTimestampCallback);
  g_server.on(F("/feedback"),httpFeedbackCallback);
  g_server.on(F("/setValues"),httpSetValuesCallback);
  g_server.on(F("/jog"),httpJogCallback);
  g_server.on(F("/abort"),httpAbortCallback);
  g_server.on(F("/trajectory/linear"),httpTrajLinearCallback);
  g_server.on(F("/mqttConfig"),httpMqttConfigCallback);
  g_server.on(F("/queue"),httpQueueCallback);
  g_server.on(F("/reboot"),httpRebootCallback);
  g_server.onNotFound(httpNotFoundCallback);
  g_server.begin();
  
  Serial.print(F("HTTP Server started @ IP : "));
  Serial.println(ETH.localIP());


}

void initializeMqttConnection(){
  if(g_mqttConfig.enabled){
    g_mqttClient.setServer(g_mqttConfig.brokerIp,g_mqttConfig.brokerPort);
    g_mqttClient.setCallback(mqttCallback);
    for(uint8_t i=0;i<10;i++){ // let us try 10 times to connect
        if(!g_mqttClient.connected()){
          if(strcmp(g_mqttConfig.userName,"")==0){
            g_mqttClient.connect(g_deviceName);
          }else{
            g_mqttClient.connect(g_deviceName,g_mqttConfig.userName,g_mqttConfig.password);
          }
          delay(150);
      }
    }
    if(!g_mqttClient.connected()){
      Serial.println("Mqtt connection failed");
      return;
    }
    g_mqttClient.subscribe(g_mqttTopics.motorOn);
    g_mqttClient.subscribe(g_mqttTopics.motorsOn);
    g_mqttClient.subscribe(g_mqttTopics.motorOff);
    g_mqttClient.subscribe(g_mqttTopics.motorsOff);
    g_mqttClient.subscribe(g_mqttTopics.motorsEnableCL);
    g_mqttClient.subscribe(g_mqttTopics.motorEnableCL);
    g_mqttClient.subscribe(g_mqttTopics.motorsDisableCL);
    g_mqttClient.subscribe(g_mqttTopics.motorDisableCL);
    g_mqttClient.subscribe(g_mqttTopics.motorGo);
    g_mqttClient.subscribe(g_mqttTopics.motorsGo);
    g_mqttClient.subscribe(g_mqttTopics.motorAbort);
    g_mqttClient.subscribe(g_mqttTopics.motorsAbort);
    Serial.println("Connected to mqtt");
  }
}

// ************************************************************************************
//                           HTTP callbacks functions
// ************************************************************************************
void httpRootCallback(){
  Serial.println("Received root request");
  String currentpage=generateHomePage();

  g_server.send(200, F("text/html"),currentpage);
}

void httpNotFoundCallback(){
  Serial.println("not found handle!");
  String message = F("File Not Found\n\n");
  message += F("URI: ");
  message += g_server.uri();
  message += F("\nMethod: ");
  message += (g_server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("\nArguments: ");
  message += g_server.args();
  message += F("\n");
  for (uint8_t i = 0; i < g_server.args(); i++){
    message += " " + g_server.argName(i) + ": " + g_server.arg(i) + "\n";
  }
  g_server.send(404, F("text/plain"), message);
  Serial.println(message);
}

void httpFaviconCallback(){ // todo : this does not work, but I don't really care for now.
  const char favicon[] PROGMEM = {
    0x20,0x20,0x01,0x20,0x01,0x20,0x10,0x10,0x20,0x20,
    0x20,0x20,0x20,0x20,0x77,0x20,0x20,0x20,0x16,0x20,
    0x20,0x20,0x89,0x50,0x4e,0x47,0x0d,0x0a,0x1a,0x0a,
    0x20,0x20,0x20,0x0d,0x49,0x48,0x44,0x52,0x20,0x20,
    0x20,0x10,0x20,0x20,0x20,0x10,0x08,0x06,0x20,0x20,
    0x20,0x1f,0xf3,0xff,0x61,0x20,0x20,0x20,0x3e,0x49,
    0x44,0x41,0x54,0x78,0x9c,0x63,0xfc,0xff,0xff,0x3f,
    0x03,0x25,0x80,0x89,0x22,0xdd,0xc3,0xd8,0x20,0x46,
    0x46,0xc6,0xff,0xf8,0xf8,0x44,0xb9,0x20,0xa6,0x09,
    0x9f,0x66,0xbc,0x06,0x10,0x0b,0xf0,0x1a,0x40,0xc8,
    0x76,0xbc,0x06,0xfc,0xff,0xff,0x9f,0x11,0x99,0x26,
    0xcb,0x05,0xc4,0x20,0xc6,0xd1,0xa4,0x4c,0xb9,0x01,
    0x20,0x42,0xac,0x11,0x1f,0x36,0x28,0x3a,0x36,0x20,
    0x20,0x20,0x20,0x49,0x45,0x4e,0x44,0xae,0x42,0x60,
    0x82
  };
  g_server.send_P(200, "image/x-icon", favicon, sizeof(favicon));
}

void httpNetworkconfigCallback(){
  if(g_server.method()==HTTP_PUT){
    Serial.println("Incoming PUT to network configuration");
    String payload=g_server.arg("plain");
    if(setNetworkConfigFromJson(payload)){
      g_server.send(200,F("text/html"), "Received");
    }else{
      g_server.send(400,F("text/html"), "Bad Request");
    }
    writeConfigToEeprom();
    // Should I reboot or is there a smarter way to change IP. There must be one.
    return;
  } else if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to network configuration");
    String payload="{";
    payload+="\"ipAddress\":\""+String(g_networkConfig.ip[0])+"."+String(g_networkConfig.ip[1])+"."+String(g_networkConfig.ip[2])+"."+String(g_networkConfig.ip[3])+"\",";
    payload+="\"gatewayIp\":\""+String(g_networkConfig.gatewayIp[0])+"."+String(g_networkConfig.gatewayIp[1])+"."+String(g_networkConfig.gatewayIp[2])+"."+String(g_networkConfig.gatewayIp[3])+"\",";
    payload+="\"networkMasks\":\""+String(g_networkConfig.masks[0])+"."+String(g_networkConfig.masks[1])+"."+String(g_networkConfig.masks[2])+"."+String(g_networkConfig.masks[3])+"\"";
    payload+='}';
    g_server.send(200, F("text/html"),payload);
    return;
  }
  g_server.send(405, F("text/html"),"Method Not Allowed");
}

void httpControlParamsCallback(){
  if(g_server.method()==HTTP_PUT){
    Serial.println("Incoming PUT to control Parameters");
    String payload=g_server.arg("plain");
    if(setControlParamsFromJson(payload)){
      writeConfigToEeprom();
      g_server.send(200,F("text/html"), "Received");
      return;
    }else{
      g_server.send(400,F("text/html"), "Bad Request");
      return;
    }
  }else if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to configuration");
    String payload="{";
    payload+="\"Kp\":"+String(g_Kp.f)+",";
    payload+="\"Ki\":"+String(g_Ki.f)+",";
    payload+="\"Kd\":"+String(g_Kd.f)+",";
    payload+="\"maxVelocity\":"+String(g_maxVelocity.f)+",";
    payload+="\"errorLimit\":"+String(g_errorLimit.ui16);
    payload+='}';
    g_server.send(200, F("text/html"),payload);
    return;
  }
  g_server.send(405, F("text/html"),"Method Not Allowed");
}

void httpHwConfigCallback(){
  if(g_server.method()==HTTP_PUT){
    Serial.println("Incoming PUT to hardware config");
    String payload=g_server.arg("plain");
    if(setHwConfigFromJson(payload)){
      writeConfigToEeprom();
      g_server.send(200,F("text/html"), "Received");
      return;
    }else{
      g_server.send(400,F("text/html"), "Bad Request");
      return;
    }
  }else if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to hardware config");
    String payload="{";
    payload+="\"motorStepsPerRev\":";
    if(g_invertedMotor){
      payload+="-";
    }
    payload+=String(g_motorStepsPerRev.i32)+",";
    payload+="\"encoderTicksPerRev\":";
    if(g_invertedEncoder){
      payload+="-";
    }
    payload+=String(g_encoderStepsPerRev.i32);
    payload+="}";
   
    g_server.send(200, F("text/html"),payload);
    return;
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}
void httpMotorStatusCallback(){
  if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to Motor Status");
    String payload="{";
    payload+="\"motorOn\":";
    if(g_isMotorOn) payload+="true,";
    else payload+="false,";
    payload+="\"closedLoopEnabled\":";
    if(g_closedLoopEnabled) payload+="true";
    else payload+="false";
    payload+="}";
   
    g_server.send(200, F("text/html"),payload);
    return;
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }

}

void httpMotorOnCallback(){
  i2cMotorOn();
  i2cGetFullData();
  processErrorCodes();
  g_server.send(200, F("text/html"),"OK");
}

void httpMotorOffCallback(){
  i2cMotorOff();
  i2cGetFullData();
  processErrorCodes();
  g_server.send(200, F("text/html"),"OK");
}

void httpEnableClosedLoopCallback(){
  if(g_encoderStepsPerRev.i32!=0){
    g_closedLoopEnabled=true;
    i2cSetSlaveMode(CLOSEDLOOP_MODE);
    i2cGetFullData();
    processErrorCodes();
    g_server.send(200, F("text/html"),"OK");
  }else{
    g_server.send(409, F("text/html"),"Refused : encoder steps per revolution currently set to 0");
  }
}

void httpDisableClosedLoopCallback(){
  g_closedLoopEnabled=true;
  i2cSetSlaveMode(OPENLOOP_MODE);
  i2cGetFullData();
  processErrorCodes();
  g_server.send(200, F("text/html"),"OK");
}

void httpGoCallback(){
  Serial.println("Received a Go");
  i2cGo();
  g_server.send(200, F("text/html"),"OK");
}

void httpStopCallback(){
  Serial.println("Received a Stop");
  i2cStop();
  g_server.send(200, F("text/html"),"OK");
}

void httpSetPointCallback(){
  if(g_server.method()==HTTP_POST){
    Serial.println("Incoming POST to setPoint");
    String payload=g_server.arg("plain");
    JsonDocument currentJson;
    DeserializationError jsonErr=deserializeJson(currentJson, payload);
    if(jsonErr){
      Serial.print("JSON error : ");
      Serial.println(jsonErr.c_str());
      g_server.send(400,F("text/html"), "Bad Request");
      return;
    }
    
    int32_t targetReceived;
    if(currentJson.containsKey("targetPosition")){
      targetReceived=currentJson["targetPosition"];
    }else{
      g_server.send(400,F("text/html"), "Bad Request");
      return;
    }

    bool relativePosition=true;
    if(currentJson.containsKey("type")){
      String typeReceived=currentJson["type"];
      if(typeReceived=="ABSOLUTE"){
        relativePosition=false;
      }else if(typeReceived=="RELATIVE"){
        relativePosition=true;
      }else{
        g_server.send(400,F("text/html"), "Bad Request : type must be either \"ABSOLUTE\" or \"RELATIVE\"");
        return;
      }
    }

    if(relativePosition){
      i2cGetMotionData();
      i2cSetPoint(g_currentSetPoint.i32+targetReceived);
      g_server.send(200, F("text/html"),"OK");
    }else{
      if(!g_closedLoopEnabled){
        i2cSetPoint(targetReceived-g_valuesOffset.pulses);
      }else{
        i2cSetPoint(targetReceived-g_valuesOffset.encoder);
      }
      g_server.send(200, F("text/html"),"OK");
    }
  }else if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to setPoint");
    i2cGetMotionData();
    g_server.send(200, F("text/html"),String(g_currentSetPoint.i32));
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}

void httpPositionCallback(){
  if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to Position");
    i2cGetMotionData();
    g_server.send(200, F("text/html"),String(g_currentMotorPulses.i32));
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}

void httpEncoderCallback(){
  if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to Encoder");
    i2cGetMotionData();
    g_server.send(200, F("text/html"),String(g_currentEncoderPosition.f));
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}
void httpTimestampCallback(){
  if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to timeStamp");
    g_server.send(200, F("text/html"),String(float(millis()/1000.)));
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}
void httpFeedbackCallback(){
  if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to full Feedback");
    i2cGetMotionData();
    uint32_t currentTime=millis()+g_valuesOffset.time_ms;
    String payload="{";
    payload+="\"pulses\":"+String(g_currentMotorPulses.i32+g_valuesOffset.pulses)+",";
    payload+="\"encoder\":"+String(g_currentEncoderPosition.f+g_valuesOffset.encoder)+",";
    payload+="\"timeStamp\":"+String(float(currentTime/1000.))+",";
    if(g_closedLoopEnabled){
      payload+="\"setPoint\":"+String(g_currentSetPoint.i32+g_valuesOffset.encoder)+"}";
    }else{
      payload+="\"setPoint\":"+String(g_currentSetPoint.i32+g_valuesOffset.pulses)+"}";
    }
    g_server.send(200, F("text/html"),payload);
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}

void httpSetValuesCallback(){
  if(g_server.method()==HTTP_PUT){
    Serial.println("Incoming PUT to setValues");
    
    String payload=g_server.arg("plain");
    JsonDocument currentJson;
    DeserializationError jsonErr=deserializeJson(currentJson, payload);
    if(jsonErr){
      Serial.print("JSON error : ");
      Serial.println(jsonErr.c_str());
      g_server.send(400,F("text/html"), "Bad Request");
      return;
    }

    if(currentJson.containsKey("pulses")){
      g_valuesOffset.pulses=currentJson["pulses"];
      g_valuesOffset.pulses-=g_currentMotorPulses.i32;
    }

    if(currentJson.containsKey("encoder")){
      g_valuesOffset.encoder=currentJson["encoder"];
      g_valuesOffset.encoder-=g_currentEncoderPosition.f;
    }

    if(currentJson.containsKey("timeStamp")){
      g_valuesOffset.time_ms=currentJson["timeStamp"];
      g_valuesOffset.time_ms=g_valuesOffset.time_ms*1000-millis();
    }

    Serial.println(g_valuesOffset.pulses);
    Serial.println(g_valuesOffset.encoder);
    Serial.println(g_valuesOffset.time_ms);
    

    g_server.send(200, F("text/html"),"OK");
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }

}

void httpJogCallback(){
  if(g_server.method()==HTTP_POST){
    Serial.println("Incoming POST to jog");
    String payload=g_server.arg("plain");
    JsonDocument currentJson;
    DeserializationError jsonErr=deserializeJson(currentJson, payload);
    if(jsonErr){
      Serial.print("JSON error : ");
      Serial.println(jsonErr.c_str());
      g_server.send(400,F("text/html"), "Bad Request");
      return;
    }

    if(currentJson.containsKey("velocity")){
      float velocityReceived=currentJson["velocity"];
      i2cSetJogSpeed(velocityReceived);
      g_server.send(200,F("text/html"), "OK");
    }else{
      g_server.send(400,F("text/html"), "Bad Request : JSON must contain key \"velocity\"");
      return;
    }
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}

void httpAbortCallback(){
  i2cAbort();
  g_server.send(200,F("text/html"), "OK");
}

void httpTrajLinearCallback(){
  Serial.println("Incoming POST to Linear Trajectory");
  String payload=g_server.arg("plain");
  JsonDocument currentJson;
  DeserializationError jsonErr=deserializeJson(currentJson, payload);
  if(jsonErr){
    Serial.print("JSON error : ");
    Serial.println(jsonErr.c_str());
    g_server.send(400,F("text/html"), "Bad Request");
    return;
  }

  uint16_t nPositionsReceived=0;
  uint16_t nTimesReceived=0;
  int32_t positionsReceived[128];
  uint32_t timesReceived[128];
  if(currentJson.containsKey("position")){
    nPositionsReceived=copyArray(currentJson["position"],positionsReceived);
  }else{
    g_server.send(400,F("text/html"), "Bad Request : must have a \"position\" key");
    return;
  }
  
  if(currentJson.containsKey("time")){
    nTimesReceived=copyArray(currentJson["time"],timesReceived);
  }else{
    g_server.send(400,F("text/html"), "Bad Request : must have a \"time\" key");
    return;
  }

  if(nTimesReceived!=nPositionsReceived){
    g_server.send(400,F("text/html"), "Bad Request : time and position arrays must have the same length");
  }
  if(nTimesReceived>=128 || nPositionsReceived>128){
    g_server.send(400,F("text/html"), "Bad Request : maximum 128 points supported");
  }

  trajectory currentTraj;
  currentTraj.nPoints=nPositionsReceived;
  currentTraj.type=TRAJTYPE_LINEAR; 
  
  if(currentJson.containsKey("id")){
    currentTraj.id=(uint16_t)currentJson["id"];
    g_lastTrajectoryIdReceived=currentTraj.id;
  }else{
    currentTraj.id=++g_lastTrajectoryIdReceived;
  }

  if(currentJson.containsKey("positionRelativity")){
    String relativityReceived=currentJson["positionRelativity"];
    if(relativityReceived=="ABSOLUTE"){
      currentTraj.positionRelativity=ABSOLUTE_POSITION;
    }else if(relativityReceived=="RELATIVE"){
      currentTraj.positionRelativity=RELATIVE_POSITION;
    }else{
      g_server.send(400,F("text/html"), "Bad Request : positionRelativity must be either \"ABSOLUTE\" or \"RELATIVE\"");
    }
  }else{
    currentTraj.positionRelativity=RELATIVE_POSITION;
  }

  for(uint8_t i=0;i<nPositionsReceived;i++){
    currentTraj.position[i]=positionsReceived[i];
    if(currentTraj.positionRelativity==ABSOLUTE_POSITION && g_closedLoopEnabled){
      currentTraj.position[i]-=g_valuesOffset.encoder;
    }
    if(currentTraj.positionRelativity==ABSOLUTE_POSITION && !g_closedLoopEnabled){
      currentTraj.position[i]-=g_valuesOffset.pulses;
    }
    currentTraj.time[i]=timesReceived[i]*1000; // received in seconds from the JSON, but processed in milliseconds elsewhere
  }
  addTrajectoryToQueue(currentTraj);
  g_server.send(200,F("text/html"), "OK");
}

void httpMqttConfigCallback(){
  if(g_server.method()==HTTP_PUT){
    Serial.println("Incoming PUT to Mqtt Configuration");
    String payload=g_server.arg("plain");
    Serial.println(payload);
    JsonDocument currentJson;
    DeserializationError jsonErr=deserializeJson(currentJson, payload);
    if(jsonErr){
      Serial.print("JSON error : ");
      Serial.println(jsonErr.c_str());
      g_server.send(400,F("text/html"), "Bad Request");
      return;
    }

    if(currentJson.containsKey("brokerIp")){
      if(!stringIpToByte(currentJson["brokerIp"],g_mqttConfig.brokerIp)){
        g_server.send(400,F("text/html"), "Broker Ip format error");
      }
    }

    if(currentJson.containsKey("brokerPort")){
      g_mqttConfig.brokerPort=currentJson["brokerPort"];
    }

    if(currentJson.containsKey("mqttEnabled")){
      g_mqttConfig.enabled=currentJson["mqttEnabled"];
    }

    if(currentJson.containsKey("boardNumber")){
      if(currentJson["boardNumber"]<0)
        g_boardNumber=0;
      else if(currentJson["boardNumber"]>99)
        g_boardNumber=99;
      else 
        g_boardNumber=currentJson["boardNumber"];
      // must reboot, as it may change the mqtt subscription. There must be a smarter way of doing this
    }

    if(currentJson.containsKey("pulseFeedbackEnabled")){
      g_mqttConfig.pulseFeedbackEnabled=currentJson["pulseFeedbackEnabled"];
    }

    if(currentJson.containsKey("encoderFeedbackEnabled")){
      g_mqttConfig.encoderFeedbackEnabled=currentJson["encoderFeedbackEnabled"];
    }

    if(currentJson.containsKey("setPointFeedbackEnabled")){
      g_mqttConfig.setpointFeedbackEnabled=currentJson["setPointFeedbackEnabled"];
    }

    if(currentJson.containsKey("errorFeedbackEnabled")){
      g_mqttConfig.timeStampFeedbackEnabled=currentJson["errorFeedbackEnabled"];
    }

    if(currentJson.containsKey("timeStampFeedbackEnabled")){
      g_mqttConfig.errorFeedbackEnabled=currentJson["timeStampFeedbackEnabled"];
    }

    if(currentJson.containsKey("publishPeriod")){
      g_mqttConfig.feedbackPublishPeriod=currentJson["publishPeriod"];
    }

    if(currentJson.containsKey("username")){
      String currentUserName=currentJson["username"];
      if(currentUserName.length()<=MQTT_CRED_MAXSIZE){
        sprintf(g_mqttConfig.userName,currentUserName.c_str());
        uint8_t usernameBytes[MQTT_CRED_MAXSIZE+1];
        for(uint8_t i=0;i<MQTT_CRED_MAXSIZE+1;i++){
          usernameBytes[i]=g_mqttConfig.userName[i];
        }
        g_eeprom.writeBlock(200,usernameBytes,MQTT_CRED_MAXSIZE+1);
      }
    }

    writeConfigToEeprom();
    g_server.send(200,F("text/html"), "OK");
  }else if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to Mqtt Configuration");
    String payload="{";
    payload+="\"brokerIp\":\""+String(g_mqttConfig.brokerIp[0])+"."
                              +String(g_mqttConfig.brokerIp[1])+"."
                              +String(g_mqttConfig.brokerIp[2])+"."
                              +String(g_mqttConfig.brokerIp[3])+"\",";
    payload+="\"brokerPort\":"+String(g_mqttConfig.brokerPort)+String(",");

    payload+="\"mqttEnabled\":";
    if(g_mqttConfig.enabled) payload+="true,";
    else payload+="false,";

    payload+="\"boardNumber\":"+String(g_boardNumber)+",";

    payload+="\"pulseFeedbackEnabled\":";
    if(g_mqttConfig.pulseFeedbackEnabled) payload+="true,";
    else payload+="false,";

    payload+="\"encoderFeedbackEnabled\":";
    if(g_mqttConfig.encoderFeedbackEnabled) payload+="true,";
    else payload+="false,";

    payload+="\"setPointFeedbackEnabled\":";
    if(g_mqttConfig.setpointFeedbackEnabled) payload+="true,";
    else payload+="false,";

    payload+="\"timeStampFeedbackEnabled\":";
    if(g_mqttConfig.timeStampFeedbackEnabled) payload+="true,";
    else payload+="false,";

    payload+="\"errorFeedbackEnabled\":";
    if(g_mqttConfig.errorFeedbackEnabled) payload+="true,";
    else payload+="false,";

    payload+="\"publishPeriod\":"+String(g_mqttConfig.feedbackPublishPeriod)+",";
    payload+="\"username\":\""+String(g_mqttConfig.userName)+"\"";
    payload+="}";

    g_server.send(200, F("text/html"),payload);
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}

void httpQueueCallback(){
  if(g_server.method()==HTTP_GET){
    Serial.println("Incoming GET to queue");
    uint8_t nTrajSlotsAvailable=g_nTrajSlotsAvailableOnSlave+TRAJ_BUFFER_SIZE-g_trajEmptySlotIndex;
    uint8_t nTrajSlotsOccupied=g_trajEmptySlotIndex+2-g_nTrajSlotsAvailableOnSlave;
    String payload="{";
    payload+="\"currentQueueId\":"+String(g_curentActiveTraj.ui16)+',';
    payload+="\"queueSlotsOccupied\":"+String(nTrajSlotsOccupied)+",";
    payload+="\"queueSlotsAvailable\":"+String(nTrajSlotsAvailable);
    payload+="}";
    g_server.send(200, F("text/html"),payload);
  }else{
    g_server.send(405, F("text/html"),"Method Not Allowed");
  }
}

void httpRebootCallback(){
  g_server.send(200, F("text/html"),"OK, rebooting");
  ESP.restart();
}
// ************************************************************************************
//                           MQTT callback function
// ************************************************************************************
void mqttCallback(char* topic, byte* payload, unsigned int length){
  Serial.print("Incoming publication to mqtt topic : ");
  Serial.println(topic);

  if(strcmp(topic,g_mqttTopics.motorOn)==0 || strcmp(topic,g_mqttTopics.motorsOn)==0){
    i2cMotorOn();
  }
  if(strcmp(topic,g_mqttTopics.motorOff)==0 || strcmp(topic,g_mqttTopics.motorsOff)==0){
    i2cMotorOff();
  }
  if(strcmp(topic,g_mqttTopics.motorGo)==0 || strcmp(topic,g_mqttTopics.motorsGo)==0){
    i2cGo();
  }
  if(strcmp(topic,g_mqttTopics.motorStop)==0 || strcmp(topic,g_mqttTopics.motorsStop)==0){
    i2cStop();
  }
  if(strcmp(topic,g_mqttTopics.motorAbort)==0 || strcmp(topic,g_mqttTopics.motorsAbort)==0){
    i2cAbort();
  }
  if(strcmp(topic,g_mqttTopics.motorsEnableCL)==0 || strcmp(topic,g_mqttTopics.motorEnableCL)==0){
    g_closedLoopEnabled=true;
    i2cSetSlaveMode(CLOSEDLOOP_MODE);
    i2cGetFullData();
    processErrorCodes();
  }
  if(strcmp(topic,g_mqttTopics.motorsDisableCL)==0 || strcmp(topic,g_mqttTopics.motorDisableCL)==0){
    g_closedLoopEnabled=false;
    i2cSetSlaveMode(OPENLOOP_MODE);
    i2cGetFullData();
    processErrorCodes();
  }
}

void publishMqttTopics(){
  static uint32_t lastPublicationDate;
  static uint16_t lastCurrentQueueIdPublished=59244; // just a random initialization that is different than 0
  
  String payload;

  if(g_curentActiveTraj.ui16!=lastCurrentQueueIdPublished){
    uint8_t nTrajSlotsAvailable=g_nTrajSlotsAvailableOnSlave+TRAJ_BUFFER_SIZE-g_trajEmptySlotIndex;
    uint8_t nTrajSlotsOccupied=g_trajEmptySlotIndex+2-g_nTrajSlotsAvailableOnSlave;
    String payload="{";
    payload+="\"currentQueueId\":"+String(g_curentActiveTraj.ui16)+',';
    payload+="\"queueSlotsOccupied\":"+String(nTrajSlotsOccupied)+",";
    payload+="\"queueSlotsAvailable\":"+String(nTrajSlotsAvailable);
    payload+="}";
    g_mqttClient.publish(g_mqttTopics.queue,payload.c_str());
    lastCurrentQueueIdPublished=g_curentActiveTraj.ui16;
  }

  if(!g_mqttConfig.enabled || millis()-lastPublicationDate<g_mqttConfig.feedbackPublishPeriod*1000){
    return;
  }
  // publishing motion feedback
  
  payload="{";

  if(g_mqttConfig.pulseFeedbackEnabled){
    payload+="\"pulses\":"+String(g_currentMotorPulses.i32+g_valuesOffset.pulses);
    if(g_mqttConfig.encoderFeedbackEnabled || g_mqttConfig.setpointFeedbackEnabled || g_mqttConfig.timeStampFeedbackEnabled){
      payload+=",";
    }
  }

  if(g_mqttConfig.encoderFeedbackEnabled){
    payload+="\"encoder\":"+String(g_currentEncoderPosition.f+g_valuesOffset.encoder,3);
    if(g_mqttConfig.setpointFeedbackEnabled || g_mqttConfig.timeStampFeedbackEnabled){
      payload+=",";
    }
  }

  if(g_mqttConfig.timeStampFeedbackEnabled){
    payload+="\"timeStamp\":"+String((millis()+g_valuesOffset.time_ms)/1000.,3);
    if(g_mqttConfig.setpointFeedbackEnabled){
      payload+=",";
    }
  }

  if(g_mqttConfig.setpointFeedbackEnabled){
    if(g_closedLoopEnabled){
      payload+="\"setPoint\":"+String(g_currentSetPoint.i32+g_valuesOffset.encoder);
    }else{
      payload+="\"setPoint\":"+String(g_currentSetPoint.i32+g_valuesOffset.pulses);
    }
  }

  payload+="}";

  g_mqttClient.publish(g_mqttTopics.feedback,payload.c_str());
  lastPublicationDate=millis();

  


}

// ************************************************************************************
//                           Setters from JSON functions
// ************************************************************************************
bool setNetworkConfigFromJson(String inputJsonString){
  JsonDocument currentJson;
  DeserializationError jsonErr=deserializeJson(currentJson, inputJsonString);
  if(jsonErr){
    Serial.print("JSON error : ");
    Serial.println(jsonErr.c_str());
    return false;
  }
    
  // Variables useful for the following cases : 
  int currentDotPosition;
  int currentStart;

  if(currentJson.containsKey("ipAddress")){
    String ipReceived=currentJson["ipAddress"];
    if(!stringIpToByte(ipReceived,g_networkConfig.ip)){
      return false;
    }
  }
  if(currentJson.containsKey("gatewayIp")){
    String gatewayReceived=currentJson["gatewayIp"];
    uint8_t temp[4];
	  if(!stringIpToByte(gatewayReceived,g_networkConfig.gatewayIp)){
      return false;
    }
  }
  if(currentJson.containsKey("networkMasks")){
    String masksReceived=currentJson["networkMasks"];
	  if(!stringIpToByte(masksReceived,g_networkConfig.masks)){
      return false;
    }
  }
  return true;
}

bool setControlParamsFromJson(String inputJsonString){
  JsonDocument currentJson;
  DeserializationError jsonErr=deserializeJson(currentJson, inputJsonString);
  if(jsonErr){
    Serial.print("JSON error : ");
    Serial.println(jsonErr.c_str());
    return false;
  }

  bool slaveConfigMustChange=false;
  if(currentJson.containsKey("Kp")){
     g_Kp.f=(float)currentJson["Kp"];
     slaveConfigMustChange=true;
  }
  if(currentJson.containsKey("Ki")){
     g_Ki.f=(float)currentJson["Ki"];
     slaveConfigMustChange=true;
  }
  if(currentJson.containsKey("Kd")){
     g_Kd.f=(float)currentJson["Kd"];
     slaveConfigMustChange=true;
  }
  if(currentJson.containsKey("maxVelocity")){
     g_maxVelocity.f=(float)currentJson["maxVelocity"];
     slaveConfigMustChange=true;
  }
  if(currentJson.containsKey("errorLimit")){
     g_errorLimit.ui16=(uint16_t)currentJson["errorLimit"];
     slaveConfigMustChange=true;
  }
  if(slaveConfigMustChange){
    i2cSetControlCoef(g_Kp.f,g_Ki.f,g_Kd.f,g_maxVelocity.f,g_errorLimit.ui16);
    i2cGetFullData();
	  processErrorCodes();
  }

  writeConfigToEeprom();
  return true;
}

bool setHwConfigFromJson(String inputJsonString){
  JsonDocument currentJson;
  DeserializationError jsonErr=deserializeJson(currentJson, inputJsonString);
  if(jsonErr){
    Serial.print("JSON error : ");
    Serial.println(jsonErr.c_str());
    return false;
  }
  
  bool slaveConfigMustChange=false;
  if(currentJson.containsKey("motorStepsPerRev")){
    int32_t rawMotorStepsPerRev=(int32_t)currentJson["motorStepsPerRev"];
    if(rawMotorStepsPerRev<0){
      g_motorStepsPerRev.i32=-rawMotorStepsPerRev;
      g_invertedMotor=true;
    }else{
      g_motorStepsPerRev.i32=rawMotorStepsPerRev;
      g_invertedMotor=false;
    }
    slaveConfigMustChange=true;
  }

  if(currentJson.containsKey("encoderTicksPerRev")){
    int32_t rawEncoderTicksPerRev=(int32_t)currentJson["encoderTicksPerRev"];
    if(rawEncoderTicksPerRev<0){
      g_encoderStepsPerRev.i32=-rawEncoderTicksPerRev;
      g_invertedEncoder=true;
    }else{
      g_encoderStepsPerRev.i32=rawEncoderTicksPerRev;
      g_invertedEncoder=false;
    }
    slaveConfigMustChange=true;
  }
  if(slaveConfigMustChange){
    // Setting the hardware config can mess things up on the slave, so a careful reset of the slave must be done
    // First, set to open loop control before changing the points :
    i2cSetSlaveMode(OPENLOOP_MODE);
    // Then, set the motor position, encoder position and setPoint, to 0 : 
    // Only then upload the configuration : 
    i2cSetHardwareConfig(g_motorStepsPerRev.i32,g_encoderStepsPerRev.i32,g_invertedMotor,g_invertedEncoder);
    i2cSetControlCoef(g_Kp.f,g_Ki.f,g_Kd.f,g_maxVelocity.f,g_errorLimit.ui16);
    i2cSetSlaveMotorPosition(0);
    i2cSetSlaveEncoderPosition(0);
    if(g_closedLoopEnabled){
      i2cSetSlaveMode(CLOSEDLOOP_MODE);
    }
    
    i2cGetFullData();
	  processErrorCodes();
  }
  /*
  if(currentJson.containsKey("controlLoop")){
    String rawEncoderTicksPerRev=currentJson["controlLoop"];
    if(rawEncoderTicksPerRev=="CLOSED"){
      g_closedLoopEnabled=true;
      i2cSetSlaveMode(CLOSEDLOOP_MODE);
    }else if(rawEncoderTicksPerRev=="OPEN"){
      g_closedLoopEnabled=false;
      i2cSetSlaveMode(OPENLOOP_MODE);
    }else{
      return false;
    }
    i2cGetFullData();
	  processErrorCodes();
  }
  */
  return true;
}
// ************************************************************************************
//                           EEPROM related functions
// ************************************************************************************
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

  Serial.println("Config written to EEPROM");
}

void eepromUpdateByte(uint16_t address,uint8_t value){
  if(g_eepromImage[address]!=value){
    g_eeprom.updateByte(address,value);
    g_eepromImage[address]=value;
  }
}

void readConfigFromEeprom(){

  // doing a quick check that all data is not 255, which would mean this is a brand new eeprom chip, and we don't want to get our config from it
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

  Serial.println("config read from EEPROM");

}

void setDefaultConfiguration(){

  g_networkConfig.ip[0]=192;
  g_networkConfig.ip[1]=168;
  g_networkConfig.ip[2]=1;
  g_networkConfig.ip[3]=12;

  g_networkConfig.gatewayIp[0]=192;
  g_networkConfig.gatewayIp[1]=168;
  g_networkConfig.gatewayIp[2]=1;
  g_networkConfig.gatewayIp[3]=1;

  g_networkConfig.masks[0]=255;
  g_networkConfig.masks[1]=255;
  g_networkConfig.masks[2]=255;
  g_networkConfig.masks[3]=0;

  g_networkConfig.errorFeedbackIp[0]=192;
  g_networkConfig.errorFeedbackIp[1]=168;
  g_networkConfig.errorFeedbackIp[2]=1;
  g_networkConfig.errorFeedbackIp[3]=14;

  g_boardNumber=1;

  g_Kp.f=10;
  g_Ki.f=0;
  g_Kd.f=0;

  g_maxVelocity.f=4000;
  g_errorLimit.ui16=0;

  g_motorStepsPerRev.i32=400;
  g_encoderStepsPerRev.i32=400;

  g_invertedMotor=false;
  g_invertedEncoder=false;
  g_closedLoopEnabled=false;

  g_mqttConfig.brokerIp[0]=192;
  g_mqttConfig.brokerIp[1]=168;
  g_mqttConfig.brokerIp[2]=1;
  g_mqttConfig.brokerIp[3]=14;
  g_mqttConfig.brokerPort=1883;
  g_mqttConfig.enabled=false;
  g_mqttConfig.pulseFeedbackEnabled=true;
  g_mqttConfig.encoderFeedbackEnabled=true;
  g_mqttConfig.setpointFeedbackEnabled=true;
  g_mqttConfig.timeStampFeedbackEnabled=true;
  g_mqttConfig.feedbackPublishPeriod=0.2;
  sprintf(g_mqttConfig.userName,"");
  sprintf(g_mqttConfig.password,"");

  g_errorFeedbackSettings.httpErrorEnabled=false;
  g_errorFeedbackSettings.httpWarningEnabled=false;
  g_errorFeedbackSettings.mqttErrorEnabled=false;
  g_errorFeedbackSettings.mqttWarningEnabled=false;

  i2cSetHardwareConfig(g_motorStepsPerRev.i32,g_encoderStepsPerRev.i32,g_invertedMotor,g_invertedEncoder);
  i2cSetControlCoef(g_Kp.f,g_Ki.f,g_Kd.f,g_maxVelocity.f,g_errorLimit.ui16);
  i2cSetSlaveMode(OPENLOOP_MODE);

  writeConfigToEeprom();

  // writing mqtt username and password to EEPROM : 
  uint8_t usernameBytes[MQTT_CRED_MAXSIZE+1];
  uint8_t passwordBytes[MQTT_CRED_MAXSIZE+1];
  for(uint8_t i=0;i<MQTT_CRED_MAXSIZE+1;i++){
    usernameBytes[i]=g_mqttConfig.userName[i];
    passwordBytes[i]=g_mqttConfig.password[i];
  }
  g_eeprom.writeBlock(200,usernameBytes,MQTT_CRED_MAXSIZE+1);
  g_eeprom.writeBlock(300,passwordBytes,MQTT_CRED_MAXSIZE+1);


  Serial.println("Default config set");

}


// ************************************************************************************
//                           Miscellanous functions
// ************************************************************************************

void printData(uint8_t* data, uint8_t size){
  Serial.print("Data : ");
  Serial.print(data[0]);
  if(size>1)
    for(uint8_t i=1;i<size;i++){
      Serial.print(",");
      Serial.print(data[i]);
    }
  Serial.println("");
}

bool stringIpToByte(String inputString,uint8_t outputByte[]){
  int32_t currentDotPosition=inputString.indexOf('.');
  int32_t currentStart=0;
  int32_t currentByte;

  uint8_t tempBytes[4]; // We use a buffer to avoid changing the output if something is wrong
  for(uint8_t i=0;i<3;i++){
    if(currentDotPosition==-1){
      return false;
    }
    currentByte=(inputString.substring(currentStart,currentDotPosition)).toInt();
    if(currentByte<0 || currentByte>255){
      return false;
    }else{
      tempBytes[i]=currentByte;
      currentStart=currentDotPosition+1;
      currentDotPosition=inputString.indexOf('.',currentStart);
    }
  }
  currentByte=(inputString.substring(currentStart,inputString.length())).toInt();
  if(currentByte<0 || currentByte>255){
    return false;
  }
  tempBytes[3]=currentByte;

  outputByte[0]=tempBytes[0];
  outputByte[1]=tempBytes[1];
  outputByte[2]=tempBytes[2];
  outputByte[3]=tempBytes[3];

  return true;
}

void manageButtonPress(){
  static uint32_t lastButtonPressDate;
  if(digitalRead(PIN_RESET_BUTTON)==LOW){
    if(millis()-lastButtonPressDate>3000){
      analogWrite(PIN_LEDG,0);
      analogWrite(PIN_LEDR,0);
      analogWrite(PIN_LEDB,LEDBRIGHTNESS); delay(200);
      analogWrite(PIN_LEDB,0); delay(200);
      analogWrite(PIN_LEDB,LEDBRIGHTNESS); delay(200);
      analogWrite(PIN_LEDB,0); delay(200);
      analogWrite(PIN_LEDB,LEDBRIGHTNESS); delay(200);
      analogWrite(PIN_LEDB,0); delay(200);
      setDefaultConfiguration();
      ESP.restart();
	  }
  }else{
    lastButtonPressDate=millis();
  }
}

void manageLedColor(){
  
  if(!g_ethClient.connected()){
    analogWrite(PIN_LEDR,LEDBRIGHTNESS);
    analogWrite(PIN_LEDG,LEDBRIGHTNESS);
    analogWrite(PIN_LEDB,0);
    return;
  }
	
  if(g_mqttConfig.enabled && !g_mqttClient.connected()){
    analogWrite(PIN_LEDR,LEDBRIGHTNESS);
    analogWrite(PIN_LEDG,0);
    analogWrite(PIN_LEDB,LEDBRIGHTNESS);
    return;
  }

  analogWrite(PIN_LEDR,0);
  analogWrite(PIN_LEDG,LEDBRIGHTNESS);
  analogWrite(PIN_LEDB,0);
}

// ************************************************************************************
//                           Home page generation
// ************************************************************************************

String generateHomePage(){
  String output="";
  output+="<html>";
  output+=
    #include "htmlHeader.h"
  ;
  output+="<body>\n";
  output+="<div class='container'>\n";
  output+=generateNetworkConfigHtmlForm();
  output+=generateControlParamHtmlForm();
  output+=generateHtmlMqttconfig();
  output+=generateHwConfigHtmlForm();
  output+=generateHtmlPowerButtons();
  output+=generateHtmlclosedLoopButtons();
  output+=generateHtmlBasicPosition();
  output+=generateHtmlBasicVelocity();
  output+=generateHtmlReboot();

  output+="<iframe name='hiddenFrame' style=' display:none;'></iframe>\n";
  
  output+=
    #include "htmlScripts.h"
  ;

  output+="\n</container>\n</body>\n</html>";
  return output;
}

String generateNetworkConfigHtmlForm(){
  String output="";
  output+="    <div class='form-container'>";
  output+="    <h2>NetworkConfiguration (reboot)</h2>";
  output+="    <form method='put' action='/networkConfig' target='hiddenFrame' onsubmit='putNetworkConfig()'>";
  output+="      <div class='ip-input-container'>";
  output+="        <label>IPAddress:</label>";
  output+="        <input type='text' id='ipByte1' size='4' placeholder='"+String(g_networkConfig.ip[0])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='ipByte2' size='4' placeholder='"+String(g_networkConfig.ip[1])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='ipByte3' size='4' placeholder='"+String(g_networkConfig.ip[2])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='ipByte4' size='4' placeholder='"+String(g_networkConfig.ip[3])+"'>";
  output+="      </div>";
  output+="      <div class='ip-input-container'>";
  output+="        <label>GatewayIP:</label>";
  output+="        <input type='text' id='gatewayByte1' size='4' placeholder='"+String(g_networkConfig.gatewayIp[0])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='gatewayByte2' size='4' placeholder='"+String(g_networkConfig.gatewayIp[1])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='gatewayByte3' size='4' placeholder='"+String(g_networkConfig.gatewayIp[2])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='gatewayByte4' size='4' placeholder='"+String(g_networkConfig.gatewayIp[3])+"'>";
  output+="      </div>";
  output+="      <div class='ip-input-container'>";
  output+="        <label>SubnetMasks:</label>";
  output+="        <input type='text' id='masksByte1' size='4' placeholder='"+String(g_networkConfig.masks[0])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='masksByte2' size='4' placeholder='"+String(g_networkConfig.masks[1])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='masksByte3' size='4' placeholder='"+String(g_networkConfig.masks[2])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='masksByte4' size='4' placeholder='"+String(g_networkConfig.masks[3])+"'>";
  output+="      </div>";
  output+="      <input type='submit' value='Update'>";
  output+="    </form>";
  output+="  </div>";
  output+="\n";
  return output;
}

String generateControlParamHtmlForm(){
  String output="";
  output+="    <div class='form-container'>";
  output+="    <h2>Control Parameters</h2>";
  output+="    <form method='put' action='/controlParameters' target='hiddenFrame' onsubmit='putControlParams()'>";
  output+="          <div class='form-group'>";
  output+="            <label>Kp:</label>";
  output+="            <input type='text' id='Kp' placeholder='"+String(g_Kp.f)+"'>";
  output+="          </div>";
  output+="          <div class='form-group'>";
  output+="            <label>Ki:</label>";
  output+="            <input type='text' id='Ki' placeholder='"+String(g_Ki.f)+"'>";
  output+="          </div>";
  output+="          <div class='form-group'>";
  output+="            <label>Kd:</label>";
  output+="            <input type='text' id='Kd' placeholder='"+String(g_Kd.f)+"'>";
  output+="          </div>";
  output+="          <div class='form-group'>";
  output+="            <label>maxVelocity:</label>";
  output+="            <input type='text' id='maxVelocity' placeholder='"+String(g_maxVelocity.f)+"'>";
  output+="          </div>";
  output+="          <div class='form-group'>";
  output+="            <label>errorLimit:</label>";
  output+="            <input type='text' id='errorLimit' placeholder='"+String(g_errorLimit.ui16)+"'>";
  output+="          </div>";
  output+="          <input type='submit' value='Update'>";
  output+="    </form>";
  output+="  </div>";
  output+="\n";
  return output;
}

String generateHwConfigHtmlForm(){
  String output="";
  output+="  <div class='form-container'>";
  output+="    <h2>Hardware Configuration</h2>";
  output+="    <form method='put' action='/hardwareConfig' target='hiddenFrame' onsubmit='putHwConfig()'>";
  output+="          <div class='form-group'>";
  output+="            <label>motorStepsPerRev:</label>";
  output+="            <input type='text' id='motorStepsPerRev' placeholder='";
  if(g_invertedMotor){output+="-";}
  output+=String(g_motorStepsPerRev.i32)+"'>";
  output+="          </div>";
  output+="          <div class='form-group'>";
  output+="            <label>encoderTicksPerRev:</label>";
  output+="            <input type='text' id='encoderTicksPerRev' placeholder='";
  if(g_invertedEncoder){output+="-";}
  output+=String(g_encoderStepsPerRev.i32)+"'>";
  output+="          </div>";
  output+="          <input type='submit' value='Update'>";
  output+="    </form>";
  output+="  </div>";
  output+="\n";
  return output;
}

String generateHtmlPowerButtons(){
  String output="";
  if(g_isMotorOn){
    output+="  <div class='form-container'>";
    output+="    <h2>Motor power : ON</h2>";
    output+="      <form method='put' action='/motor/off' target='hiddenFrame' onsubmit='motorOff()'>";
    output+="      <div class='form-group'>";
    output+="        <input type='submit' value='Turn Off'>";
    output+="      </div>";
    output+="    </form>";
    output+="  </div>";
    output+="\n";
  }else{
    output+="  <div class='form-container'>";
    output+="    <h2>Motor power : OFF</h2>";
    output+="    <form method='put' action='/motor/on' target='hiddenFrame' onsubmit='motorOn()'>";
    output+="      <div class='form-group'>";
    output+="        <input type='submit' value='Turn On'>";
    output+="      </div>";
    output+="    </form>";
    output+="  </div>";
    output+="\n";
  }
  return output;
}

String generateHtmlclosedLoopButtons(){
  String output="";
  if(g_closedLoopEnabled){
    output+="  <div class='form-container'>";
    output+="    <h2>Closed loop : Enabled</h2>";
    output+="    <form method='put' action='/motor/closedLoop/disable ' target='hiddenFrame' onsubmit='disableClosedLoop()'>";
    output+="      <div class='form-group'>";
    output+="        <input type='submit' value='Disable closed loop'>";
    output+="      </div>";
    output+="    </form>";
    output+="  </div>";
    output+="\n";
  }else{
    output+="  <div class='form-container'>";
    output+="    <h2>Closed loop : Disabled</h2>";
    output+="    <form method='put' action='/motor/closedLoop/enable ' target='hiddenFrame' onsubmit='enableClosedLoop()'>";
    output+="      <div class='form-group'>";
    output+="        <input type='submit' value='Enable closed loop'>";
    output+="      </div>";
    output+="    </form>";
    output+="  </div>";
    output+="\n";
  }
  return output;
}

String generateHtmlBasicPosition(){
  String output="";
  output+="    <div class='form-container'>";
  output+="    <h2>Basic Position</h2>";
  output+="    <form method='post' action='/setPoint' target='hiddenFrame' onsubmit='setPoint()'>";
  output+="          <div class='form-group'>";
  output+="            <label>Target</label>";
  output+="            <input type='text' id='setPointTarget'>";
  output+="          <input type='submit' value='Go'>";
  output+="          </div>";
  output+="          <div>";
  output+="            <input type='radio' id='Relative' value='Relative' name='setPointRelativity'checked>";
  output+="            <label for='Relative'>Relative</label>";
  output+="            <input type='radio' id='Absolute' value='Absolute' name='setPointRelativity'>";
  output+="            <label for='Absolute'>Absolute</label>";
  output+="          </div>";
  output+="    </form>";
  output+="  </div>";
  output+="\n";
  return output;
}

String generateHtmlBasicVelocity(){
  String output="";
  output+="    <div class='form-container'>";
  output+="    <h2>Basic Velocity</h2>";
  output+="    <form method='post' action='/jog' target='hiddenFrame' onsubmit='jog()'>";
  output+="          <div class='form-group'>";
  output+="            <label>Velocity</label>";
  output+="            <input type='text' id='setVelocity'>";
  output+="            <input type='submit' value='Go'>";
  output+="          </div>";
  output+="    </form>";
  output+="  </div>";
  output+="\n";
  return output;
}

String generateHtmlMqttconfig(){
  String output="";
  output+="    <div class='form-container'>";
  output+="    <h2>MQTT conrfiguration (reboot)</h2>";
  output+="    <form method='put' action='/mqttConfig' target='hiddenFrame' onsubmit='putMqttConfig()'>";
  output+="      <div class='ip-input-container'>";
  output+="        <label>BrokerIP:</label>";
  output+="        <input type='text' id='brokerIp1' size='4' placeholder='"+String(g_mqttConfig.brokerIp[0])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='brokerIp2' size='4' placeholder='"+String(g_mqttConfig.brokerIp[1])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='brokerIp3' size='4' placeholder='"+String(g_mqttConfig.brokerIp[2])+"'>";
  output+="        <label>.</label>";
  output+="        <input type='text' id='brokerIp4' size='4' placeholder='"+String(g_mqttConfig.brokerIp[3])+"'>";
  output+="      </div>";
  output+="      <div class='form-group'>";
  output+="        <label>BrokerPort:</label>";
  output+="        <input type='text' id='brokerPort' placeholder='"+String(g_mqttConfig.brokerPort)+"'>";
  output+="      </div>";
  output+="       <div>";
  output+="         <label>MQTT:</label>";
  output+="         <input type='radio' id='mqttEnabled' value='Enabled' name='mqttEnabledRadio'";
                    if(g_mqttConfig.enabled){
                       output+="checked";
                    }
                    output+=">"; // The indentation is weird, but it kind of makes sense to me
  output+="         <label for='mqttEnabled'>Enabled</label>";
  output+="         <input type='radio' id='mqttDisabled' value='Disabled' name='mqttEnabledRadio'";
                    if(!g_mqttConfig.enabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttDisabled'>Disabled</label>";
  output+="       </div>";
  output+="       <div class='form-group'>";
  output+="         <label>BoardNumber:</label>";
  output+="         <input type='text' id='boardNumber' placeholder='"+String(g_boardNumber)+"'>";
  output+="       </div>";

  output+="       <div>";
  output+="         <label>PulseFeedback:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</label>";
  output+="         <input type='radio' id='mqttPulseFbEnabled' value='Enabled' name='mqttPulseFbRadio'";
                    if(g_mqttConfig.pulseFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttPulseFbEnabled'>Enabled</label>";
  output+="         <input type='radio' id='mqttPulseFbDisabled' value='Disabled' name='mqttPulseFbRadio'";
                    if(!g_mqttConfig.pulseFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttPulseFbDisabled'>Disabled</label>";
  output+="       </div>";

  output+="       <div>";
  output+="         <label>EncoderFeedback:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</label>";
  output+="         <input type='radio' id='mqttEncoderFbEnabled' value='Enabled' name='mqttEncoderFbRadio'";
                    if(g_mqttConfig.encoderFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttEncoderFbEnabled'>Enabled</label>";
  output+="         <input type='radio' id='mqttEncoderFbDisabled' value='Disabled' name='mqttEncoderFbRadio'";
                    if(!g_mqttConfig.encoderFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttEncoderFbDisabled'>Disabled</label>";
  output+="       </div>";

  output+="       <div>";
  output+="         <label>SetpointFeedback:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</label>";
  output+="         <input type='radio' id='mqttSetpointFbEnabled' value='Enabled' name='mqttSetpointFbRadio'";
                    if(g_mqttConfig.setpointFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttSetpointFbEnabled'>Enabled</label>";
  output+="         <input type='radio' id='mqttSetpointFbDisabled' value='Disabled' name='mqttSetpointFbRadio'";
                    if(!g_mqttConfig.setpointFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttSetpointFbDisabled'>Disabled</label>";
  output+="       </div>";

  output+="       <div>";
  output+="         <label>TimestampFeedback:&nbsp;</label>";
  output+="         <input type='radio' id='mqttTimestampFbEnabled' value='Enabled' name='mqttTimestampFbRadio'";
                    if(g_mqttConfig.timeStampFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttTimestampFbEnabled'>Enabled</label>";
  output+="         <input type='radio' id='mqttTimestampFbDisabled' value='Disabled' name='mqttTimestampFbRadio'";
                    if(!g_mqttConfig.timeStampFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttTimestampFbDisabled'>Disabled</label>";
  output+="       </div>";

  output+="       <div>";
  output+="         <label>ErrorFeedback:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</label>";
  output+="         <input type='radio' id='mqttErrorFbEnabled' value='Enabled' name='mqttErrorFbRadio'";
                    if(g_mqttConfig.errorFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttErrorFbEnabled'>Enabled</label>";
  output+="         <input type='radio' id='mqttErrorFbDisabled' value='Disabled' name='mqttErrorFbRadio'";
                    if(!g_mqttConfig.errorFeedbackEnabled){
                       output+="checked";
                    }
                    output+=">";
  output+="         <label for='mqttErrorFbDisabled'>Disabled</label>";
  output+="       </div>";
  
  output+="       <div class='form-group'>";
  output+="         <label>PublishPeriod(s):</label>";
  output+="         <input type='text' id='publishPeriod' placeholder='"+String(g_mqttConfig.feedbackPublishPeriod)+"'>";
  output+="       </div>";

  output+="       <div class='form-group'>";
  output+="         <label>Username:</label>";
  output+="         <input type='text' id='mqttUsername' placeholder='"+String(g_mqttConfig.userName)+"'>";
  output+="       </div>";

  output+="       <div class='form-group'>";
  output+="         <label>Password:</label>";
  output+="         <input type='password' id='mqttPassword'>";
  output+="       </div>";
  output+="       <input type='submit' value='Update'>";
  output+="    </form>";
  output+="  </div>";
  output+="\n";
  return output;
}

String generateHtmlReboot(){
  String output="";
  output+="  <div class='form-container'>";
  output+="    <form method='put' action='/reboot' target='hiddenFrame' onsubmit='reboot()'>";
  output+="      <div class='form-group'>";
  output+="        <input type='submit' value='Reboot'>";
  output+="      </div>";
  output+="    </form>";
  output+="  </div>";
  output+="\n";
  return output;
}