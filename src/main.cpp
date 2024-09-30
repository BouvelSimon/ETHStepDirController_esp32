// From brilliant people :
#include <Arduino.h>
#include <Wire.h>
#include <WebServer_WT32_ETH01.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <I2C_eeprom.h>
#include <analogWrite.h>

// From me : 
#include <defines.h>
#include <types.h>
#include <i2cFunctions.h>
#include <trajectory.h>

struct valuesOffset g_valuesOffset;
struct errorFeedbackSettings g_errorFeedbackSettings;
struct mqttTopics g_mqttTopics;
struct mqttConfiguration g_mqttConfig;
struct networkConfiguration g_networkConfig;

// Network initialization functions :
void initializeHttpServer();
void initializeMqttConnection();

// default config & button press: 
void setDefaultConfiguration();
void manageButtonPress();

// http server callbacks. Did not manage to put them in an .h file with definitions in a .cpp file, I'm not sure why ("first defined here" error)
void httpRootCallback();
void httpScopeCallback();
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


// Miscellanous functions declaration :
void printData(uint8_t*, uint8_t);
bool stringIpToByte(String,uint8_t[]);
void manageLedColor();
void processErrorCodes();

// html related functions : 
String generateHomePage();
String generateScopePage();

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

  // http server setup : 
  initializeHttpServer();

  // Real time os tasks
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
//                           Trajectory related functions
// ************************************************************************************


// ************************************************************************************
//                           network related functions
// ************************************************************************************
void initializeHttpServer()
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

  // CORS settings
  g_server.enableCORS();
  
  // Starting the web server : 
  g_server.on(F("/"), httpRootCallback);
  g_server.on(F("/scope"),httpScopeCallback);
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

void httpScopeCallback(){
  Serial.println("Received scope request");
  String currentpage=generateScopePage();

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
     if(g_maxVelocity.f<0){
      g_maxVelocity.f=0;
     }
     if(g_maxVelocity.f<0){
      g_maxVelocity.f=0;
     }
     if(g_maxVelocity.f>MAXMAXVELOCITY){
      g_maxVelocity.f=MAXMAXVELOCITY;
     }
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
  return true;
}

// ************************************************************************************
//                           Miscellanous functions
// ************************************************************************************

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
  
  if(!WT32_ETH01_eth_connected){
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
  output+=
    #include "homePage.h"
  ;

  String currentIp=String(g_networkConfig.ip[0])+'.';
  currentIp+=String(g_networkConfig.ip[1])+'.';
  currentIp+=String(g_networkConfig.ip[2])+'.';
  currentIp+=String(g_networkConfig.ip[3]);

  output.replace("{{IP_ADDRESS}}",currentIp);

  return output;
}

String generateScopePage(){
  String output="";
  output+=
    #include "scopePage.h"
  ;

  String currentIp=String(g_networkConfig.ip[0])+'.';
  currentIp+=String(g_networkConfig.ip[1])+'.';
  currentIp+=String(g_networkConfig.ip[2])+'.';
  currentIp+=String(g_networkConfig.ip[3]);

  output.replace("{{IP_ADDRESS}}",currentIp);

  return output;
}
