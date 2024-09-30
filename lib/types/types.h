#ifndef TYPES_H
#define TYPES_H

#include <defines.h>

struct networkConfiguration{
  uint8_t ip[4];
  uint8_t gatewayIp[4];
  uint8_t masks[4];
  uint8_t errorFeedbackIp[4];
};

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

struct errorFeedbackSettings{
  bool httpErrorEnabled;
  bool httpWarningEnabled;
  bool mqttErrorEnabled;
  bool mqttWarningEnabled;
};


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


#endif