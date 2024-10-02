#ifndef DEFINES_H
#define DEFINES_H

// Set to true if you want verbose serial output
#define SERIAL_PRINTS false

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
#define MAXMAXVELOCITY 500000

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

#endif