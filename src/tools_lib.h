#ifndef TOOLS_LIB_H_INCLUDED
#define TOOLS_LIB_H_INCLUDED

#include "definitions.h"

#define SERVO_BUFSIZ 128
#define SERVO_BROADCAST 0xFE

#define SERVO_RECEIVED_OK 0
#define SERVO_HEAD_ERROR 1
#define SERVO_MULTIPLE_BROADCAST 2
#define SERVO_WRONG_SENDER 3
#define SERVO_BAD_CHKSUM 4
#define SERVO_NOT_RESP 5

//--- Instruction ---
#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET          0x06
#define INST_DIGITAL_RESET  0x07
#define INST_SYSTEM_READ    0x0C
#define INST_SYSTEM_WRITE   0x0D
#define INST_SYNC_WRITE     0x83
#define INST_SYNC_REG_WRITE 0x84

// Paramètres des flaps
#define FLAP_MAX_VAL_HIGH   0x03
#define FLAP_MAX_VAL_LOW    0xFF

#define FLAP_RIGHT_UP_HIGH  0x01
#define FLAP_RIGHT_UP_LOW   0x7C
#define FLAP_RIGHT_DOWN_HIGH 0x03
#define FLAP_RIGHT_DOWN_LOW 0x28
#define FLAP_RIGHT_BALL_HIGH 0x02
#define FLAP_RIGHT_BALL_LOW 0x60
#define FLAP_RIGHT_SECURE_HIGH 0x02
#define FLAP_RIGHT_SECURE_LOW 0xD0

#define FLAP_LEFT_UP_HIGH   FLAP_MAX_VAL_HIGH - FLAP_RIGHT_UP_HIGH
#define FLAP_LEFT_UP_LOW    FLAP_MAX_VAL_LOW  - FLAP_RIGHT_UP_LOW
#define FLAP_LEFT_DOWN_HIGH FLAP_MAX_VAL_HIGH - FLAP_RIGHT_DOWN_HIGH
#define FLAP_LEFT_DOWN_LOW  FLAP_MAX_VAL_LOW  - FLAP_RIGHT_DOWN_LOW
#define FLAP_LEFT_BALL_HIGH FLAP_MAX_VAL_HIGH - FLAP_RIGHT_BALL_HIGH
#define FLAP_LEFT_BALL_LOW  FLAP_MAX_VAL_LOW  - FLAP_RIGHT_BALL_LOW
#define FLAP_LEFT_SECURE_HIGH FLAP_MAX_VAL_HIGH - FLAP_RIGHT_SECURE_HIGH
#define FLAP_LEFT_SECURE_LOW  FLAP_MAX_VAL_LOW  - FLAP_RIGHT_SECURE_LOW

/// Servo library
// Protocol functions
void servoCmdRAW(char ID, char instruction, char paramLength,
                  unsigned long base, unsigned long ctrl_pin_base, unsigned long ctrl_pin_nb,
                  char* servoParam, char* servoBufferTx);

/*
 * Send instruction to a servo
 * @param ID id of the servo
 * @param instruction instruction code
 * @param paramLength nb_parameters
 * @param servoParam array containing the parameters of the instruction
 */
void servoCmdParam(char ID, char instruction, char paramLength, char* servoParam);

char servoListenRAW(portTickType* xLastWakeTime, unsigned long base, char* bufferTx, char* bufferRx, unsigned long* rx_ms_wait);

void servoCmd(char ID, char instruction, char paramLength);
void flapCmd(char ID, char instruction, char paramLength, portTickType* xLastWakeTime);
void flapCmdUnchecked(char ID, char instruction, char paramLength);

bool servoCheck(portTickType* xLastWakeTime);
bool flapCheck(portTickType* xLastWakeTime);

char servoListen(portTickType* xLastWakeTime);
char flapListen(portTickType* xLastWakeTime);

char servoRcvStatusOK();
char flapRcvStatusOK();

void servoRxBufferClrRAW(unsigned long base);

// Manual controls
char servoForward(portTickType* xLastWakeTime, char ID, char upval, char downval);
char servoBackward(portTickType* xLastWakeTime, char ID, char upval, char downval);
char servoForwardFULL(portTickType* xLastWakeTime, char ID);
char servoBackwardFULL(portTickType* xLastWakeTime, char ID);

void servoFreeWheel();
void servoSTOP();
void flapSTOP();

void flapLeftConfig(portTickType* xLastWakeTime);
void flapLeftDown(portTickType* xLastWakeTime);
void flapLeftUp(portTickType* xLastWakeTime);
void flapLeftBall(portTickType* xLastWakeTime);
void flapLeftSecure(portTickType* xLastWakeTime);
void flapRightConfig(portTickType* xLastWakeTime);
void flapRightDown(portTickType* xLastWakeTime);
void flapRightUp(portTickType* xLastWakeTime);
void flapRightBall(portTickType* xLastWakeTime);

// Automatic control
void servoLeft(portTickType* xLastWakeTime, char upval, char downval);
void servoRight(portTickType* xLastWakeTime, char upval, char downval);
void servoSync();



/// Captors
#define ULTRAS_VCC 5
float ultrason_convert(unsigned long value);
float sharp_convert(unsigned long value);

#endif // TOOLS_LIB_H_INCLUDED
