#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "ustdlib.h"
#include "uartstdio.h"

/// Board includes
#include "lm3s2965.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "rit128x96x4.h"

/// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/// Boolean type
typedef char bool;
#define true 1
#define false 0

#define PIN_ON 0xFF
#define PIN_OFF 0x00

#define STRAT_1 1
#define STRAT_2 0

#define WDOG_LIMIT 100000
#define INTEL_WDOG_LIMIT 100

// Generic data structures 
/*********************/
/*      ODOMETRY     */
/*********************/
/**
 * Holds data related to a fetch of the encoders
 */
typedef struct Encoder_t {
   int tickvalue; 	     // position at the last update
   portTickType time;    // time of the last update
   bool forward;         // true if the encoder goes forward
   unsigned long ulBase; // QEI peripheral base address
} Encoder;

/*******************/
/*   ROBOT STATE   */
/*******************/
/**
 * Defines the state of a robot. x, y are the absolute
 * coordinates, phi is the relative angle, and stop is
 * set to true if the robot is idle.
 */
typedef struct State_t {
   float x; 		// position of the robot
   float y;         // position of the robot
   float phi;       // angle of the robot
   bool stop;       // true if the robot must stop
} State;

/********************/
/*    OBJECTIVES    */
/********************/
/**
 * holds a destination of the robot
 */
typedef struct PositionGoal_t
{
	float x; 		// final position
	float y;		// final position
	float phi;		// final angle
	float k;        // curvature

	bool stop;      // Have to stop at this point
} PositionGoal;

typedef struct Coord_t {
   float x;
   float y;
} Coord;

/// Own includes
#include "IO_parameters.h"
#include "custom_math.h"
#include "tools_lib.h"
#include "cmdline.h"
#include "captors.h"
#include "intel.h"
#include "control.h"
#include "gametable.h"
#include "odometry.h"
#include "world.h"

void errorReport(char* msg);
void batteryReport(unsigned long bVolt);

#endif
