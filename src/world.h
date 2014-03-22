#ifndef WORLD_H_DEFINED
#define WORLD_H_DEFINED

#include "definitions.h"

/** Encoder measures identifiers */
#define ODO_PREV_ENCODER_LEFT  0x01
#define ODO_PREV_ENCODER_RIGHT 0x02
#define ODO_CURR_ENCODER_LEFT  0x03
#define ODO_CURR_ENCODER_RIGHT 0x04

/** */

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
typedef PositionGoal_t
{
	float x; 		// final position
	float y;		// final position
	float phi;		// final angle
	float k;        // curvature
} PositionGoal;


/** Initialize the world */
void init_world();

#endif