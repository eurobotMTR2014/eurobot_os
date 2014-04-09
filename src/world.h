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
typedef struct PositionGoal_t
{
	float x; 		// final position
	float y;		// final position
	float phi;		// final angle
	float k;        // curvature
} PositionGoal;

typedef struct Coord_t {
   float x;
   float y;
} Coord;

/** Initialize the world */
void init_world();

/**
 * @fn world_peek_next_goal()
 * Returns a pointer to the first element of the goal buffer without removing it
 * @return a pointer to the goal
 * @note The caller is blocked till a value is available in the buffer
 */
PositionGoal world_peek_next_goal();

/**
 * @fn world_peek_next_goal()
 * Returns a pointer to the first element of the goal buffer and removes it
 * @return a pointer to the goal
 * @note The caller is blocked till a value is available in the buffer
 */
PositionGoal world_pick_next_goal();

/**
 * @fn world_put goal()
 * Put a position goal in the goal buffer
 * @param a pointer to a PositionGoal structure
 * @note The caller is blocked till an empty slot is available in the buffer
 */
void world_put_goal(PositionGoal pg);

/**
 * @fn world_goal_flush()
 * Flush the goal buffer
 */
void world_goal_flush();

/**
 * @fn world_goal_remove_peek()
 * Remove the first goal stored in the buffer
 */
void world_goal_remove_peek();

/**
 * @fn world_get_coord()
 * Returns the coordinates of the robot
 * @return a Coord struct
 */
Coord world_get_coord();

/**
 * @fn world_get_state()
 * Returns the state of the robot
 * @return a State struct
 */
State world_get_state();

/**
 * @fn world_update_encoder()
 * Updates the given encoder
 * @param encoder_id id of the encoder (ODO_****_ENCODER_*****)
 */
void world_update_encoder(int encoder_id);

/**
 * @fn world_update_state()
 * Updates the encoders values (put curr in prev, and get new value for curr) and 
 * updates the state of the robot
 */
void world_update_state();

#endif