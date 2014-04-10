#ifndef WORLD_H_DEFINED
#define WORLD_H_DEFINED

#include "definitions.h"

/** Encoder measures identifiers */
#define ODO_PREV_ENCODER_LEFT  0x01
#define ODO_PREV_ENCODER_RIGHT 0x02
#define ODO_CURR_ENCODER_LEFT  0x03
#define ODO_CURR_ENCODER_RIGHT 0x04

/** Mecanical values */
#ifndef INTER_WHEEL
   #define INTER_WHEEL 257.0
#endif
#ifndef WHEEL_DIAM
   #define WHEEL_DIAM 59.0
#endif
#ifndef ENC_TRNSF
   #define ENC_TRNSF (WHEEL_DIAM*PI)/(2 * 1024.0 * (36.0/22.0)) // Changé 20 -> 22
#endif

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
 * @fn world_goal_isempty()
 * Returns true if the buffer of goals is empty
 */
bool world_goal_isempty();

/**
 * @fn world_goal_isfull()
 * Returns true if the buffer of goals is full
 */
bool world_goal_isfull();

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

/**
 * @fn world_set_stop_state()
 * Set the value 'stop' of the robot.
 */
void world_set_stop_state(bool stop);

/**
 * @fn world_get_stop_state()
 * Get the value 'stop' of the robot.
 */
bool world_get_stop_state();

/**
  * @fn goals_full_or_empty
  * Check if the buffer is full or empty
  */
bool goals_full_or_empty();

/*
 * Adds a given state to the desired points. REMEMBER phi has to be 42 in most cases (if one want to reach it the fastest possible).
 **/
void world_add_goal(float x, float y, float phi, float k, bool stop);

#endif