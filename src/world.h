#ifndef WORLD_H_DEFINED
#define WORLD_H_DEFINED

#include "definitions.h"

/** Encoder measures identifiers */
#define ODO_PREV_ENCODER_LEFT  0x01
#define ODO_PREV_ENCODER_RIGHT 0x02
#define ODO_CURR_ENCODER_LEFT  0x03
#define ODO_CURR_ENCODER_RIGHT 0x04

/** */


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
  * Get the current speed of the robot
  */
ServoSpeed world_get_servo_speed();

/**
  * Set the current speed of the robot
  */
void world_set_servo_speed(ServoSpeed new_speed);

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

/*
 * Adds a given state to the desired points. REMEMBER phi has to be 42 in most cases (if one want to reach it the fastest possible).
 **/
void world_add_goal(float x, float y, float phi, float k, bool stop);

/**
 * @fn world_set_sharp_vals()
 * Sets the values of the sharp buffers (size 4)
 * @param sharpVals an array of size 4 containing the values from the sharps (1->)
 */
void world_set_sharp_vals(unsigned long sharpVals[]);

/**
 * @fn world_get_sharp_vals()
 * Gets the values of the sharp buffers (size 4)
 * @param sharpVals an array of size 4 in which will be store the values of the sharp
 */
void world_get_sharp_vals(unsigned long sharpVals[]);

void world_get_prev_sharp_vals(unsigned long sharpVals[]);

/**
 * @fn world_set_ultra_vals()
 * Sets the values of the ultrasound captors
 * @param usVals an array of size 4 containing the values of the captors
 */
void world_set_ultra_vals(unsigned long usVals[]);

/**
 * @fn world_get_ultra_vals()
 * Gets the values of the ultrasounds 
 */
void world_get_ultra_vals(unsigned long usVals[]);

void world_get_prev_ultra_vals(unsigned long ultraVals[]);

#endif