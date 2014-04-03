#ifndef CONTROLLER
#define CONTROLLER

#include "definitions.h"

/**
 * Structure used to define a coordinate. Not sure it's useful.
 **/
typedef struct coordinate_t {
   float x;
   float y;
} coordinate;

/**
 * Defines the state of a robot. x, y are the absolute
 * coordinates, phi is the relative angle, and stop is
 * set to true if the robot is idled.
 **/
typedef struct state_t {
   float x;
   float y;
   float phi;
   bool stop;
} state;

void controlTask (void* pvParameters);

/**
 * Initializes the robot's state and the encoders.
 * @param x The x coordinate.
 * @param y The y coordinate.
 * @param phi The relative angle.
 **/
void ctrl_initControl(float x, float y, float phi);

/**
 * When the robot is moving, updates its state from the encoders,
 * launches the planner and the tracker. Makes the thingy move.
 * @param xLastWakeTime You know what this is.
 **/
void ctrl_refresh(portTickType* xLastWakeTime); // Must be refreshed very often

/**
 * Restarts the robot when stopped.
 * @param xLastWakeTime You know what this is.
 * @return true if the restart was successful, false if the restart was not.
 * Typical cases where restart fails: there is no next goal, the robot hasn't stopped.
 **/
bool ctrl_restart(portTickType* xLastWakeTime);

/**
 * Sends an emergency stop command.
 * @param xLastWakeTime You know what this is.
 **/
void ctrl_stop(portTickType* xLastWakeTime);

/**
 * @return The last known state.
 **/
const state* ctrl_getCurrentState();

/**
 * Adds a desired state to the state buffer.
 * @param x The x coordinate, in mm.
 * @param y The y coordinate, in mm.
 * @param phi The relative angle, in rads. Not supported (yet, who knows).
 * @param k The curvature. The lesser, the rougher.
 * @param stop Set to true if the robot must stop when the state is reached.
 **/
void ctrl_setNextGoalState(float x, float y, float phi, float k, bool stop);

/**
  * Erases the goal states. Do not use unless the robot isn't moving, can do unexpected stuff otherwise.
  * You don't want to use this while the robot is moving, it will not change its movement and will eventually kill kittens.
  **/
void ctrl_flush();

/**
 * Reset the robot's state.
 * Do not use unless you really want to reset (recalibration)
 **/
void ctrl_resetState(float x, float y, float phi, bool stop);

/**
 * These functions allow the AI to recalibrate the state.
 * Don't use them unless you know what you're doing.
 **/
void ctrl_calibx(float x);
void ctrl_caliby(float y);
void ctrl_calibphi(float phi);

/**
 * @return 0 if going backwards, 1 if going forward, 2 if stopped.
 */
char ctrl_getForward();

/**
 * Update the encoder enc
 * @param enc a pointer to the encoder structure
 * @note the field ulBase must be set
 */
void updateEncoder(Encoder* enc);

#endif
