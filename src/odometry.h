
#ifndef ODOMETRY
#define ODOMETRY

#include "definitions.h"

void odometryTask (void* odometryTask);

/**
  * Update the values of an encoder
  */
void updateEncoder(volatile Encoder* enc);

/**
 * Returns in a State struct the displacement of the robot (dx, dy and dphi) 
 * @param er structure containing the current data fetched from the right encoder
 * @param el structure containing the current data fetched from the left encoder
 * @param last_er structure containing the previous data fetched from the right encoder
 * @param last_el structure containing the previous data fetched from the left encoder
 * @param cs_phi the current orientation of the robot ("currentstate.phi")
 */
State getDisplacement(Encoder er, Encoder el, Encoder last_er, Encoder last_el, float cs_phi);

#endif