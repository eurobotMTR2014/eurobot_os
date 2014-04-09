
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
 */
State getDisplacement(Encoder er, Encoder el, Encoder last_er, Encoder last_el);

#endif