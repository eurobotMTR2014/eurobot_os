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

#endif