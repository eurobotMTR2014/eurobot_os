#include "world.h"

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

typedef struct OdometryData_t {
	// previous fetch
	Encoder prev_right;
	Encoder prev_left;
	// current fetch
	Encoder curr_right;
	Encoder curr_left;
} OdometryData;

/*********************/
/*     CAPTORS       */
/*********************/

typedef struct Captors_t {
	unsigned long sharp_vals[4]; // sharps measure buffers
	unsigned long ultra_vals[4]; // ultrasound captors measure buffers
} Captors;

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
typedef MoveGoal_t
{
	float x; 		// final position
	float y;		// final position
	float phi;		// final angle
	float k;        // curvature
} MoveGoal;


/***************/
/*    WORLD    */
/***************/

// struct holding the world data
typedef struct World_t {
	OdometryData odo;
	Captors capt;
	State state;
	MoveGoal mg;
} World;


// world data
World world;


void init_world()
{
	
}



