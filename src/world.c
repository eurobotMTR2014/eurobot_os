#include "world.h"

#define POSITION_GOAL_BUF_SIZE  500

/*******************/
/*  GOALS BUFFER   */
/*******************/

typedef struct GoalsBuffer_t {
	unsigned long in, out;				// index the first filled spot and the first empty spot
	 									// of in == out then the buffer is either empty or full
										// the counting semaphore allows to check emptiness and fullness
	xSemaphoreHandle empty_slot_count,  // semaphore counts the empty slots
					 filled_slot_count; // semaphore counts the filled slotes
	xSemaphoreHandle goals_mutex; 		// semaphore ensure a mutual exclusion for accessing the buffer
	PositionGoal goals[POSITION_GOAL_BUF_SIZE]; // buffer
} GoalsBuffer;

/***************/
/*    WORLD    */
/***************/

// struct holding the world data
typedef struct World_t {

	// odometry and captors measures
	// previous fetch
	Encoder prev_right;
	Encoder prev_left;
	// current fetch
	Encoder curr_right;
	Encoder curr_left;

	
	unsigned long sharp_vals[4]; // sharps measure buffers
	unsigned long ultra_vals[4]; // ultrasound captors measure buffers

	// robot state
	float x; 		 // position of the robot
   	float y;         // position of the robot
   	float phi;       // angle of the robot
   	bool stop;       // true if the robot must stop

   	// goals
	GoalsBuffer goals_buffer;
} World;

// semaphores


// static functions 
/**
 * Initialize the fields of each encoder structure of the World
 */
static void init_encoders();

/**
 * Initialize robot state : position, angle and stop
 */
static void init_state();

/**
 * Initialize the goals buffer
 */
static void init_goals_buffer();

// world data
World world;


void init_world()
{
	init_encoders();
	init_state();
	init_goals_buffer();

}

static void init_encoders()
{
	// sets QEI peripheral 
	world->prev_left.ulBase = QEI1_BASE;
	world->curr_left.ulBase = QEI1_BASE;
	world->prev_right.ulBase = QEI0_BASE;
	world->curr_right.ulBase = QEI0_BASE;

	// sets initial values for the fields
	updateEncoder(&(world->prev_left));
	updateEncoder(&(world->curr_left));
	updateEncoder(&(world->prev_right));
	updateEncoder(&(world->curr_right));
}

static void init_state()
{
	world->x = INIT_X_1;
	world->y = INIT_Y_1;
	world->stop = true;
}


static void init_goals_buffer()
{
	world->goals_buffer->in = 0;
	world->goals_buffer->out = 0;
	world->goals_buffer->empty_slot_count = xSemaphoreCreateCounting(POSITION_GOAL_BUF_SIZE, POSITION_GOAL_BUF_SIZE);
	world->goals_buffer->filled_slot_count = xSemaphoreCreateCounting(POSITION_GOAL_BUF_SIZE, 0);
	world->goals_buffer->goals_mutex = xSemaphoreCreateBinary();
}