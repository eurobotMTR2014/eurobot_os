#include "world.h"

#define POSITION_GOAL_BUF_SIZE  500
#define INIT_X_1 0
#define INIT_Y_1 0

/*******************/
/*  GOALS BUFFER   */
/*******************/

typedef struct GoalsBuffer_t {
	unsigned long in, out;				// index the first filled spot and the first empty spot
	 									// of in == out then the buffer is either empty or full
										// the counting semaphore allows to check emptiness and fullness
	xSemaphoreHandle empty_slot_count,  // semaphore counts the empty slots
					 filled_slot_count; // semaphore counts the filled slots
	xSemaphoreHandle goals_mutex; 		// semaphore ensures a mutual exclusion for accessing the buffer
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

	xSemaphoreHandle encoder_mutex;
	
	unsigned long sharp_vals[4]; // sharps measure buffers
	unsigned long ultra_vals[4]; // ultrasound captors measure buffers

	xSemaphoreHandle sharp_mutex;
	xSemaphoreHandle ultra_mutex;

	// robot state
	float x; 		 // position of the robot
   	float y;         // position of the robot
   	float phi;       // angle of the robot
   	bool stop;       // true if the robot must stop

   	xSemaphoreHandle state_mutex;

   	// goals
	GoalsBuffer goals_buffer;
} World;

// semaphores
// extern functions 
extern void updateEncoder(Encoder* enc);

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

/**
 * Initialize the captors related fields
 */ 
static void init_captors();

/**
 * Resets counting semaphores og goal buffer
 */
static void reset_goal_buffer_sem();

/**
 * Copy the content of the first goal structure into the second
 */
static void copy_goal(volatile PositionGoal* from, volatile PositionGoal* to);

/**
 * Copy the content of the first encoder structure into the second
 * (ulBase field is not copied)
 */
static void copy_encoder(volatile Encoder* from, volatile Encoder* to);

// world data
volatile World world;


void init_world()
{
	init_encoders();
	init_state();
	init_goals_buffer();
}

static void init_encoders()
{
	// sets QEI peripheral 
	world.prev_left.ulBase = QEI1_BASE;
	world.curr_left.ulBase = QEI1_BASE;
	world.prev_right.ulBase = QEI0_BASE;
	world.curr_right.ulBase = QEI0_BASE;

	// update encoder values
	updateEncoder(&(world.prev_left));
	updateEncoder(&(world.prev_right));
	updateEncoder(&(world.curr_left));
	updateEncoder(&(world.curr_right));

	// sets encoder mutex semaphore
	world.encoder_mutex = xSemaphoreCreateMutex();
}

static void init_state()
{
	world.x = INIT_X_1;
	world.y = INIT_Y_1;
	world.stop = true;

	world.state_mutex = xSemaphoreCreateMutex();
}

static void init_captors()
{
	world.sharp_mutex = xSemaphoreCreateMutex();
	world.ultra_mutex = xSemaphoreCreateMutex();
}

static void init_goals_buffer()
{
	world.goals_buffer.in = 0;
	world.goals_buffer.out = 0;
	world.goals_buffer.goals_mutex = xSemaphoreCreateMutex();
	reset_goal_buffer_sem();
}

static void reset_goal_buffer_sem()
{
	world.goals_buffer.empty_slot_count = xSemaphoreCreateCounting(POSITION_GOAL_BUF_SIZE, POSITION_GOAL_BUF_SIZE);
	world.goals_buffer.filled_slot_count = xSemaphoreCreateCounting(POSITION_GOAL_BUF_SIZE, 0);
}

static void copy_goal(volatile PositionGoal* from, volatile PositionGoal* to)
{
	to->x = from->x;
	to->y = from->y;
	to->phi = from->phi;
	to->k = from->k;
}

void copy_encoder(volatile Encoder* from, volatile Encoder* to)
{
	to->tickvalue = from->tickvalue;
	to->time = from->time;
	to->forward = from->forward;
}

PositionGoal world_peek_next_goal()
{
	volatile GoalsBuffer* gb = &(world.goals_buffer);
	PositionGoal pg;
	
	xSemaphoreTake(gb->filled_slot_count, portMAX_DELAY); // waits for data in the buffer
	xSemaphoreTake(gb->goals_mutex, portMAX_DELAY); // mutex

	pg = gb->goals[gb->out];

	xSemaphoreGive(gb->goals_mutex);

	return pg;
}

PositionGoal world_pick_next_goal()
{
	volatile GoalsBuffer* gb = &(world.goals_buffer);
	PositionGoal pg;
	
	xSemaphoreTake(gb->filled_slot_count, portMAX_DELAY); // waits for data in the buffer
	xSemaphoreTake(gb->goals_mutex, portMAX_DELAY); // mutex

	pg = gb->goals[gb->out];
	gb->out = (gb->out + 1) % POSITION_GOAL_BUF_SIZE;

	xSemaphoreGive(gb->goals_mutex);
	xSemaphoreGive(gb->empty_slot_count);

	return pg;
}

void world_put_goal(PositionGoal pg)
{
	volatile GoalsBuffer* gb = &(world.goals_buffer);

	xSemaphoreTake(gb->empty_slot_count, portMAX_DELAY);
	xSemaphoreTake(gb->goals_mutex, portMAX_DELAY);

	gb->goals[gb->in] = *pg;
	gb->in = (gb->in + 1) % POSITION_GOAL_BUF_SIZE;

	xSemaphoreGive(gb->goals_mutex);
	xSemaphoreGive(gb->filled_slot_count);
}

void world_goal_flush()
{
	volatile GoalsBuffer* gb = &(world.goals_buffer);

	xSemaphoreTake(gb->goals_mutex, portMAX_DELAY);
	// reset semaphores 
	reset_goal_buffer_sem();

	gb->in = gb->out = 0;

	xSemaphoreGive(gb->goals_mutex);
}

void world_goal_remove_peek()
{
	volatile GoalsBuffer* gb = &(world.goals_buffer);

	xSemaphoreTake(gb->filled_slot_count, portMAX_DELAY);
	xSemaphoreTake(gb->goals_mutex, portMAX_DELAY);

	gb->out = (gb->out + 1) % POSITION_GOAL_BUF_SIZE;

	xSemaphoreGive(gb->goals_mutex);
	xSemaphoreGive(gb->empty_slot_count);
}


Coord world_get_coord()
{	
	Coord c;
	
	xSemaphoreTake(world.state_mutex, portMAX_DELAY);
	c.x = world.x;
	c.y = world.y;
	xSemaphoreGive(world.state_mutex);

	return c;
}

State world_get_state()
{
	State s;

	xSemaphoreTake(world.state_mutex, portMAX_DELAY);
	s.x = world.x;
	s.y = world.y;
	s.phi = world.phi;
	s.stop = world.stop;
	xSemaphoreGive(world.state_mutex);

	return s;
}

void world_update_encoder(int encoder_id)
{
	// call updateEncoder function from odometry.h
	xSemaphoreTake(world.encoder_mutex, portMAX_DELAY);
	switch(encoder_id)
	{
		case ODO_PREV_ENCODER_LEFT : 
			updateEncoder(&(world.prev_left));
			world.prev_left.forward = !world.prev_left.forward;
			world.prev_left.tickvalue = 1023 - world.prev_left.tickvalue;
			break;
		
		case ODO_PREV_ENCODER_RIGHT :
			updateEncoder(&(world.prev_right));
			break;

		case ODO_CURR_ENCODER_LEFT :
			updateEncoder(&(world.curr_left));
			world.curr_left.forward = !world.curr_left.forward;
			world.curr_left.tickvalue = 1023 - world.curr_left.tickvalue;
			break;

		case ODO_CURR_ENCODER_RIGHT :
			updateEncoder(&(world.curr_right));
		  	break;
	}
	xSemaphoreGive(world.encoder_mutex);
}

void world_update_state()
{
	// save current encoder data in prev encoder structures
	xSemaphoreTake(world.encoder_mutex, portMAX_DELAY);
	copy_encoder(&(world.curr_left), &(world.prev_left));
	copy_encoder(&(world.curr_right), &(world.curr_left));
	xSemaphoreGive(world.encoder_mutex);

	// update encoder values
	world_update_encoder(ODO_CURR_ENCODER_RIGHT);
	world_update_encoder(ODO_CURR_ENCODER_LEFT);

	// get displacement based on current tickvalues of encoders
	State ds = getDisplacement(world.curr_right, world.curr_left,
								world.prev_right, world.prev_left);

	// update state of the robot
	xSemaphoreTake(world.state_mutex, portMAX_DELAY);
  	world.phi += ds.phi;
	if (world.phi < -PI) {
	   	world.phi += 2*PI;
	}
	if (world.phi > PI) {
	  	world.phi += -2*PI;
	}
	world.x += ds.x;
	world.y += ds.y;
	xSemaphoreGive(world.state_mutex);
}