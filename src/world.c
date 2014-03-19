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
void reset_goal_buffer_sem();

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

	// sets initial values for the fields
	updateEncoder(&(world.prev_left));
	updateEncoder(&(world.curr_left));
	updateEncoder(&(world.prev_right));
	updateEncoder(&(world.curr_right));

	// sets encoder mutex semaphore
	world.encoder_mutex = xSemaphoreCreateMutex();
}

static void init_state()
{
	world.x = INIT_X_1;
	world.y = INIT_Y_1;
	world.stop = true;

	world.state_mutex = xSemaphoreCreateMutex()
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

const PositionGoal* world_peek_next_goal()
{
	GoalsBuffer* gb = &(world.goals_buffer);
	PositionGoal* pg;
	
	xSemaphoreTake(gb->filled_slot_count, portMAX_DELAY); // waits for data in the buffer
	xSemaphoreTake(gb->goals_mutex); // mutex

	pg = &(gb->goals[gb->out]);

	xSemaphoreGive(gb->goals_mutex);

	return pg;
}

PositionGoal* world_pick_next_goal()
{
	GoalsBuffer* gb = &(world.goals_buffer);
	PositionGoal* pg;
	
	xSemaphoreTake(gb->filled_slot_count, portMAX_DELAY); // waits for data in the buffer
	xSemaphoreTake(gb->goals_mutex); // mutex

	pg = &(gb->goals[gb->out]);
	gb->out = (gb->out + 1) % POSITION_GOAL_BUF_SIZE;

	xSemaphoreGive(gb->goals_mutex);
	xSemaphoreGive(gb->empty_slot_count);

	return pg;
}

void world_put_goal(PositionGoal* pg)
{
	GoalsBuffer* gb = &(world.goals_buffer);

	xSemaphoreTake(gb->empty_slot_count, portMAX_DELAY);
	xSemaphoreTake(gb->goals_mutex);

	gb->goals[gb->in] = *pg;
	gb->in = (gb->in + 1) % POSITION_GOAL_BUF_SIZE;

	xSemaphoreGive(gb->goals_mutex);
	xSemaphoreGive(gb->filled_slot_count);
}

void world_goal_flush()
{
	GoalsBuffer* gb = &(world.goals_buffer);

	xSemaphoreTake(gb->goals_mutex);
	// reset semaphores 
	xSemaphoreDelete(gb->filled_slot_count);
	xSemaphoreDelete(gb->empty_slot_count);
	reset_goal_buffer_sem();

	gb->in = gb->out = 0;

	xSemaphoreGive(gb->goals_mutex);
}

void world_goal_remove_peek()
{
	GoalsBuffer* gb = &(world.goals_buffer);

	xSemaphoreTake(gb->filled_slot_count);
	xSemaphoreTake(gb->goals_mutex);

	gb->out = (gb->out + 1) % POSITION_GOAL_BUF_SIZE;

	xSemaphoreGive(gb->goals_mutex);
	xSemaphoreGive(gb->empty_slot_count);
}


Coord world_get_coord()
{	
	Coord c;
	
	xSemaphoreTake(world.state_mutex);
	c.x = world.x;
	c.y = world.y;
	xSemaphoreGive(world.state_mutex);

	return c;
}

State world_get_state()
{
	State s;

	xSemaphoreTake(world.state_mutex);
	s.x = world.x;
	s.y = world.y;
	s.phi = world.phi;
	s.stop = world.stop;
	xSemaphoreGive(world.state_mutex);

	return s;
}