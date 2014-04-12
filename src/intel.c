#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "intel.h"
#include "custom_math.h"

#define DISTANCE_LIMIT_DETECTION 20.0

#define ACTION_INTERRUPTED 1
#define ACTION_CURRENT 2
#define ACTION_DONE 3
#define ACTION_POSSIBLE 4


// an action
#define PATH_LENGTH 40
typedef struct Action_t
{
    int state; // state of the action
    char name[10];    // name of the action
    Coord path[PATH_LENGTH];   // path to reach the action
    int nb_point; // number of points (Coord) in path
    //float require_phi; // required phi for doing the action [-pi, pi]
    int (*action_fn)(void); // function that does the action
} Action;


// buffers that stores the actions
#define ACTION_BUF_SIZE 20
typedef struct ActionBuffers_t
{
    Action actionBuffer[ACTION_BUF_SIZE]; // stores all the actions for a game
    Action* current_action; // holds a pointer to the current action
    Action* last_interrupted_action; // holds the pointer to the last interrupted action
    Action* action_todo_buffer[ACTION_BUF_SIZE]; // holds pointers to action that have to be done
    int ac_buf_size; // number of actions in action buffer
    int ac_todo_size; // number of actions in the "action_todo_buffer"
} ActionBuffers;

static ActionBuffers ab;


extern xQueueHandle screenMsgQueue;
extern bool ROBOT_start;
extern bool intelStop;

volatile bool game_nearly_stopped = false;

static bool ENEMY_FRONT, ENEMY_BACK;


/**
 * Initializes all the intel's data structures
 */
static void intel_init();

/**
 * Initializes the actions to be performed
 */
static void intel_init_actions();

/**
 * Returns true if the current action must be interrupted 
 * (because enemy was detected in the path or state has changed)
 */
static bool intel_curr_need_interrupt();

/**
 * Puts back the last interrupted action (if there is one) in the todo buffer (at the end of it)
 * The pointer "last_interupted_action" is set to NULL and ready to receive another action
 */
static void intel_restore_last_interrupted();

/**
 * Puts the current action in the last_interrupted_action (after putting the prev. last_int. back in todo)
 */
static void intel_set_curr_interrupted();

/**
 * Picks a new action to be the current one. The ctrl is refreshed with the new goals
 */
static void intel_refresh_current();

/**
 * Returns true if the final coord for the current action was reached
 */
static bool intel_has_reached_action_dest();

/**
 * Find the nearest point (regarding the current state) in the 
 * path vector of the given action
 */
static int intel_find_nearest_coord(Action* a);

bool compareFloat(float x, float y, float eps);
bool acceptableDistance(float distance);
bool isEnemyFront();
//bool isEnemyBack();

void intelligenceTask (void* pvParameters)
{

    portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    char* msg = pvPortMalloc(sizeof(char) * 21);

    UARTprintf("Before launch\n");

    ENEMY_FRONT = false;
    ENEMY_BACK = false;

    while (!ROBOT_start)
    {
        vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
    }

	vTaskDelayUntil (&xLastWakeTime, (500 / portTICK_RATE_MS));

	UARTprintf("AI launched");
    msg = "AI launched";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);	

    //ctrl_initControl(0, 0, 0);

    msg = "world_add_goal";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    
    while(true){
        if(isEnemyFront()){
            UARTprintf("============= ENNEMY !!!! ============\n");
        }

       vTaskDelayUntil (&xLastWakeTime, (800 / portTICK_RATE_MS));
    }
        

//    int i = 0;


    msg = "Begin loop";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);


     while(!intelStop)
    {
        // checks if current needs refresh
        if(ab.current_action == NULL && 
            ((ab.ac_todo_size == 0 && ab.last_interrupted_action != NULL) || ab.ac_todo_size != 0))
        {
            intel_restore_last_interrupted();
            intel_refresh_current();
        }
        else if(ab.current_action == NULL) // nothing else to do (all actions were executed)
            break;

        // picks a new action if it has to be 
        if(intel_curr_need_interrupt())
        {
            intel_set_curr_interrupted();
            intel_refresh_current();
        }

        // the destination point of an action has been reached
        if(intel_has_reached_action_dest())
        {
            ab.current_action->action_fn(); // execute handler
            ab.current_action->state = ACTION_DONE;
            intel_refresh_current(); // refresh current (the previous is now considered as )
        }


        vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_RATE_MS));
    }


    while(1){}
}





bool acceptableDistance(float distance)
{
    if(distance < DISTANCE_LIMIT_DETECTION)
        return false;

    return true;
}

bool isEnemyFront()
{
    unsigned long /*USvalues[2], */
                    sharpValues[2], 
                    /*prev_USvalues[2], */
                    prev_sharpValues[2];
    bool ENEMY_FRONT;

    //world_get_ultra_vals(USvalues);
    world_get_sharp_vals(sharpValues);
    world_get_prev_sharp_vals(prev_sharpValues);
    //world_get_prev_ultra_vals(prev_USvalues);
    
    UARTprintf("Sharp : %d ; %d\n", sharpValues[0],sharpValues[1]);
    //UARTprintf("US : %d ; %d\n", USvalues[0],USvalues[1]);

    int count = 0;
    if(!acceptableDistance(sharp_convert(sharpValues[0])))
        ++count;
    if(!acceptableDistance(sharp_convert(sharpValues[1])))
        ++count;
    //if(!acceptableDistance(ultrason_convert(USvalues[1]))) 
      //  ++count;
    if(!acceptableDistance(sharp_convert(prev_sharpValues[0])))
        ++count;
    if(!acceptableDistance(sharp_convert(prev_sharpValues[1])))
        ++count;
    //if(!acceptableDistance(ultrason_convert(prev_USvalues[1])))
      //  ++count;

    if(count >= 3)
        ENEMY_FRONT = true;
    else 
        ENEMY_FRONT = false;

    return ENEMY_FRONT;
}

/*
bool isEnemyBack()
{
    unsigned long USvalues[2], prev_USvalues[2];
    bool ENEMY_BACK;

    world_get_ultra_vals(USvalues);
    world_get_prev_ultra_vals(prev_USvalues);

    ENEMY_BACK = (!acceptableDistance(ultrason_convert(USvalues[0])) || !acceptableDistance(ultrason_convert(prev_USvalues[0]))); 

    return ENEMY_BACK;
}
*/



void intel_init()
{
    ab.current_action = NULL;
    ab.last_interrupted_action = NULL;
    ab.ac_todo_size = ab.ac_buf_size = 0; 

    intel_init_actions();
}

void intel_init_actions()
{
 /// put actions here
}

bool intel_curr_need_interrupt()
{
    return false; // not final -> must be modified with values from 
}

void intel_restore_last_interrupted()
{
    if(ab.last_interrupted_action == NULL)
        return;

    ab.last_interrupted_action->state = ACTION_POSSIBLE;
    ab.action_todo_buffer[ab.ac_todo_size++] = ab.last_interrupted_action;
    ab.last_interrupted_action = NULL; 
}

void intel_set_curr_interrupted()
{
    if(ab.current_action == NULL)
        return;

    intel_restore_last_interrupted();
    ab.current_action->state = ACTION_INTERRUPTED;
    ab.last_interrupted_action = ab.current_action;
    ab.current_action = NULL;
}

void intel_refresh_current()
{
    // if an action is stored then, it's considered as done
    if(ab.current_action != NULL)
        ab.current_action->state = ACTION_DONE;
    
    // find the new action
    unsigned int new_current_index = getPseudoRandomNumber(ab.ac_todo_size);
    Action* new_current = ab.action_todo_buffer[new_current_index];

    for(int i = new_current_index; i < ab.ac_todo_size - 1; i++)
        ab.action_todo_buffer[i] = ab.action_todo_buffer[i + 1];

    ab.ac_todo_size--;
    
    new_current->state = ACTION_CURRENT;
    ab.current_action = new_current;

    // refresh control

    // stop control
    world_set_stop_state(true);
    world_goal_flush();

    // write new goals in the world
    int nearest_path_coord_index = intel_find_nearest_coord(new_current);
    for(int i = nearest_path_coord_index; i < new_current->nb_point; i++)
    {
        bool stop_ctrl = (i == new_current->nb_point - 1);
        world_add_goal(new_current->path[i].x, new_current->path[i].y, 42, 42, stop_ctrl);   
    }  

    // restart control
    world_set_stop_state(false);

}

bool intel_has_reached_action_dest()
{
    State curr_state = world_get_state();
    int nb_points_in_path = ab.current_action->nb_point;
    Coord last_coord = ab.current_action->path[nb_points_in_path - 1];

    float dx = last_coord.x - curr_state.x, 
          dy = last_coord.y - curr_state.y;

    if(custom_sqrt(dx * dx + dy * dy) <= EPSILON)
        return true;

    return false;
}

int intel_find_nearest_coord(Action* a)
{
    State curr_state = world_get_state();

    int min_index, min = 500000;

    for(int i = 0; i < a->nb_point; i++)
    {
        float dx = curr_state.x - a->path[i].x,
                dy = curr_state.y - a->path[i].y,
                dist = custom_sqrt(dx * dx + dy * dy);

        if(dist < min)
        {
            min = dist;
            min_index = i;
        }
    }

    return min_index;
}
