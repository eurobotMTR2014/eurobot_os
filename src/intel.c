#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "intel.h"
#include "custom_math.h"

#define DISTANCE_LIMIT 10


extern xQueueHandle screenMsgQueue;
extern bool ROBOT_start;
extern bool intelStop;

volatile bool game_nearly_stopped = false;

static bool ENEMY_FRONT, ENEMY_BACK;

bool compareFloat(float x, float y, float eps);

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

    msg = "ctrl_setNextGoalState";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    /*
    world_add_goal(500, 0, 42, 42, false);
    world_add_goal(1000, 0, 42, 42, false);
    world_add_goal(1200, 0, 42, 42, false);
    world_add_goal(1500, 0, 42, 42, false);
    world_add_goal(1500, 500, 42, 42, false);
    world_add_goal(1500, 1000, 42, 42, false);
    world_add_goal(1500, 1200, 42, 42, false);
    world_add_goal(1500, 1500, 42, 42, false);
    world_add_goal(1200, 1500, 42, 42, false);
    world_add_goal(1000, 1500, 42, 42, false);
    world_add_goal(500, 1500, 42, 42, false);
    world_add_goal(0, 1500, 42, 42, false);
    world_add_goal(0, 1200, 42, 42, false);
    world_add_goal(0, 1000, 42, 42, false);
    world_add_goal(0, 500, 42, 42, false);
    world_add_goal(0, 0, 42, 42, true);
    ctrl_restart(&xLastWakeTime);
    */
//    int i = 0;

    world_add_goal(500, 0, 42, 42, false);
    world_add_goal(250, 0, 42, 42, true);

    msg = "Begin loop";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    /*
    state curr, prev;
    const state* temp = ctrl_getCurrentState();
    curr.x = temp->x;
    curr.y = temp->y;
    curr.phi = temp->phi;
    */
    /*
    state curr;
    while(!intelStop)
    {
        //prev = curr;
        const state* got = ctrl_getCurrentState();
        curr.x = got->x;
        curr.y = got->y;
        curr.phi = got->phi;

        //UARTprintf("x = %d, y = %d; phi = %d\n", (int)curr.x, (int)curr.y, (int)curr.phi);
        //UARTprintf("state (in mm) (x = %d, ", (int) (curr.x * 1.0)); // %d = int, %u = uint.
        //UARTprintf("y = %d, ", (int) (curr.y * 1.0)); // %d = int, %u = uint.
        //UARTprintf("phi = %d (in rad*1000))\n", (int) (curr.phi * 1000.0)); // %d = int, %u = uint.
        /*
        if(compareFloat(curr.x, prev.x,0.0001) || compareFloat(curr.y, prev.y,0.0001) || compareFloat(curr.phi, prev.phi,0.0001) || curr.stop != prev.stop){
            msg = "Changed!";
            xQueueSend(screenMsgQueue, (void*) &msg, 0);
        }
        */
/*
        vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_RATE_MS));
    }
    */

    while(1){}
}



bool acceptableDistance(float distance)
{
    if(distance < DISTANCE_LIMIT)
        return false;

    return true;
}

bool isEnemyFront()
{
    unsigned long USvalues[2], sharpValues[2], prev_USvalues[2], prev_sharpValues[2];
    bool ENEMY_FRONT;

    world_get_ultra_vals(USvalues);
    world_get_sharp_vals(sharpValues);
    world_get_prev_sharp_vals(prev_sharpValues);
    world_get_prev_ultra_vals(prev_USvalues);

    int count = 0;
    if(!acceptableDistance(sharp_convert(sharpValues[0])))
        ++count;
    if(!acceptableDistance(sharp_convert(sharpValues[1])))
        ++count;
    if(!acceptableDistance(ultrason_convert(USvalues[0]))) //To check ! maybe it's back...
        ++count;
    if(!acceptableDistance(sharp_convert(prev_sharpValues[0])))
        ++count;
    if(!acceptableDistance(sharp_convert(prev_sharpValues[1])))
        ++count;
    if(!acceptableDistance(ultrason_convert(prev_USvalues[0]))) //To check ! maybe it's back...
        ++count;

    if(count >= 4)
        ENEMY_FRONT = true;
    else 
        ENEMY_FRONT = false;

    return ENEMY_FRONT;
}

bool isEnemyBack()
{
    unsigned long USvalues[2], prev_USvalues[2];
    bool ENEMY_BACK;

    world_get_ultra_vals(USvalues);
    world_get_prev_ultra_vals(prev_USvalues);

    ENEMY_BACK = (!acceptableDistance(ultrason_convert(USvalues[1])) && !acceptableDistance(ultrason_convert(prev_USvalues[1]))); // A vérifier

    return ENEMY_BACK;
}
