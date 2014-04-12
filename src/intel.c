#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "intel.h"
#include "custom_math.h"

extern xQueueHandle screenMsgQueue;
extern bool ROBOT_start;
extern bool intelStop;

volatile bool game_nearly_stopped = false;

bool compareFloat(float x, float y, float eps);

void intelligenceTask (void* pvParameters)
{

    portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    char* msg = pvPortMalloc(sizeof(char) * 21);

    UARTprintf("Before launch\n");

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

    // First hardcoding

    world_add_goal(400, 1500, 42, 42, false);
    world_add_goal(500, 1400, 42, 42, false);
    world_add_goal(750, 1400, 42, 42, false);

    world_add_goal(2250, 1400, 42, 42, true);


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

