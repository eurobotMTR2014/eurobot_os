#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "intel.h"

extern xQueueHandle screenMsgQueue;
extern bool ROBOT_start;

volatile bool game_nearly_stopped = false;

void intelligenceTask (void* pvParameters)
{

    portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    char* msg = pvPortMalloc(sizeof(char) * 21);

    while (!ROBOT_start)
    {
        vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
    }

	vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));

	pln2("AI launched");
    msg = "AI launched";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);	

    

}