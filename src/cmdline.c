#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "cmdline.h"

extern xQueueHandle screenMsgQueue;

extern char servoParam[];
extern char flapParam[];

extern char servoBufferTx[];
extern char flapBufferTx[];

extern char servoBufferRx[];
extern char flapBufferRx[];

extern char servoRxStatus[];
extern char flapRxStatus[];

extern char servoRxParams[];
extern char flapRxParams[];

volatile bool launchcmd = true;

void servoCmdLine(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    char* msg = pvPortMalloc(sizeof(char) * 21);

    while (!launchcmd)
    {
        vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));
    }

    pln("Starting Servo cmd");

    msg = "Starting Interactive";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    msg = " Servo cmd line";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));

    char buf[50];

    while(1)
    {
        UARTprintf("Adr? ");
        UARTgets(buf, 50);
        char adr = m_atoc(buf);

        UARTprintf("Ins? ");
        UARTgets(buf, 50);
        char inst = m_atoc(buf);

        UARTprintf("NbP? ");
        UARTgets(buf, 50);
        char nbp = m_atoc(buf);

        for (char i = 0; i < nbp; ++i)
        {
            UARTprintf("Param %d? ",i);
            UARTgets(buf, 50);
            servoParam[i] = m_atoc(buf);
        }

        pln("Executing\n---\n");
        servoCmd(adr, inst, nbp);

        char rval = servoListen(&xLastWakeTime);
        if (rval != SERVO_RECEIVED_OK)
        {
            UARTprintf("Servo Protocol error: %d\n", rval);
            continue;
        }
        if (!servoRcvStatusOK())
            UARTprintf("Reception error: %d\n", *servoRxStatus);

        if (servoBufferRx[4] == 0)
        {
            pln("OK");
        }
        else
        {
            UARTprintf("ERR: %d\n",servoBufferRx[4]);
        }

        if (servoBufferRx[3] > 2)
        {
            for (char bCount = 0; bCount < (servoBufferRx[3]-2); ++bCount)
            {
                UARTprintf("%d: %d\n",bCount,servoBufferRx[bCount+5]);
            }
        }
    }
}

void flapCmdLine(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    char* msg = pvPortMalloc(sizeof(char) * 21);

    pln("Starting FLAP cmd");

    msg = "Starting Interactive";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    msg = " FLAP cmd line";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));

    char buf[50];

    while(1)
    {
        UARTprintf("Adr? ");
        UARTgets(buf, 50);
        char adr = m_atoc(buf);

        UARTprintf("Ins? ");
        UARTgets(buf, 50);
        char inst = m_atoc(buf);

        UARTprintf("NbP? ");
        UARTgets(buf, 50);
        char nbp = m_atoc(buf);

        for (char i = 0; i < nbp; ++i)
        {
            UARTprintf("Param %d? ",i);
            UARTgets(buf, 50);
            flapParam[i] = m_atoc(buf);
        }

        pln("Executing\n---\n");
        flapCmdUnchecked(adr, inst, nbp);

        char rval = flapListen(&xLastWakeTime);
        if (rval != SERVO_RECEIVED_OK)
        {
            UARTprintf("Servo Protocol error: %d\n", rval);
            continue;
        }
        if (!flapRcvStatusOK())
            UARTprintf("Reception error: %d\n", *flapRxStatus);

        if (flapBufferRx[4] == 0)
        {
            pln("OK");
        }
        else
        {
            UARTprintf("ERR: %d\n",flapBufferRx[4]);
        }

        if (flapBufferRx[3] > 2)
        {
            for (char bCount = 0; bCount < (flapBufferRx[3]-2); ++bCount)
            {
                UARTprintf("%d: %d\n",bCount,flapBufferRx[bCount+5]);
            }
        }
    }
}
