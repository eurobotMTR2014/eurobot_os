// Just some Git test
#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

/// General includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "definitions.h"

#include "custom_lib.h"

extern volatile bool launchcmd;

typedef unsigned long cpuUsageType;

/// Shared data
xQueueHandle cpuUseQueue = NULL;
xQueueHandle heapFreeQueue = NULL;
xQueueHandle batteryVoltQueue = NULL;
xQueueHandle screenMsgQueue = NULL;

xSemaphoreHandle cpuCounterMutex;
volatile unsigned long cpuUsageCounter = 0;

volatile char ROBOT_team_choice = 1;
volatile bool ROBOT_start = false;
extern bool robot_strategy;
extern volatile bool game_nearly_stopped;

volatile bool controlStop = false;
volatile bool intelStop = false;

xSemaphoreHandle usBufSwitchMutex;
xSemaphoreHandle sharpBufSwitchMutex;

///  Prototype of private functions
void init();
void bootmenu(void);

void idleTask       (void* pvParameters);
void idleStat       (void* pvParameters);
void blinky         (void* pvParameters);
void launchOLED     (void* pvParameters);

void ROOTtask       (void* pvParameters);
bool checkServoStatus(portTickType* xLastWakeTime);
void flapInit(portTickType* xLastWakeTime);
void servoInit(portTickType* xLastWakeTime);

void flapCmdLine    (void* pvParameters);
void servoCmdLine   (void* pvParameters);

void servoLaunchSequence();
void flapLaunchSequence();

void testGiftTask(void* pvParameters);
void testCandleTask(void* pvParameters);
void testQEI(void* pvParameters);
void testADC(void* pvParameters);

int main (void)
{
    init();
    pln("Initialization over");

    //servoLEDWrite();
    servoLaunchSequence();
    //flapLaunchSequence();

    bootmenu();

    // Creating semaphores
    cpuCounterMutex = xSemaphoreCreateMutex();
    sharpBufSwitchMutex = xSemaphoreCreateMutex();
    usBufSwitchMutex = xSemaphoreCreateMutex();

    // Creating queues
    cpuUseQueue = xQueueCreate(1,sizeof(char*));
    heapFreeQueue = xQueueCreate(1,sizeof(char*));
    batteryVoltQueue = xQueueCreate(1,sizeof(unsigned long));
    screenMsgQueue = xQueueCreate(10,sizeof(char*));

    // Defining params


    // Creating tasks
    xTaskCreate(idleTask, (signed char *) "idleTask", 40, NULL, (tskIDLE_PRIORITY), NULL);
    xTaskCreate(idleStat, (signed char *) "idleStat", 100, NULL, (tskIDLE_PRIORITY+1), NULL);
    xTaskCreate(blinky, (signed char *) "blinky", 40, NULL, (tskIDLE_PRIORITY + 2), NULL);
    xTaskCreate(launchOLED, (signed char *) "launchOLED", 100, NULL, (tskIDLE_PRIORITY + 2), NULL);

    //xTaskCreate(servoBroadcast, (signed char *) "servoBroadcast", 1000, NULL, (tskIDLE_PRIORITY + 3), NULL);
    //xTaskCreate(servoCmdLine, (signed char *) "servoCmdLine", 100, NULL, (tskIDLE_PRIORITY + 6), NULL);

    xTaskCreate(ROOTtask, (signed char *) "ROOTtask", 100, NULL, (tskIDLE_PRIORITY + 6), NULL);
    //xTaskCreate(odometryTask, (signed char*) "odometryTask", 1000, NULL, (tskIDLE_PRIORITY + 4), NULL);
    xTaskCreate(captorsTask, (signed char *) "captorsTask", 100, NULL, (tskIDLE_PRIORITY + 5), NULL);
    //xTaskCreate(controlTask, (signed char *) "controlTask", 1000, NULL, (tskIDLE_PRIORITY + 3), NULL);
    //xTaskCreate(intelligenceTask, (signed char *) "intelligenceTask", 1000, NULL, (tskIDLE_PRIORITY + 3), NULL);

    pln("Launching scheduler");
    vTaskStartScheduler();
    pln("Out of scheduler!");


    while(1)
    {}


    return 0;
}


void vApplicationStackOverflowHook(xTaskHandle* pxTask, signed portCHAR *pcTaskName)
{
    RIT128x96x4StringDraw("OVF ", 0, LINE_0, 15);
    RIT128x96x4StringDraw((char*) pcTaskName, 20, LINE_0, 15);
    for( ;; );
}

void init()
{
    // Running at 50Mhz from PLL
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOG;

    /// Led initialisation
    GPIO_PORTG_DIR_R |= LedGreen;
    GPIO_PORTG_DEN_R |= LedGreen;

    RIT128x96x4Init(1000000);

    /// Init all GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);

    /// UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioInit(0);
    //UARTStdioInitExpClk(0, 9600);
    pln("Robot online!");

    /// QEI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinTypeQEI(GPIO_PORTH_BASE, GPIO_PIN_3);
    GPIOPinTypeQEI(GPIO_PORTG_BASE, GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTG_BASE, GPIO_PIN_7);
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET
                             | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1023);
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A | QEI_CONFIG_NO_RESET
                             | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1023);
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);

    /// ADC
    GPIOPinTypeGPIOOutput(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB);
    GPIOPinTypeGPIOOutput(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);
    ADCSequenceConfigure(ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    ADCSequenceEnable(ADC_BASE, 0);

    ADCSequenceConfigure(ADC_BASE, 1, ADC_TRIGGER_PROCESSOR, 1);
    ADCSequenceStepConfigure(ADC_BASE, 1, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
    ADCSequenceEnable(ADC_BASE, 1);

    ADCSequenceConfigure(ADC_BASE, 2, ADC_TRIGGER_PROCESSOR, 2);
    ADCSequenceStepConfigure(ADC_BASE, 2, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
    ADCSequenceEnable(ADC_BASE, 2);

    /// Servos
    GPIOPinTypeGPIOOutput(SERVO_CMD_PIN_BASE, SERVO_CMD_PIN_NB);
    GPIOPinTypeGPIOOutput(FLAP_CMD_PIN_BASE, FLAP_CMD_PIN_NB);

    // UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // ATTENTION : 200000 -> 115200
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // 8bit, stop1, no parity

    UARTEnable(UART1_BASE);
    UARTFIFOEnable(UART1_BASE);

    // UART2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    GPIOPinTypeUART(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // 8bit, stop1, no parity

    UARTEnable(UART2_BASE);
    UARTFIFOEnable(UART2_BASE);

    /// CANON
    GPIOPinTypeGPIOOutput(CANON_PIN_BASE, CANON_PIN_NB);

    /// Security (batteries)
    GPIOPinTypeGPIOOutput(SECURITY_PIN_BASE, SECURITY_PIN_NB);

    /// Start module
    GPIOPinTypeGPIOInput(STARTUP_MODULE_PIN_BASE, STARTUP_MODULE_PIN_NB);

    // Init the World
    init_world();
}

void blinky (void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));
        GPIO_PORTG_DATA_R ^= LedGreen;
    }
}

void launchOLED (void *pvParameters)
{
    RIT128x96x4StringDraw("Initializing OLED", 0, LINE_0, 15);

    if (cpuUseQueue == NULL)
    {
        RIT128x96x4StringDraw("ERR:cpuUseQueue unalloc", 0, LINE_0, 15);
        while(1)
        {}
    }

    // Definining some messages
    char* emptyLine = "";
    //char* INIT = team_choice? "RED" : "BLUE";
    char* fCPU = "CPU:";
    char* percent = "%";
    char* fHeap = "Free Heap:";

    char* bat = "BAT:";
    char* vUnit = "cV";

    const char SCREENRESERVED = 2;
    const char SCREENBUFSIZ = 12;
    char** buf = pvPortMalloc(sizeof(char*) * SCREENBUFSIZ);
    for (unsigned char i =  SCREENRESERVED; i < SCREENBUFSIZ; ++i)
        buf[i] = emptyLine;

    buf[0] = emptyLine;
    buf[1] = emptyLine;

    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    char* tmp_msg = NULL;
    unsigned long tmp_bat = 0;
    int int_bat = 0;
    char* battery_buf = pvPortMalloc(sizeof(char) * 21);

    char newItem = 1;
    char drawCpu = 0;
    char drawHeap = 0;

    while(1)
    {
        // Clearing screen
        if (newItem)
        {
            drawCpu = 1;
            drawHeap = 1;

            RIT128x96x4Clear();
            RIT128x96x4StringDraw(fCPU, 0, LINE_0, 15);
            RIT128x96x4StringDraw(percent, 9*5, LINE_0, 15);
            RIT128x96x4StringDraw(fHeap, 0, LINE_1, 15);
        }

        if (xQueueReceive(batteryVoltQueue, &(tmp_bat), 0) != pdFALSE)
        {
            int_bat = (int)(tmp_bat * 100 / 44.5) + 30;
            m_itoa(int_bat,battery_buf,10);
        }

        if (xQueueReceive(cpuUseQueue, &(tmp_msg), 0) != pdFALSE)
        {
            buf[0] = tmp_msg;
            drawCpu = 1;
        }

        if (xQueueReceive(heapFreeQueue, &(tmp_msg), 0) != pdFALSE)
        {
            buf[1] = tmp_msg;
            drawHeap = 1;
        }

        // Drawing special lines
        // Free CPU
        
        if (drawCpu)
        {
            if (/*int_bat < 1300*/false)
            {
                RIT128x96x4StringDraw("WARNING: LOW BATTERY!!!", 0, LINE_0, 15);
            }
            else
            {
                RIT128x96x4StringDraw("   ", 5*5, LINE_0, 15);
                RIT128x96x4StringDraw(buf[0], 5*5, LINE_0, 15);
                RIT128x96x4StringDraw(bat, 11*5, LINE_0, 15);
                RIT128x96x4StringDraw(vUnit, 21*5, LINE_0, 15);
                RIT128x96x4StringDraw(battery_buf, 16*5, LINE_0, 15);
            }

            drawCpu = 0;
        }



        // Free Heap
        if (drawHeap)
        {
            RIT128x96x4StringDraw("           ", 12*5, LINE_1, 15);
            RIT128x96x4StringDraw(buf[1], 12*5, LINE_1, 15);
            drawHeap = 0;
        }


        if (newItem)
        {
            // Drawing screen
            for (unsigned char i = SCREENRESERVED; i < SCREENBUFSIZ; ++i)
                RIT128x96x4StringDraw(buf[i], 0, i*LINE_SPACE, 15);
        }

        newItem = 0;

        if (xQueueReceive(screenMsgQueue, &(tmp_msg), 0) != pdFALSE)
        {
            // Shifting elements
            for (unsigned char i = SCREENRESERVED; i < SCREENBUFSIZ-1; ++i)
                buf[i] = buf[i+1];

            buf[SCREENBUFSIZ-1] = tmp_msg;
            //m_strcpy(tmp_msg, buf[SCREENBUFSIZ-1]);
            newItem = 1;
        }

        vTaskDelayUntil (&xLastWakeTime, (50 / portTICK_RATE_MS));
    }
}

void idleStat      (void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    const unsigned int MAX_USE = 121643;

    char* msg = pvPortMalloc(sizeof(char) * 21);
    char* freeHeap = pvPortMalloc(sizeof(char) * 21);

    unsigned int index = 0;

    while (1)
    {
        xSemaphoreTake(cpuCounterMutex, portMAX_DELAY);
        /// CPU USAGE
        int percUse = 100 - cpuUsageCounter*100 / MAX_USE;
        if (percUse == 0)
            index = 1;
        else if (percUse < 0)
            index = 0;
        /*else if(percUse > 100)
            index = 101;*/
        else if (percUse == 999)
            index = 998;
        else if(percUse > 999)
            index = 999;
        else
            index = percUse;

        m_itoa(index, msg, 10);
        xQueueSend(cpuUseQueue, (void*) &msg, 0);

        cpuUsageCounter = 0;

        /// FREE HEAP SPACE
        m_itoa(xPortGetFreeHeapSize(), freeHeap, 10);
        xQueueSend(heapFreeQueue, (void*) &freeHeap, 0);

        xSemaphoreGive(cpuCounterMutex);

        vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));
    }
}

void idleTask(void* pvParameters)
{
    while(1)
    {
        xSemaphoreTake(cpuCounterMutex, portMAX_DELAY);
        cpuUsageCounter += 1;
        xSemaphoreGive(cpuCounterMutex);
    }
}

void ROOTtask(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    char* msg = pvPortMalloc(sizeof(char) * 21);

    msg = "Checking servo status";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    //servoRespond(&xLastWakeTime, 2);
    //servoRespond(&xLastWakeTime, 1);

    /*
     * Note : Changer checkServoStatus pour intégrer le flap avant les tests!!!!
     */
    /*
    if (!checkServoStatus(&xLastWakeTime))
    {
        msg = "Error during check";
        xQueueSend(screenMsgQueue, (void*) &msg, 0);
        msg = " abort!";
        xQueueSend(screenMsgQueue, (void*) &msg, 0);
        while (true)
            vTaskDelayUntil (&xLastWakeTime, (10000 / portTICK_RATE_MS));
    }
    */
    msg = "Initializing flaps...";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    //flapInit(&xLastWakeTime);

    msg = "Initializing servos...";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    //servoInit(&xLastWakeTime);


    vTaskDelayUntil (&xLastWakeTime, (200 / portTICK_RATE_MS));

    // Throw a ball
    /*
    msg = "FIRE...";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    */
    //throwSomeSpears(&xLastWakeTime, 6, 3000);

    // Test throw + move
    /*
    throwSomeSpears(&xLastWakeTime, 2, 1500);
    robotForward(&xLastWakeTime, 1000);
    vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));
    throwSomeSpears(&xLastWakeTime, 2, 1500);
    robotForward(&xLastWakeTime, 1000);
    vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));
    throwSomeSpears(&xLastWakeTime, 2, 1500);
    robotForward(&xLastWakeTime, 1000);
    */
    /*    

    if (ROBOT_team_choice)
        msg = "We are on RED team";
    else
        msg = "We are on BLUE team";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    if (robot_strategy == STRAT_1)
        msg = "Using GIFTS strategy";
    else
        msg = "Using CANDLES strategy";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    msg = "Waiting for start...";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    msg = "Time for threads initialization...";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));

    msg = "Waiting for start...";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    vTaskDelayUntil (&xLastWakeTime, (300 / portTICK_RATE_MS));
    while (!GPIOPinRead(STARTUP_MODULE_PIN_BASE, STARTUP_MODULE_PIN_NB))
    {
        // Loop, waiting for start module
    }
    
    seedRandomGen();

    
*/
    vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));
    ROBOT_start = true; // Go!

    msg = "Playing!";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    xLastWakeTime = xTaskGetTickCount();

    vTaskDelayUntil (&xLastWakeTime, (75000 / portTICK_RATE_MS)); // Game happens here
    game_nearly_stopped = true;
    vTaskDelayUntil (&xLastWakeTime, (15000 / portTICK_RATE_MS));
    
    intelStop = true;
    controlStop = true;

    // STOP!
    servoSTOP();
    flapSTOP();
    servoSTOP();
    flapSTOP();
    servoSTOP();
    flapSTOP();
    IntMasterDisable();

    while(true)
    {
        servoSTOP();
        flapSTOP();
    } // Game over
    
    while(true){}
}

bool checkServoStatus(portTickType* xLastWakeTime)
{
    bool ok = true;
    char* msg;
    
    servoCmd(1, INST_PING, 0);
    if (!servoCheck(xLastWakeTime))
    {
        ok = false;
        pln2("Servo 1 error");

        msg = "Servo 1 PAS OK!";
        xQueueSend(screenMsgQueue, (void*) &msg, 0);
    }

    servoCmd(2, INST_PING, 0);
    if (!servoCheck(xLastWakeTime))
    {
        ok = false;
        pln2("Servo 2 error");

        msg = "Servo 2 PAS OK!";
        xQueueSend(screenMsgQueue, (void*) &msg, 0);
    }
    
    
    /*
    flapCmdUnchecked(3, INST_PING, 0);

    if (!flapCheck(xLastWakeTime))
    {
        ok = false;
        pln2("Flap error");
        msg = "Flap PAS OK!";
        xQueueSend(screenMsgQueue, (void*) &msg, 0);
    }
    */
    
    return ok;
}

void flapInit(portTickType* xLastWakeTime)

{
    flapConfig(xLastWakeTime, 60/*90°*/, 150/*180°*/);
 
    flapDown(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (500 / portTICK_RATE_MS));

    flapUp(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (500 / portTICK_RATE_MS));
}


void servoInit(portTickType* xLastWakeTime)
{

    char * msg;
    msg = "FORWARD";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    servoSetSpeed(xLastWakeTime, 2, -0.5);
    servoSetSpeed(xLastWakeTime, 1, 0.5);
    servoSync();

    vTaskDelayUntil (xLastWakeTime, (2000 / portTICK_RATE_MS));
    servoSTOP();

    msg = "WAIT";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);

    vTaskDelayUntil (xLastWakeTime, (1000 / portTICK_RATE_MS));

    msg = "BACKWARD";
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    
    servoSetSpeed(xLastWakeTime, 1, -0.5);
    servoSetSpeed(xLastWakeTime, 2, 0.5);
    servoSync();
    
    vTaskDelayUntil(xLastWakeTime, (2000 / portTICK_RATE_MS));

    servoSTOP();
}

void testGiftTask(void* pvParameters)
{
    portTickType xLastWakeTime_tmp;
    xLastWakeTime_tmp = xTaskGetTickCount();
    portTickType* xLastWakeTime = &xLastWakeTime_tmp;

    flapRightConfig(xLastWakeTime);
    flapLeftConfig(xLastWakeTime);

    flapRightDown(xLastWakeTime);
    flapLeftDown(xLastWakeTime);

    int delay = 1000;

    while (true)
    {
        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapRightUp(xLastWakeTime);

        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapRightDown(xLastWakeTime);

        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapLeftUp(xLastWakeTime);

        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapLeftDown(xLastWakeTime);
    }
}

void testCandleTask(void* pvParameters)
{
    portTickType xLastWakeTime_tmp;
    xLastWakeTime_tmp = xTaskGetTickCount();
    portTickType* xLastWakeTime = &xLastWakeTime_tmp;

    flapRightConfig(xLastWakeTime);
    flapLeftConfig(xLastWakeTime);

    flapRightUp(xLastWakeTime);
    flapLeftUp(xLastWakeTime);

    int delay = 1000;

    while (true)
    {
        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapRightBall(xLastWakeTime);

        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapRightUp(xLastWakeTime);

        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapLeftBall(xLastWakeTime);

        vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
        flapLeftUp(xLastWakeTime);
    }
}

void testQEI(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    char* msg = pvPortMalloc(sizeof(char) * 21);

    unsigned long ulval = 0;
    portTickType tick = 0; // FreeRTOS specific type. Might not be included and may throw errors, to fix.

    pln("Starting encoder/timer test");
    while(1)
    {

        tick = xTaskGetTickCount(); // FreeRTOS specific function. Should be available if vTaskStartScheduler has been started.
        m_ultoa(tick, msg, 10);
        UARTprintf("Tick: ");
        pln(msg);

        ulval = QEIPositionGet(QEI0_BASE);
        m_ultoa(ulval, msg, 10);
        UARTprintf("Pos1: ");
        pln(msg);


        ulval = QEIDirectionGet(QEI0_BASE);
        m_ultoa(ulval, msg, 10);
        UARTprintf("Dir1: ");
        pln(msg);

        ulval = QEIPositionGet(QEI1_BASE);
        m_ultoa(ulval, msg, 10);
        UARTprintf("Pos2: ");
        pln(msg);

        ulval = QEIDirectionGet(QEI1_BASE);
        m_ultoa(ulval, msg, 10);
        UARTprintf("Dir2: ");
        pln(msg);

        /*ulval = QEIIntStatus(QEI_BASE,false);
        m_ultoa(ulval, msg, 10);
        UARTprintf("Int: ");
        pln(msg);*/

        if (QEIErrorGet(QEI0_BASE))
            pln2("ERROR IN QEI 1");

        if (QEIErrorGet(QEI1_BASE))
            pln2("ERROR IN QEI 2");

        pln("---");
        //xQueueSend(screenMsgQueue, (void*) &msg, 0);
        vTaskDelayUntil (&xLastWakeTime, (1000 / portTICK_RATE_MS));
    }
}

void testADC(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    char* msg = pvPortMalloc(sizeof(char) * 21);
    unsigned long ADCval = 0;

    GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_OFF);  // Ultrasons avant
    GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_OFF);

    while(1)
    {
        ADCProcessorTrigger(ADC_BASE, 0);

        while(!ADCIntStatus(ADC_BASE, 0, false))
        {}

        ADCSequenceDataGet(ADC_BASE, 0, &ADCval);

        m_ultoa(ADCval, msg, 10);

        pln(msg);
        xQueueSend(screenMsgQueue, (void*) &msg, 0);

        vTaskDelayUntil (&xLastWakeTime, (20 / portTICK_RATE_MS));
    }
}

void bootmenu(void)
{
    RIT128x96x4StringDraw("Mymosh sys - v3.0", 0, LINE_0, 15);
    RIT128x96x4StringDraw("BOOT MENU", 0, LINE_1, 15);
    RIT128x96x4StringDraw("______________", 0, LINE_2, 15);

    /*
    PF4 = UP
    PF5 = DOWN
    PF6 = LEFT
    PF7 = RIGHT
    */
    //*

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,
                         GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTF_BASE,
                     GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);


    /*

    RIT128x96x4StringDraw("Which team?", 0, LINE_3, 15);



    while (1)
    {
        if (ROBOT_team_choice)
        {
            RIT128x96x4StringDraw("-> Red", 0, LINE_5, 15);
            RIT128x96x4StringDraw("Blue   ", 0, LINE_6, 15);
        }
        else
        {
            RIT128x96x4StringDraw("Red   ", 0, LINE_5, 15);
            RIT128x96x4StringDraw("-> Blue", 0, LINE_6, 15);
        }

        while( GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)
               && GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_5)
               && GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_4)
               )
        {
            // loop
        }

        if (!GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_4))
            break;

        ROBOT_team_choice = !ROBOT_team_choice;

        for (int i = 0; i < 1000000; i++)
        {
            // loop
        }
    }

    for (int i = 0; i < 1000000; i++)
    {
        // loop
    }

    RIT128x96x4StringDraw("Which strategy?", 0, LINE_3, 15);

    while (1)
    {
        if (robot_strategy)
        {
            RIT128x96x4StringDraw("-> Gifts", 0, LINE_5, 15);
            RIT128x96x4StringDraw("Candles   ", 0, LINE_6, 15);
        }
        else
        {
            RIT128x96x4StringDraw("Gifts   ", 0, LINE_5, 15);
            RIT128x96x4StringDraw("-> Candles", 0, LINE_6, 15);
        }

        while( GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)
               && GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_5)
               && GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_4)
               )
        {
            // loop
        }

        if (!GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_4))
            break;

        robot_strategy = !robot_strategy;

        for (int i = 0; i < 1000000; i++)
        {
            // loop
        }
    }
    // */
}

void servoLaunchSequence()
{
    servoFreeWheel();
    servoSTOP();
}

void flapLaunchSequence()
{
}

void errorReport(char* msg)
{
    /*char* buf = pvPortMalloc(sizeof(char) * 21);
    m_strcpy(msg,buf);
    xQueueSend(screenMsgQueue, (void*) &buf, 0);*/
}

void batteryReport(unsigned long bVolt)
{
    xQueueSend(batteryVoltQueue, (void*) &bVolt, 0);
}



/**  End of main.c  **/

