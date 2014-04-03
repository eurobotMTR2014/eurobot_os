#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "intel.h"

extern xQueueHandle screenMsgQueue;
extern xSemaphoreHandle usBufSwitchMutex;
extern xSemaphoreHandle sharpBufSwitchMutex;
extern char ROBOT_team_choice;
extern bool ROBOT_start;
extern bool intelStop;

extern bool usBufSwitch;
extern bool sharpBufSwitch;
extern unsigned long* ultraValsBuf1;
extern unsigned long* ultraValsBuf2;
extern unsigned long* sharpValsBuf1;
extern unsigned long* sharpValsBuf2;


volatile bool game_nearly_stopped = false;

static unsigned long intelUSBuf[4];
static unsigned long intelSharpBuf[4];
static float usDistance[4];
static float sharpDistance[4];
static char FWcontactcounter = 0;
static char FWnearcounter = 0;
static char FWsightcounter = 0;

static bool reverse_stopped = false;
static bool stopped_for_enemy = false;
static bool wait_for_enemy = false;
static bool startEncounter = false;
static bool encounterEscaping = false;
static bool encounterLock = false;
static portTickType encounterTick;
static unsigned int wait_watchdog = 0;


static bool gifts_done[4];
static bool balls_done[4];
static bool juice_self_done[10];
static bool juice_enemy_done[4];
static bool candles_done[4];
static char move_direction = 2;
bool robot_strategy = STRAT_1;


enum Task
{
    STARTING_JUICE,
    FINISHING_JUICE,
    DUMPING_JUICE,
    DUMPING_REVERSE,
    GOING_CAKE,
    CANDLES,
    CANDLES_FINISHED,
    GOING_GIFTS,
    GIFTS,
    GIFTS_FINISHED,
    NONE
};

enum Task currentTask = NONE;

static const state* robotState;

void setStrategy();
bool chooseGiftsTask();
void getStatus();

void detectEnemy();
void usDetect();
void sharpDetect();
void getSharpAdditionalInfo();
void refreshEnemyInfo();
bool outOfBoundaries(int distance);

#define DEF_XY_TOL 25
#define DEF_TH_TOL 0.087
bool isAtPos(float x, float y, float th);
bool isAtPosTol(float x, float y, float th, float xyTol, float thTol);
float dist(float x, float y);
float dist2(float x1, float y1, float x2, float y2);

void intel_flapLeftUp(portTickType* xLastWakeTime);
void intel_flapLeftBall(portTickType* xLastWakeTime);
void intel_flapLeftDown(portTickType* xLastWakeTime);
void intel_flapRightUp(portTickType* xLastWakeTime);
void intel_flapRightBall(portTickType* xLastWakeTime);
void intel_flapRightDown(portTickType* xLastWakeTime);

void intel_flush();
void intel_initControl(float x, float y, float phi);
void intel_setNextGoalState(float x, float y, float phi, float k, bool stop, bool noSpline);
void intel_restart(portTickType* xLastWakeTime);

void forceAngle(float th);

void doSpline();
void endSpline();

void decideActions(portTickType* xLastWakeTime);
void checkEnemyAction(portTickType* xLastWakeTime);
void encounterEscapeContact(portTickType* xLastWakeTime, bool reverse);
void encounterEscape(portTickType* xLastWakeTime);
void encounterEscapeR(portTickType* xLastWakeTime, bool reverse);
void encounterDeadlock(portTickType* xLastWakeTime);
void encounterDeadlockR(portTickType* xLastWakeTime, bool reverse);
void intel_recover();
void intel_recoverR();

void initTasks();
void goTask(char sel);
void setJuiceTask1();
void setJuiceTask2();
void setDumpJuiceTask1();
void setDumpJuiceTask2();
void setDumpFinal();
void setParkTask1();
void setParkTask2();
void setGiftsTask1();
void setGiftsTask2();
void setCandlesTask1();
void setCandlesTask2();

typedef enum wheelStatus
{
    WHEEL_STOP,
    WHEEL_FORWARD,
    WHEEL_BACKWARD,
} wheelStatus;

typedef enum enemyStatus
{
    ENEMY_NOT_AROUND = 0,
    ENEMY_IN_SIGHT = 1,
    ENEMY_NEAR = 2,
    ENEMY_CONTACT = 3,
} enemyStatus;


#define SIGHT_DIST 100
#define NEAR_DIST 60
#define CONTACT_DIST 40

wheelStatus wStat = WHEEL_STOP;

enemyStatus enemyForward = ENEMY_NOT_AROUND;
char extra_info = 0; // If right, 1, if left, 2, if center 3.
enemyStatus enemyBackwardR = ENEMY_NOT_AROUND;
enemyStatus enemyBackwardL = ENEMY_NOT_AROUND;
char bwrcounter = 0;
char bwlcounter = 0;

//
enemyStatus usEnemyForward = ENEMY_NOT_AROUND;
enemyStatus usEnemyBackward = ENEMY_NOT_AROUND;
enemyStatus sharpEnemyForwardC = ENEMY_NOT_AROUND;
enemyStatus sharpEnemyForwardL = ENEMY_NOT_AROUND;
enemyStatus sharpEnemyForwardR = ENEMY_NOT_AROUND;
char sharpLcounter = 0;
char sharpRcounter = 0;


#define NB_TASKS 7
static unsigned int tasks_weights[NB_TASKS];
static unsigned int natural_weights[NB_TASKS];
static unsigned int working_weights[NB_TASKS];

static float prec_x = INIT_1_X;
static float prec_y = INIT_1_Y;
static float walked_dist = 0;

void intelligenceTask (void* pvParameters)
{

    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (!ROBOT_start)
    {
        vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
    }

    pln2("AI launched");

    intel_initControl(INIT_1_X, INIT_1_Y, PI/2); // 610 , 220
    getStatus();

    ctrl_setNextGoalState(INIT_1_X, INIT_1_Y + 500.0, PI, 200, true);
    ctrl_setNextGoalState(INIT_1_X + 500.0, INIT_1_Y + 500.0, PI, 200, true);
    ctrl_setNextGoalState(INIT_1_X + 500.0, INIT_1_Y, PI, 200, true);
    ctrl_setNextGoalState(INIT_1_X, INIT_1_Y, PI, 200, true);



    /*
    initTasks();
    intel_initControl(INIT_1_X, INIT_1_Y, PI/2);
    getStatus();

    setStrategy();
//    ctrl_initControl(610, 220, PI/2);
//    ctrl_setNextGoalState(200, 1500, 42, 200, false);
//    ctrl_setNextGoalState(400, 1500, 42, -200, false);
//    ctrl_setNextGoalState(1000, 2800, 42, 170, false);
//    ctrl_setNextGoalState(1900, 1500, 42, 170, false);
//    ctrl_setNextGoalState(1000, 100, 42, 170, false);
//    ctrl_setNextGoalState(1000, 1500, 42, 170, true);
//    ctrl_restart(xLastWakeTime);

    intel_restart(&xLastWakeTime);

    unsigned int stop_wdog = 0;
    while (!intelStop)
    {
        getStatus();

        if (robotState->stop)
        {
            UARTprintf("STOPPED: %d\n",stop_wdog);
            ++stop_wdog;

            if (stop_wdog > INTEL_WDOG_LIMIT)
            {
                UARTprintf("wdog1\n");
                ctrl_stop(&xLastWakeTime);
                intel_flush();
                intel_recover();
                intel_restart(&xLastWakeTime);
                stopped_for_enemy = false;
                startEncounter = false;
                wait_for_enemy = false;
                encounterEscaping = false;
                reverse_stopped = false;
                wait_watchdog = 0;
                stop_wdog = 0;
            }
        }
        else
        {
            stop_wdog = 0;
        }

        detectEnemy();
        decideActions(&xLastWakeTime);

        vTaskDelayUntil (&xLastWakeTime, (50 / portTICK_RATE_MS));
    }

    */
    while(true);
}

static bool initStrategy = false;
static enum Task precAlert = NONE;

float getFictionalY()
{
    if (ROBOT_team_choice == BLUE)
    {
        return (MAX_Y-robotState->y);
    }
    else
    {
        return robotState->y;
    }
}

unsigned int fetchStrat()
{
    for (unsigned int i = 0; i < NB_TASKS; ++i)
    {
        working_weights[i] = tasks_weights[i] + natural_weights[i];
    }

    while (true)
    {
        --working_weights[getPseudoRandomNumber(NB_TASKS)];

        for (unsigned int i = 0; i < NB_TASKS; ++i)
        {
            if (working_weights[i] == 0)
            {
                if (i == 6 && !game_nearly_stopped)
                {
                    working_weights[6] += 10000;
                }
                else
                {
                    return i;
                }
            }
        }
    }
}

void setStrategy()
{
    if (getFictionalY() > 1500)
    {
        natural_weights[2] = 10; // gifts
        natural_weights[1] = 25;
    }
    else
    {
        natural_weights[2] = 25;
        natural_weights[1] = 10;
    }

    natural_weights[0] = 20;  // juices

    natural_weights[3] = 90; // candles
    natural_weights[4] = 55;

    if (game_nearly_stopped)
    {
        natural_weights[5] = 1000000;
        natural_weights[6] = 0;
    }
    else
    {
        natural_weights[6] = 1000000;

        if (walked_dist < 1000)
        {
            natural_weights[5] = 30;
        }
        else if (walked_dist < 2000)
        {
            natural_weights[5] = 20;
        }
        else
        {
            natural_weights[5] = 10;
        }
    }

    if (!initStrategy)
    {
        walked_dist = 0;
        tasks_weights[0] = 0;
        tasks_weights[1] = 0;
        tasks_weights[2] = 0;
        tasks_weights[3] = 0;
        tasks_weights[4] = 0;
        tasks_weights[5] = 0;


        if (robot_strategy == STRAT_1)
        {
            goTask(0);
            goTask(1);
            goTask(3);
            goTask(6);
        }
        else
        {
            goTask(0);
            goTask(4);
            goTask(2);
            goTask(6);
        }
        initStrategy = true;
        currentTask = STARTING_JUICE;
    }
    else
    {
        unsigned int strat = fetchStrat();

        tasks_weights[strat] += 5;

        goTask(strat);

        if (robot_strategy == STRAT_1)
        {
            if (strat != 0)
                goTask(0);

            if (strat != 1)
                goTask(1);

            if (strat != 3)
                goTask(3);

            if (strat != 6)
                goTask(6);
        }
        else
        {
            if (strat != 0)
                goTask(0);

            if (strat != 4)
                goTask(4);

            if (strat != 2)
                goTask(2);

            if (strat != 6)
                goTask(6);
        }
    }

    /*else
    {
        enum Task tmp_t = currentTask;

//        UARTprintf("Switching strategy\n");
        switch (currentTask)
        {
        	case STARTING_JUICE:
                if (precAlert != GIFTS && precAlert != GOING_GIFTS)
                {
                    chooseGiftsTask();
                    setCandlesTask1();
                    setDumpJuiceTask1();
                    setJuiceTask1();
                    setDumpJuiceTask2();
                    currentTask = GOING_GIFTS;
                }
                else
                {
                    setCandlesTask2();
                    setGiftsTask2();
                    setDumpJuiceTask1();
                    setJuiceTask1();
                    setDumpJuiceTask2();
                    currentTask = GOING_CAKE;
                }

        		break;

            case FINISHING_JUICE:
                if (precAlert != CANDLES && precAlert != GOING_CAKE)
                {
                    setCandlesTask2();
                    setGiftsTask2();
                    setDumpJuiceTask1();
                    setJuiceTask1();
                    setDumpJuiceTask2();
                    currentTask = GOING_CAKE;
                }
                else
                {
                    chooseGiftsTask();
                    setCandlesTask2();
                    setDumpJuiceTask1();
                    setJuiceTask1();
                    setDumpJuiceTask2();
                    currentTask = GOING_GIFTS;
                }

        		break;

            case DUMPING_JUICE:
                intel_setNextGoalState(INIT_1_X, JUICE_REVERSE_Y, 42, -20, false, false);
            case DUMPING_REVERSE:
                if (precAlert != GOING_GIFTS && precAlert != GIFTS)
                {
                    if (chooseGiftsTask())
                    {
                        setCandlesTask1();
                    }
                    else
                    {
                        setCandlesTask2();
                    }


                    setDumpJuiceTask2();
                    setJuiceTask1();
                    setDumpJuiceTask1();
                    setDumpJuiceTask2();
                    currentTask = GOING_GIFTS;
                }
                else
                {
                    setCandlesTask2();
                    setGiftsTask2();
                    setDumpJuiceTask1();
                    setJuiceTask1();
                    setDumpJuiceTask2();
                    currentTask = GOING_CAKE;
                }

        		break;

            case GOING_CAKE:
            case CANDLES:
            case CANDLES_FINISHED:
                if (precAlert != GOING_GIFTS && precAlert != GIFTS)
                {
                    chooseGiftsTask();

                    setDumpJuiceTask2();
                    setJuiceTask1();
                    setDumpJuiceTask1();
                    setDumpJuiceTask2();
                    setCandlesTask2();
                    currentTask = GOING_GIFTS;
                }
                else
                {
                    setDumpJuiceTask1();
                    setCandlesTask2();
                    setGiftsTask2();
                    setJuiceTask1();
                    setDumpJuiceTask2();
                    currentTask = STARTING_JUICE;
                }
                break;

            case GOING_GIFTS:
            case GIFTS:
                if (precAlert != STARTING_JUICE
                    && precAlert != FINISHING_JUICE
                    && precAlert != DUMPING_JUICE
                    && precAlert != DUMPING_REVERSE)
                {
                    setJuiceTask1();
                    setDumpJuiceTask1();
                    setCandlesTask2();
                    setGiftsTask2();
                    setDumpJuiceTask2();
                    currentTask = STARTING_JUICE;
                }
                else
                {
                    setCandlesTask2();
                    setGiftsTask2();
                    setDumpJuiceTask2();
                    setJuiceTask1();
                    setDumpJuiceTask1();
                    currentTask = GOING_CAKE;
                }
                break;

            case GIFTS_FINISHED:
                if (precAlert != GOING_CAKE
                    && precAlert != CANDLES
                    && precAlert != CANDLES_FINISHED)
                {
                    setCandlesTask1();
                    setDumpJuiceTask2();
                    setGiftsTask1();
                    setJuiceTask1();
                    setDumpJuiceTask1();
                    currentTask = GOING_CAKE;
                }
                else
                {
                    setJuiceTask1();
                    setDumpJuiceTask1();
                    setCandlesTask2();
                    setGiftsTask2();
                    setDumpJuiceTask2();
                    currentTask = STARTING_JUICE;
                }
                break;


        	default:
                pln2("Warn: State corrupt");
        		break;
        }

        precAlert = tmp_t;
    }*/
}

bool chooseGiftsTask()
{
    if (dist(GIFT_X, GIFT_1_Y) < dist(GIFT_X, GIFT_4_Y))
    {
        setGiftsTask1();
        return true;
    }
    else
    {
        setGiftsTask2();
        return false;
    }
}

void goTask(char sel)
{
    switch (sel)
    {
    	case 0:
            setJuiceTask1();
    		break;

        case 1:
            setGiftsTask1();
            break;

        case 2:
            setGiftsTask2();
            break;

        case 3:
            setCandlesTask1();
            break;

        case 4:
            setCandlesTask2();
            break;

        case 5:
            setDumpJuiceTask1();
            break;

        case 6:
            setDumpFinal();
            break;


    	default:
            pln2("Warn: Unk task");
    		break;
    }

}

void getStatus()
{
    robotState = ctrl_getCurrentState();

    walked_dist += dist2(robotState->x, robotState->y, prec_x, prec_y);
    prec_x = robotState->x;
    prec_y = robotState->y;

    //UARTprintf("X:%d  Y:%d  Th:%d S:%d\n",(int)robotState->x,(int)robotState->y,(int)(robotState->phi * 180.0 / PI), (int)robotState->stop);
}

void doCandleLeft(portTickType* xLastWakeTime)
{
    int delay = 500;
    intel_flapLeftBall(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
    intel_flapLeftUp(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
    intel_flapLeftBall(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
    intel_flapLeftUp(xLastWakeTime);
    intel_restart(xLastWakeTime);
}

void doCandleRight(portTickType* xLastWakeTime)
{
    int delay = 500;
    intel_flapRightBall(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
    intel_flapRightUp(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
    intel_flapRightBall(xLastWakeTime);
    vTaskDelayUntil (xLastWakeTime, (delay / portTICK_RATE_MS));
    intel_flapRightUp(xLastWakeTime);
    intel_restart(xLastWakeTime);
}

void decideActions(portTickType* xLastWakeTime)
{
    /// Recalibrations
    /*if (isAtPos(CANDLE_PREC_1_2_X-50,CANDLE_PREC_1_2_Y,42))
    {
        if (!calibrated)
        {
            vTaskDelayUntil (xLastWakeTime, (250 / portTICK_RATE_MS));
            ctrl_calibx(2000);
            ctrl_calibphi(0);
            pln2("Calibrated");
            calibrated = true;
            ctrl_flush();
            intel_recover();
            ctrl_restart(xLastWakeTime);
        }
    }*/

    checkEnemyAction(xLastWakeTime);

    /// Flaps decision
    /// Zones
    if (dist(CAKE_C_X, CAKE_C_Y) < (CAKE_ZONE_R))
    {
        currentTask = CANDLES;

        if (robotState->phi <= PI-degToRad(5) && robotState->phi >= degToRad(5))
            if (ROBOT_team_choice == BLUE)
                intel_flapLeftUp(xLastWakeTime);
            else
                intel_flapRightUp(xLastWakeTime);
        else if (robotState->phi >= -PI+degToRad(5) && robotState->phi <= -degToRad(5))
            if (ROBOT_team_choice == BLUE)
                intel_flapRightUp(xLastWakeTime);
            else
                intel_flapLeftUp(xLastWakeTime);
        else
        {
            flapLeftDown(xLastWakeTime);
            flapRightDown(xLastWakeTime);
        }
    }
    else if (robotState->x > GIFTS_ZONE_X)
    {
        currentTask = GIFTS;
        intel_flapRightDown(xLastWakeTime);
        intel_flapLeftDown(xLastWakeTime);
    }

    /// Gifts UP
    // STRAT 1
    float giftAngle = PI/2;
    float giftUpOffset = GIFT_Y_DECAL_UP_1;

    if (isAtPos(GIFT_SECURE_FLAP_X,GIFT_SECURE_FLAP_Y,42))
    {
        flapLeftSecure(xLastWakeTime);
        currentTask = GOING_GIFTS;
    }

    if (isAtPosTol(GIFT_X_DECAL,GIFT_1_Y-giftUpOffset,giftAngle, DEF_XY_TOL+80, degToRad(17)))
    {
        intel_flapLeftUp(xLastWakeTime);

        gifts_done[0] = true;
        currentTask = GIFTS;
        precAlert = NONE;
    }

    if (isAtPos(GIFT_X_DECAL,GIFT_2_Y-giftUpOffset,giftAngle))
    {
        intel_flapLeftUp(xLastWakeTime);
        gifts_done[1] = true;
        currentTask = GIFTS;
        precAlert = NONE;
    }

    if (isAtPos(GIFT_X_DECAL,GIFT_3_Y-giftUpOffset,giftAngle))
    {
        intel_flapLeftUp(xLastWakeTime);
        gifts_done[2] = true;
        currentTask = GIFTS;
        precAlert = NONE;
    }

    if (isAtPos(GIFT_X_DECAL,GIFT_4_Y-giftUpOffset,giftAngle))
    {
        intel_flapLeftUp(xLastWakeTime);
        gifts_done[3] = true;
        currentTask = GIFTS_FINISHED;
        precAlert = NONE;
    }

    // STRAT 2
    giftAngle = -PI/2;
    giftUpOffset = GIFT_Y_DECAL_UP_2;

    if (isAtPosTol(GIFT_X_DECAL,GIFT_1_Y-giftUpOffset,giftAngle, DEF_XY_TOL+10, degToRad(25)))
    {
        intel_flapRightUp(xLastWakeTime);
        gifts_done[0] = true;
        currentTask = GIFTS_FINISHED;
        precAlert = NONE;
    }

    if (isAtPos(GIFT_X_DECAL,GIFT_2_Y-giftUpOffset,giftAngle))
    {
        intel_flapRightUp(xLastWakeTime);
        gifts_done[1] = true;
        currentTask = GIFTS;
        precAlert = NONE;
    }

    if (isAtPos(GIFT_X_DECAL,GIFT_3_Y-giftUpOffset,giftAngle))
    {
        intel_flapRightUp(xLastWakeTime);
        gifts_done[2] = true;
        currentTask = GIFTS;
        precAlert = NONE;
    }

    if (isAtPosTol(GIFT_X_DECAL+GIFT_34_1_X_OFF,GIFT_4_Y-giftUpOffset,giftAngle, DEF_XY_TOL+20, degToRad(15)))
    {
        intel_flapRightUp(xLastWakeTime);
        gifts_done[3] = true;
        currentTask = GIFTS;
        precAlert = NONE;
    }

    /// Gifts DOWN
    giftAngle = PI/2;
    float giftDownOffset = GIFT_Y_DECAL_DOWN_1;

    if (    //isAtPosTol(GIFT_X_DECAL,GIFT_1_Y-giftDownOffset+10,giftAngle, DEF_XY_TOL, DEF_TH_TOL)
          isAtPosTol(GIFT_X_DECAL,GIFT_2_Y-giftDownOffset,giftAngle, DEF_XY_TOL, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_3_Y-giftDownOffset,giftAngle, DEF_XY_TOL, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset-20,giftAngle, DEF_XY_TOL, DEF_TH_TOL))
    {
        if (!(ROBOT_team_choice == BLUE && isAtPosTol(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset-20,giftAngle, DEF_XY_TOL, DEF_TH_TOL)))
            intel_flapLeftDown(xLastWakeTime);

        if (isAtPos(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset-10,giftAngle))
            currentTask = GIFTS_FINISHED;
        else
            currentTask = GIFTS;
    }

    giftDownOffset = GIFT_Y_DECAL_DOWN_1-30;

    if (    isAtPosTol(GIFT_X_DECAL,GIFT_1_Y-giftDownOffset+10,giftAngle, DEF_XY_TOL, DEF_TH_TOL+degToRad(5))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_2_Y-giftDownOffset,giftAngle, DEF_XY_TOL+30, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_3_Y-giftDownOffset,giftAngle, DEF_XY_TOL+30, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset-20,giftAngle, DEF_XY_TOL+20, DEF_TH_TOL))
    {
        if (!(ROBOT_team_choice == BLUE && isAtPosTol(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset-20,giftAngle, DEF_XY_TOL+20, DEF_TH_TOL)))
            intel_flapLeftDown(xLastWakeTime);

        if (isAtPos(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset-10,giftAngle))
            currentTask = GIFTS_FINISHED;
        else
            currentTask = GIFTS;
    }

    giftAngle = -PI/2;
    giftDownOffset = GIFT_Y_DECAL_DOWN_2;

    if (    isAtPosTol(GIFT_X_DECAL,GIFT_1_Y-giftDownOffset,giftAngle, DEF_XY_TOL, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_2_Y-giftDownOffset,giftAngle, DEF_XY_TOL, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_3_Y-giftDownOffset,giftAngle, DEF_XY_TOL, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset,giftAngle, DEF_XY_TOL, DEF_TH_TOL+degToRad(10)))
    {
        intel_flapRightDown(xLastWakeTime);

        if (isAtPos(GIFT_X_DECAL,GIFT_1_Y-giftDownOffset,giftAngle))
            currentTask = GIFTS_FINISHED;
        else
            currentTask = GIFTS;
    }

    giftDownOffset = GIFT_Y_DECAL_DOWN_2+30;

    if (    isAtPosTol(GIFT_X_DECAL,GIFT_1_Y-giftDownOffset,giftAngle, DEF_XY_TOL+30, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_2_Y-giftDownOffset,giftAngle, DEF_XY_TOL+30, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_3_Y-giftDownOffset,giftAngle, DEF_XY_TOL+30, DEF_TH_TOL+degToRad(10))
        ||  isAtPosTol(GIFT_X_DECAL,GIFT_4_Y-giftDownOffset,giftAngle, DEF_XY_TOL+30, DEF_TH_TOL+degToRad(10)))
    {
        intel_flapRightDown(xLastWakeTime);

        if (isAtPos(GIFT_X_DECAL,GIFT_1_Y-giftDownOffset,giftAngle))
            currentTask = GIFTS_FINISHED;
        else
            currentTask = GIFTS;
    }

    /// Candles on left
    float c2_off_x = 0;
    float c3_off_x = 0;
    float c4_off_x = 0;
    float c2_off_y = 0;
    float c3_off_y = 0;
    float c4_off_y = 0;

    if (robot_strategy == BLUE)
    {
        c2_off_x = C2_OFF_X;
        c3_off_x = C3_OFF_X;
        c4_off_x = C4_OFF_X;
        c2_off_y = C2_OFF_Y;
        c3_off_y = C3_OFF_Y;
        c4_off_y = C4_OFF_Y;
    }

    if (    robotState->stop
            && (isAtPosTol(CANDLE_1_1_X,CANDLE_1_1_Y,-PI/2,DEF_XY_TOL,PI/2)
            ||  isAtPosTol(CANDLE_2_1_X+c2_off_x,CANDLE_2_1_Y+c2_off_y,-PI/2,DEF_XY_TOL,PI/2)
            ||  isAtPosTol(CANDLE_3_1_X+c3_off_x,CANDLE_3_1_Y+c3_off_y,-PI/2,DEF_XY_TOL,PI/2)
            ||  isAtPosTol(CANDLE_4_1_X+c4_off_x,CANDLE_4_1_Y+c4_off_y,-PI/2,DEF_XY_TOL,PI/2)))
    {
        if (isAtPosTol(CANDLE_1_1_X,CANDLE_1_1_Y,-PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[0])
            {
                doCandleLeft(xLastWakeTime);
                candles_done[0] = true;
                currentTask = CANDLES;
                precAlert = NONE;
            }
        }

        if (isAtPosTol(CANDLE_2_1_X+c2_off_x,CANDLE_2_1_Y+c2_off_y,-PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[1])
            {
                doCandleLeft(xLastWakeTime);
                candles_done[1] = true;
                currentTask = CANDLES;
                precAlert = NONE;
            }
        }

        if (isAtPosTol(CANDLE_3_1_X+c3_off_x,CANDLE_3_1_Y+c3_off_y,-PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[2])
            {
                doCandleLeft(xLastWakeTime);
                candles_done[2] = true;
                currentTask = CANDLES;
                precAlert = NONE;
            }
        }

        if (isAtPosTol(CANDLE_4_1_X+c4_off_x,CANDLE_4_1_Y+c4_off_y,-PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[3])
            {
                doCandleLeft(xLastWakeTime);
                candles_done[3] = true;
                currentTask = CANDLES_FINISHED;
                precAlert = NONE;
            }
        }
    }

    /// Candles on right
    float c1_off_1_x = 0;
    float c2_off_1_x = 0;
    float c3_off_1_x = 0;
    float c4_off_1_x = 0;
    float c1_off_1_y = 0;
    float c2_off_1_y = 0;
    float c3_off_1_y = 0;
    float c4_off_1_y = 0;

    if (robot_strategy == BLUE)
    {
        c1_off_1_x = C1_OFF_1_X;
        c2_off_1_x = C2_OFF_1_X;
        c3_off_1_x = C3_OFF_1_X;
        c4_off_1_x = C4_OFF_1_X;
        c1_off_1_y = C1_OFF_1_Y;
        c2_off_1_y = C2_OFF_1_Y;
        c3_off_1_y = C3_OFF_1_Y;
        c4_off_1_y = C4_OFF_1_Y;
    }

    if (    robotState->stop
            && (isAtPosTol(CANDLE_1_2_X+c1_off_1_x,CANDLE_1_2_Y+c1_off_1_y,PI/2,DEF_XY_TOL,PI/2)
            ||  isAtPosTol(CANDLE_2_2_X+c2_off_1_x,CANDLE_2_2_Y+c2_off_1_y,PI/2,DEF_XY_TOL,PI/2)
            ||  isAtPosTol(CANDLE_3_2_X+c3_off_1_x,CANDLE_3_2_Y+c3_off_1_y,PI/2,DEF_XY_TOL,PI/2)
            ||  isAtPosTol(CANDLE_4_2_X+c4_off_1_x,CANDLE_4_2_Y+c4_off_1_y,PI/2,DEF_XY_TOL,PI/2)))
    {
        if (isAtPosTol(CANDLE_1_2_X+c1_off_1_x,CANDLE_1_2_Y+c1_off_1_y,PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[0])
            {
                doCandleRight(xLastWakeTime);
                candles_done[0] = true;
                currentTask = CANDLES_FINISHED;
                precAlert = NONE;
            }
        }

        if (isAtPosTol(CANDLE_2_2_X+c2_off_1_x,CANDLE_2_2_Y+c2_off_1_y,PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[1])
            {
                doCandleRight(xLastWakeTime);
                candles_done[1] = true;
                currentTask = CANDLES;
                precAlert = NONE;
            }
        }

        if (isAtPosTol(CANDLE_3_2_X+c3_off_1_x,CANDLE_3_2_Y+c3_off_1_y,PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[2])
            {
                doCandleRight(xLastWakeTime);
                candles_done[2] = true;
                currentTask = CANDLES;
                precAlert = NONE;
            }
        }

        if (isAtPosTol(CANDLE_4_2_X+c4_off_1_x,CANDLE_4_2_Y+c4_off_1_y,PI/2,DEF_XY_TOL,PI/2))
        {
            if (!candles_done[3])
            {
                doCandleRight(xLastWakeTime);
                candles_done[3] = true;
                currentTask = CANDLES;
                precAlert = NONE;
            }
        }
    }

    /// Juices
    if (isAtPos(JUICE_1_X, JUICE_1_Y, 42))
    {
        juice_self_done[0] = true;
        currentTask = STARTING_JUICE;
    }


    if (isAtPos(JUICE_2_X, JUICE_2_Y, 42))
    {
        juice_self_done[1] = true;
        currentTask = STARTING_JUICE;
    }


    if (isAtPos(JUICE_3_X, JUICE_3_Y, 42))
    {
        juice_self_done[2] = true;
        currentTask = STARTING_JUICE;
    }


    if (isAtPos(JUICE_4_X, JUICE_4_Y, 42))
    {
        juice_self_done[3] = true;
        currentTask = FINISHING_JUICE;
    }


    if (isAtPos(JUICE_5_X, JUICE_5_Y, 42))
    {
        juice_self_done[4] = true;
        currentTask = FINISHING_JUICE;
    }

    if (isAtPos(JUICE_6_X, JUICE_6_Y, 42))
    {
        juice_self_done[5] = true;
        currentTask = FINISHING_JUICE;
    }

    if (isAtPos(INIT_1_X, JUICE_DUMP_Y, 42))
    {
        juice_self_done[6] = true;
        currentTask = DUMPING_JUICE;
        walked_dist = 0;
    }


    if (isAtPos(INIT_1_X, JUICE_REVERSE_Y, 42))
    {
        juice_self_done[7] = true;
        currentTask = DUMPING_REVERSE;
        precAlert = NONE;
    }

    if (isAtPos(INIT_1_X, JUICE_DUMP_Y, 42) && juice_self_done[7])
    {
        juice_self_done[8] = true;
        currentTask = DUMPING_JUICE;
    }

    if (isAtPos(INIT_1_X, JUICE_REVERSE_Y, 42)&& juice_self_done[8])
    {
        juice_self_done[9] = true;
        currentTask = DUMPING_REVERSE;
        precAlert = NONE;
    }

}

void checkEnemyAction(portTickType* xLastWakeTime)
{
    //UARTprintf("SHARP FW: %d, %d, %d, US F: %d, BL: %d, BR: %d\n", sharpEnemyForwardL, sharpEnemyForwardC, sharpEnemyForwardR, enemyForward, enemyBackwardL, enemyBackwardR);
    if (encounterEscaping || encounterLock)
    {
        float x = robotState->x;
        float y = robotState->y;
        if (x <  TABLE_MARGIN
            || y < TABLE_MARGIN
            || x > MAX_X-TABLE_MARGIN
            || y > MAX_Y-TABLE_MARGIN
            || (dist(CAKE_C_X, CAKE_C_Y) < (CAKE_R + TABLE_MARGIN))
            )
        {
            encounterEscaping = false;
            ctrl_stop(xLastWakeTime);
            stopped_for_enemy = true;
        }
        else
        {
            if (reverse_stopped)
            {
                if (enemyForward >= ENEMY_NEAR)
                {
                    encounterDeadlockR(xLastWakeTime, true);
                }
            }
            else
            {
                if (enemyBackwardL >= ENEMY_NEAR || enemyBackwardR >= ENEMY_NEAR)
                {
                    encounterDeadlock(xLastWakeTime);
                }
            }
        }

    }

    move_direction = ctrl_getForward();
    if ((move_direction == 0 && !encounterEscaping && !encounterLock) || reverse_stopped)
    {
        //UARTprintf("REVERSE MODE\n");
        if (enemyBackwardL >= ENEMY_NEAR || enemyBackwardR >= ENEMY_NEAR)
        {
            if (!startEncounter && !encounterEscaping && !encounterLock)
            {
                startEncounter = true;
                reverse_stopped = true;
                encounterTick = xTaskGetTickCount();
                ctrl_stop(xLastWakeTime);
                wait_for_enemy = true;
            }

            if (wait_for_enemy && !encounterEscaping && !encounterLock)
            {
                ++wait_watchdog;

                if (((xTaskGetTickCount() - encounterTick)/portTICK_RATE_MS > 3000) || wait_watchdog >= INTEL_WDOG_LIMIT)
                {
                    if (wait_watchdog >= INTEL_WDOG_LIMIT)
                        UARTprintf("wdog2\n");

                    wait_watchdog = 0;
                    wait_for_enemy = false;
                    reverse_stopped = true;
                    stopped_for_enemy = true;
                    encounterEscapeR(xLastWakeTime, true);
                }
            }
        }
        else
        {
            if (stopped_for_enemy)
            {
                ctrl_stop(xLastWakeTime);
                intel_flush();
                intel_recoverR();
                intel_restart(xLastWakeTime);
                stopped_for_enemy = false;
                startEncounter = false;
                wait_for_enemy = false;
                encounterEscaping = false;
                reverse_stopped = false;
                wait_watchdog = 0;
            }
            else if (wait_for_enemy)
            {
                intel_restart(xLastWakeTime);
                stopped_for_enemy = false;
                startEncounter = false;
                wait_for_enemy = false;
                encounterEscaping = false;
                reverse_stopped = false;
                wait_watchdog = 0;
            }
        }
    }
    else
    {
        if ((enemyForward == ENEMY_CONTACT) || (sharpEnemyForwardL == ENEMY_CONTACT) || (sharpEnemyForwardR == ENEMY_CONTACT))
        {
            //UARTprintf("ENEMY_CONTACT\n");
            if (!stopped_for_enemy && !encounterEscaping && !encounterLock)
            {
                stopped_for_enemy = true;
                ctrl_stop(xLastWakeTime);
                intel_flush();
                vTaskDelayUntil (xLastWakeTime, (100 / portTICK_RATE_MS));
                encounterEscapeContact(xLastWakeTime, false);
            }


        }
        else if ((enemyForward == ENEMY_NEAR) || (sharpEnemyForwardL == ENEMY_NEAR) || (sharpEnemyForwardR == ENEMY_NEAR))
        {
            //UARTprintf("ENEMY_NEAR\n");
            if (!startEncounter && !encounterEscaping && !encounterLock)
            {
                startEncounter = true;
                encounterTick = xTaskGetTickCount();
                ctrl_stop(xLastWakeTime);
                wait_for_enemy = true;
            }

            if (wait_for_enemy && !encounterEscaping && !encounterLock)
            {
                ++wait_watchdog;

                if (((xTaskGetTickCount() - encounterTick)/portTICK_RATE_MS > 2000) || wait_watchdog >= INTEL_WDOG_LIMIT)
                {
                    if (wait_watchdog >= INTEL_WDOG_LIMIT)
                        UARTprintf("Wdog3\n");

                    wait_watchdog = 0;
                    wait_for_enemy = false;
                    stopped_for_enemy = true;
                    encounterEscape(xLastWakeTime);
                }
            }
        }
        else
        {
            //UARTprintf("NO_ENEMY\n");

            if (stopped_for_enemy)
            {
                ctrl_stop(xLastWakeTime);
                intel_flush();
                intel_recover();
                intel_restart(xLastWakeTime);
                stopped_for_enemy = false;
                startEncounter = false;
                wait_for_enemy = false;
                encounterEscaping = false;
                reverse_stopped = false;
                wait_watchdog = 0;
            }
            else if (wait_for_enemy)
            {
                intel_restart(xLastWakeTime);
                stopped_for_enemy = false;
                startEncounter = false;
                wait_for_enemy = false;
                encounterEscaping = false;
                reverse_stopped = false;
                wait_watchdog = 0;
            }
        }
    }
}

void encounterEscape(portTickType* xLastWakeTime)
{
    encounterEscapeR(xLastWakeTime, false);
}

void encounterEscapeR(portTickType* xLastWakeTime, bool reverse)
{
    intel_flush();
    flapLeftDown(xLastWakeTime);
    flapRightDown(xLastWakeTime);

    float offset = robotState->phi;
//            if (robotState->y < CAKE_C_Y)
//                offset -= PI/4;
//            else
//                offset += PI/4;

    encounterEscaping = true;
//            float x = robotState->x + 3000*custom_cos(offset);
//            float y = robotState->y + 3000*custom_sin(offset);
    if (robotState->phi > 0) {
       if (robotState->x < 777) {
          offset -= PI/2;
       }
       else
          offset += PI/2;
    }
    else {
       if (robotState->x <= 777) {
          offset += PI/2;
       }
       else {
          offset -= PI/2;
       }
    }

    if (offset >= PI)
        offset -= PI;

    if (offset < -PI)
        offset += PI;

    float x = robotState->x + 3000*custom_cos(offset);
    float y = robotState->y + 3000*custom_sin(offset);

    ctrl_setNextGoalState(x,y,42,20,true);
    intel_restart(xLastWakeTime);
    //UARTprintf("\nSide manoeuver\n\n");

    vTaskDelayUntil (xLastWakeTime, (1500 / portTICK_RATE_MS));
    for (int i = 0; i < 60; i++) {
       detectEnemy();
       checkEnemyAction(xLastWakeTime);
       if (enemyForward >= ENEMY_NEAR) {
          break;
       }
       else
          vTaskDelayUntil (xLastWakeTime, (50 / portTICK_RATE_MS));
    }
    getStatus();

    intel_flush();
    intel_recover();
    intel_restart(xLastWakeTime);
    stopped_for_enemy = false;
    startEncounter = false;
    wait_for_enemy = false;
    encounterEscaping = false;
    reverse_stopped = false;
    wait_watchdog = 0;
}

void encounterEscapeContact(portTickType* xLastWakeTime, bool reverse)
{
    intel_flush();

    if (reverse)
    {
        if (enemyForward < ENEMY_NEAR)
        {
            float offset = robotState->phi;
            if (robotState->y < CAKE_C_Y)
                offset -= PI/4;
            else
                offset += PI/4;

            encounterEscaping = true;
            float x = robotState->x + 3000*custom_cos(offset);
            float y = robotState->y + 3000*custom_sin(offset);

            ctrl_setNextGoalState(x,y,42,20,true);
            intel_restart(xLastWakeTime);
        }
        else
        {
            encounterDeadlockR(xLastWakeTime, reverse);
        }
    }
    else
    {
        if (enemyBackwardL < ENEMY_NEAR || enemyBackwardR < ENEMY_NEAR)
        {
            float offset = -robotState->phi;
            if (enemyBackwardL >= ENEMY_NEAR)
            {
                // Going right instead
                offset += PI/4;
            }
            else if (enemyBackwardR >= ENEMY_NEAR)
            {
                // Going left
                offset -= PI/4;
            }
            else
            {
                if (robotState->y < CAKE_C_Y)
                    offset += PI/4;
                else
                    offset -= PI/4;
            }

            encounterEscaping = true;
            float x = robotState->x + 3000*custom_cos(offset);
            float y = robotState->y + 3000*custom_sin(offset);

            ctrl_setNextGoalState(x,y,42,-20,true);
            intel_restart(xLastWakeTime);
        }
        else
        {
            encounterDeadlockR(xLastWakeTime, reverse);
        }
    }
}

void encounterDeadlock(portTickType* xLastWakeTime)
{
    encounterDeadlockR(xLastWakeTime, false);
}

void encounterDeadlockR(portTickType* xLastWakeTime, bool reverse)
{
    if (!encounterLock)
    {
        encounterLock = true;

        intel_flush();

        float offset;
        if (reverse)
        {
            offset = robotState->phi;
            if (enemyBackwardL >= ENEMY_NEAR)
            {
                offset -= PI/4;
            }
            if (enemyBackwardR >= ENEMY_NEAR)
            {
                offset += PI/4;
            }
        }
        else
        {
            offset = -robotState->phi;
            if (enemyBackwardL >= ENEMY_NEAR)
            {
                offset += PI/4;
            }
            if (enemyBackwardR >= ENEMY_NEAR)
            {
                offset -= PI/4;
            }
        }


        float x = robotState->x + 200*custom_cos(offset);
        float y = robotState->y + 200*custom_sin(offset);

        ctrl_setNextGoalState(x,y,42,1,false);
    }
}

void intel_recoverR()
{
    intel_setNextGoalState(INIT_1_X, JUICE_REVERSE_Y, 42, -20, false, false);
    setStrategy();
}

void intel_recover()
{
    setStrategy();
}

bool isAtPos(float x, float y, float th)
{
    return isAtPosTol(x,y,th,DEF_XY_TOL,DEF_TH_TOL);
}

bool isAtPosTol(float x, float y, float th, float xyTol, float thTol)
{
    float th2;
    float y2;

    if (ROBOT_team_choice == BLUE)
    {
        y2 = MAX_Y - y;

        th2 = th + PI;
        if (th2 > PI)
            th2 -= 2*PI;
    }
    else
    {
        y2 = y;
        th2 = th;
    }

    if (custom_abs(robotState->x - x) < xyTol
        && custom_abs(robotState->y - y2) < xyTol)
    {
        if (th == 42 || custom_abs(robotState->phi - th2) < thTol)
        {
            return true;
        }
    }

    return false;
}

float dist(float x, float y)
{
    return dist2(x,y,robotState->x,robotState->y);
}

float dist2(float x1, float y1, float x2, float y2)
{
    float a = x1 - x2;
    float b = y1 - y2;
    return custom_sqrt(a*a+b*b);
}

/// Flaps inversion
void intel_flapLeftUp(portTickType* xLastWakeTime)
{
    if (ROBOT_team_choice == RED)
        flapLeftUp(xLastWakeTime);
    else
        flapRightUp(xLastWakeTime);
}

void intel_flapLeftBall(portTickType* xLastWakeTime)
{
    if (ROBOT_team_choice == RED)
        flapLeftBall(xLastWakeTime);
    else
        flapRightBall(xLastWakeTime);
}

void intel_flapLeftDown(portTickType* xLastWakeTime)
{
    if (ROBOT_team_choice == RED)
        flapLeftDown(xLastWakeTime);
    else
        flapRightDown(xLastWakeTime);
}

void intel_flapRightUp(portTickType* xLastWakeTime)
{
    if (ROBOT_team_choice == RED)
        flapRightUp(xLastWakeTime);
    else
        flapLeftUp(xLastWakeTime);
}

void intel_flapRightBall(portTickType* xLastWakeTime)
{
    if (ROBOT_team_choice == RED)
        flapRightBall(xLastWakeTime);
    else
        flapLeftBall(xLastWakeTime);
}

void intel_flapRightDown(portTickType* xLastWakeTime)
{
    if (ROBOT_team_choice == RED)
        flapRightDown(xLastWakeTime);
    else
        flapLeftDown(xLastWakeTime);
}

/// Control inversion
static unsigned int pts_done = 0;
static Point splineRefs[4];
static bool precReverse = false;
static float precPhi = 0;
static float precK = 0;
static bool precStop = false;
static bool precSpline = false;

#define POINTS_PER_SPLINE 25
static Point splinePts[POINTS_PER_SPLINE];

void intel_flush()
{
    pts_done = 0;
    ctrl_flush();
}

void intel_initControl(float x, float y, float phi)
{
    float phi2;
    float y2;

    if (ROBOT_team_choice == BLUE)
    {
        y2 = MAX_Y - y;

        if (phi == 42)
        {
            phi2 = 42;
        }
        else
        {
            phi2 = phi + PI;
            if (phi2 > PI)
                phi2 -= 2*PI;
        }
    }
    else
    {
        y2 = y;
        phi2 = phi;
    }

    ctrl_initControl(x, y2, phi2);
    pts_done = 0;
}

void intel_setNextGoalState(float x, float y, float phi, float k, bool stop, bool spline)
{
    float phi2;
    float y2;

    if (ROBOT_team_choice == BLUE)
    {
        y2 = MAX_Y - y;

        if (phi == 42)
        {
            phi2 = 42;
        }
        else
        {
            phi2 = phi + PI;
            if (phi2 > PI)
                phi2 -= 2*PI;
        }
    }
    else
    {
        y2 = y;
        phi2 = phi;
    }

    /// Spline
    if (pts_done == 0)
    {
        splineRefs[0].x = robotState->x;
        splineRefs[0].y = robotState->y;
        splineRefs[1].x = robotState->x;
        splineRefs[1].y = robotState->y;

        splineRefs[2].x = x;
        splineRefs[2].y = y2;
        precPhi = phi;
        precK = k;
        precStop = stop;
        precSpline = spline;
        precReverse = k < 0;

        ++pts_done;
    }
    else
    {
        splineRefs[3].x = x;
        splineRefs[3].y = y2;

        if (precSpline)
        {
            doSpline();
        }
        else
        {
            ctrl_setNextGoalState(splineRefs[2].x, splineRefs[2].y, precPhi, precK, precStop);

            splineRefs[0].x = splineRefs[1].x;
            splineRefs[0].y = splineRefs[1].y;
            splineRefs[1].x = splineRefs[2].x;
            splineRefs[1].y = splineRefs[2].y;
            splineRefs[2].x = splineRefs[3].x;
            splineRefs[2].y = splineRefs[3].y;
        }

        precPhi = phi;
        precK = k;
        precStop = stop;
        precSpline = spline;
        precReverse = k < 0;

        ++pts_done;
    }
}

void doSpline()
{
    unsigned int nbPts = (unsigned int) (dist2(splineRefs[1].x, splineRefs[1].y, splineRefs[2].x, splineRefs[2].y) / 15.0);

    if (nbPts > POINTS_PER_SPLINE)
        nbPts = POINTS_PER_SPLINE;
    else if (nbPts < 3)
        nbPts = 3;

    Point* refs = splineRefs;
    Point* pts = splinePts;
    
    genSpline(refs,pts,0.0,nbPts);

    unsigned int i = 0;

    float ksign = (precReverse) ? -1 : 1;


    if (pts_done == 1)
    {
        ctrl_setNextGoalState(pts[0].x, pts[0].y, 42, ksign*15, false);
        i = 1;
    }

    float k = ksign*15;
    for (; i < (nbPts-1); ++i)
    {
        ctrl_setNextGoalState(pts[i].x, pts[i].y, 42, k, false);
    }

    ctrl_setNextGoalState(pts[nbPts-1].x, pts[nbPts-1].y, 42, k, precStop);

    splineRefs[0].x = splineRefs[1].x;
    splineRefs[0].y = splineRefs[1].y;
    splineRefs[1].x = splineRefs[2].x;
    splineRefs[1].y = splineRefs[2].y;
    splineRefs[2].x = splineRefs[3].x;
    splineRefs[2].y = splineRefs[3].y;
}

void endSpline()
{
    splineRefs[3].x = splineRefs[2].x;
    splineRefs[3].y = splineRefs[2].y;

    if (precSpline)
    {
        doSpline();
    }
    else
    {
        ctrl_setNextGoalState(splineRefs[2].x, splineRefs[2].y, precPhi, precK, precStop);

        splineRefs[0].x = splineRefs[1].x;
        splineRefs[0].y = splineRefs[1].y;
        splineRefs[1].x = splineRefs[2].x;
        splineRefs[1].y = splineRefs[2].y;
        splineRefs[2].x = splineRefs[3].x;
        splineRefs[2].y = splineRefs[3].y;
    }

    pts_done = 0;
}

void intel_restart(portTickType* xLastWakeTime)
{
    endSpline();

    unsigned int i = 0;

    while (!ctrl_restart(xLastWakeTime) && i < WDOG_LIMIT)
    {
        if (i >= WDOG_LIMIT)
            UARTprintf("wdog4\n");

        ++i;
//        UARTprintf("Waiting for restart\n");
        vTaskDelayUntil (xLastWakeTime, (100 / portTICK_RATE_MS));
    }

    if (!(i < WDOG_LIMIT))
    {
        UARTprintf("wdog5\n");

        ctrl_stop(xLastWakeTime);
        intel_flush();
        intel_recover();
        intel_restart(xLastWakeTime);
        stopped_for_enemy = false;
        startEncounter = false;
        wait_for_enemy = false;
        encounterEscaping = false;
        reverse_stopped = false;
        wait_watchdog = 0;
    }
}

void initTasks()
{
    for (unsigned int i = 0; i < 4; ++i)
        gifts_done[i] = false;

    for (unsigned int i = 0; i < 4; ++i)
        balls_done[i] = false;

    for (unsigned int i = 0; i < 9; ++i)
        juice_self_done[i] = false;

    for (unsigned int i = 0; i < 4; ++i)
        juice_enemy_done[i] = false;
}

void setJuiceTask1()
{
    bool dojuice = false;
    if (!juice_self_done[4])
    {
        intel_setNextGoalState(JUICE_5_X, JUICE_5_Y, PI/2, 20, false, false);
        dojuice = true;
    }

    if (!juice_self_done[3])
    {
        intel_setNextGoalState(JUICE_4_X, JUICE_4_Y, 42, 130, false, false);
        dojuice = true;
    }

    if (!juice_self_done[2])
    {
        intel_setNextGoalState(JUICE_3_X, JUICE_3_Y, 0, 130, false, false);
        dojuice = true;
    }

    if (!juice_self_done[1])
    {
        intel_setNextGoalState(JUICE_2_X, JUICE_2_Y, 42, 130, false, false);
        dojuice = true;
    }

    if (!juice_self_done[0])
    {
        intel_setNextGoalState(JUICE_1_X, JUICE_1_Y, 42, 130, false, false);
        dojuice = true;
    }

    if (dojuice)
        setDumpJuiceTask1();
}

void setJuiceTask2()
{
    /*if (!juice_self_done[0])
        intel_setNextGoalState(JUICE_1_X, JUICE_1_Y, PI/2, 20, false, true);

    if (!juice_self_done[1])
        intel_setNextGoalState(JUICE_2_X, JUICE_2_Y, 42, 130, false, true);

    if (!juice_self_done[2])
        intel_setNextGoalState(JUICE_3_X, JUICE_3_Y, PI, 130, false, true);

    if (!juice_self_done[3])
        intel_setNextGoalState(JUICE_4_X, JUICE_4_Y, 42, 130, false, true);

    if (!juice_self_done[4])
        intel_setNextGoalState(JUICE_5_X, JUICE_5_Y, -PI/2, 130, false, true);*/

    setJuiceTask1();
}

void setDumpJuiceTask1()
{
//    if (!juice_self_done[6])
//    {
        intel_setNextGoalState(INIT_1_X, JUICE_DUMP_PREC_Y, 42, 20, false, false);
        intel_setNextGoalState(INIT_1_X, JUICE_DUMP_Y, 42, 20, false, false);
//    }
//    if (!juice_self_done[7])
//    {
        intel_setNextGoalState(INIT_1_X, JUICE_REVERSE_Y, 42, -20, false, false);
//    }
}

void setDumpFinal()
{
    intel_setNextGoalState(INIT_1_X, JUICE_DUMP_PREC_Y, 42, 20, false, false);
    intel_setNextGoalState(INIT_1_X, JUICE_DUMP_Y, 42, 20, true, false);
}

void setDumpJuiceTask2()
{
    if (!juice_self_done[8])
    {
        intel_setNextGoalState(INIT_1_X, JUICE_DUMP_PREC_Y, 42, 20, false, false);
        intel_setNextGoalState(INIT_1_X, JUICE_DUMP_Y, 42, 20, false, false);
    }
    if (!juice_self_done[9])
    {
        intel_setNextGoalState(INIT_1_X, JUICE_REVERSE_Y, 42, -20, false, false);
    }
}

void setParkTask1()
{
    /*intel_setNextGoalState(INIT_1_X, JUICE_DUMP_PREC_Y, 42, 20, false, false);
    intel_setNextGoalState(INIT_1_X, JUICE_DUMP_Y, 42, 20, false, false);
    intel_setNextGoalState(INIT_1_X, JUICE_REVERSE_Y, 42, -20, false, false);*/
    //intel_setNextGoalState(INIT_1_X, INIT_1_Y+500, PI/2, 20, false, false);
    //intel_setNextGoalState(INIT_1_X, INIT_1_Y, 42, -20, true, false);
}

void setParkTask2()
{
    setParkTask1();
}

void setGiftsTask1()
{
    if (!gifts_done[0])
    {
        intel_setNextGoalState(GIFT_SECURE_FLAP_X, GIFT_SECURE_FLAP_Y, 42, 20, false, false);
        intel_setNextGoalState(GIFT_PREC_X, GIFT_PREC_Y, 42, 20, false, false);
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_1_Y-GIFT_Y_DECAL_UP_1+20, 42, 20, false, false);
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_1_Y-GIFT_Y_DECAL_DOWN_1, 42, 20, false, false);
    }

    if (!gifts_done[1])
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_2_Y-GIFT_Y_DECAL_DOWN_1, PI/2, 20, false, false);

    if (!gifts_done[2])
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_3_Y-GIFT_Y_DECAL_DOWN_1, PI/2, 20, false, false);

    if (!gifts_done[3])
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_4_Y-GIFT_Y_DECAL_DOWN_1, PI/2, 20, false, false);
}

void setGiftsTask2()
{
    if (!gifts_done[3])
    {
        intel_setNextGoalState(GIFT_X_DECAL+GIFT_34_1_X_OFF, GIFT_4_Y-GIFT_Y_DECAL_UP_2+45, 42, 20, false, false);
        intel_setNextGoalState(GIFT_X_DECAL+GIFT_34_1_X_OFF, GIFT_4_Y-GIFT_Y_DECAL_DOWN_2, 42, 20, false, false);
    }

    if (!gifts_done[2])
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_3_Y-GIFT_Y_DECAL_DOWN_2, -PI/2, 20, false, false);

    if (!gifts_done[1])
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_2_Y-GIFT_Y_DECAL_DOWN_2, -PI/2, 20, false, false);

    if (!gifts_done[0])
        intel_setNextGoalState(GIFT_X_DECAL, GIFT_1_Y-GIFT_Y_DECAL_DOWN_2, -PI/2, 20, false, false);
}

void setCandlesTask1()
{
    float offset = 0;
    if (robot_strategy == BLUE)
    {
        offset = -5;
    }



    float c2_off_x = 0;
    float c3_off_x = 0;
    float c4_off_x = 0;
    float c2_off_y = 0;
    float c3_off_y = 0;
    float c4_off_y = 0;

    if (robot_strategy == BLUE)
    {
        c2_off_x = C2_OFF_X;
        c3_off_x = C3_OFF_X;
        c4_off_x = C4_OFF_X;
        c2_off_y = C2_OFF_Y;
        c3_off_y = C3_OFF_Y;
        c4_off_y = C4_OFF_Y;
    }

    if (!candles_done[0])
    {
        intel_setNextGoalState(CANDLE_PREC_1_1_X, CANDLE_PREC_1_1_Y, 42, 70, false, false);
        intel_setNextGoalState(CANDLE_PREC_2_1_X, CANDLE_PREC_2_1_Y, 42, 70, false, true);
        intel_setNextGoalState(CANDLE_4_1_X+offset+c4_off_x, CANDLE_4_1_Y+c4_off_y, CANDLE_4_1_TH, 20, !candles_done[3], true);

        intel_setNextGoalState(CANDLE_3_1_X+offset+c3_off_x, CANDLE_3_1_Y+c3_off_y, 42, 70, !candles_done[2], true);
        intel_setNextGoalState(CANDLE_2_1_X+offset+c2_off_x, CANDLE_2_1_Y+c2_off_y, 42, 70, !candles_done[1], true);
        intel_setNextGoalState(CANDLE_1_1_X+offset, CANDLE_1_1_Y, 42, 70, !candles_done[0], true);
        intel_setNextGoalState(CANDLE_PREC_2_2_X, CANDLE_PREC_2_2_Y, 42, 70, false, true);
        intel_setNextGoalState(CANDLE_PREC_1_2_X, CANDLE_PREC_1_2_Y, 42, 70, false, true);
    }
}

void setCandlesTask2()
{
    float c1_off_1_x = 0;
    float c2_off_1_x = 0;
    float c3_off_1_x = 0;
    float c4_off_1_x = 0;
    float c1_off_1_y = 0;
    float c2_off_1_y = 0;
    float c3_off_1_y = 0;
    float c4_off_1_y = 0;

    if (robot_strategy == BLUE)
    {
        c1_off_1_x = C1_OFF_1_X;
        c2_off_1_x = C2_OFF_1_X;
        c3_off_1_x = C3_OFF_1_X;
        c4_off_1_x = C4_OFF_1_X;
        c1_off_1_y = C1_OFF_1_Y;
        c2_off_1_y = C2_OFF_1_Y;
        c3_off_1_y = C3_OFF_1_Y;
        c4_off_1_y = C4_OFF_1_Y;
    }

    if (!candles_done[3])
    {
        intel_setNextGoalState(CANDLE_PREC_1_2_X, CANDLE_PREC_1_2_Y, 42, 70, false, true);
        intel_setNextGoalState(CANDLE_PREC_2_2_X, CANDLE_PREC_2_2_Y, 42, 70, false, true);
        intel_setNextGoalState(CANDLE_1_2_X+c1_off_1_x, CANDLE_1_2_Y+c1_off_1_y, CANDLE_1_2_TH, 70, !candles_done[0], true);

        intel_setNextGoalState(CANDLE_2_2_X+c2_off_1_x, CANDLE_2_2_Y+c2_off_1_y, 42, 70, !candles_done[1], true);
        intel_setNextGoalState(CANDLE_3_2_X+c3_off_1_x, CANDLE_3_2_Y+c3_off_1_y, 42, 70, !candles_done[2], true);
        intel_setNextGoalState(CANDLE_4_2_X+c4_off_1_x, CANDLE_4_2_Y+c4_off_1_y, 42, 70, !candles_done[3], true);
//        intel_setNextGoalState(CANDLE_PREC_2_2_X, CANDLE_PREC_2_2_Y, 42, 70, false, true);
//        intel_setNextGoalState(CANDLE_PREC_1_2_X, CANDLE_PREC_1_2_Y, 42, 70, false, true);
    }
}


void detectEnemy()
{
    usDetect();
    sharpDetect();
    refreshEnemyInfo();
    getSharpAdditionalInfo();
}

void usDetect()
{
    xSemaphoreTake(usBufSwitchMutex, portMAX_DELAY);

    if (usBufSwitch)
    {
        for (unsigned int i = 0; i < 4; ++i)
            intelUSBuf[i] = ultraValsBuf1[i];
    }
    else
    {
        for (unsigned int i = 0; i < 4; ++i)
            intelUSBuf[i] = ultraValsBuf2[i];
    }

    xSemaphoreGive(usBufSwitchMutex);

    for (unsigned int i = 0; i < 4; ++i)
        usDistance[i] = ultrason_convert(intelUSBuf[i]);

    /// US debug output
    //UARTprintf("usDistance [0] = %d \t \t", (int) (usDistance[0]));


    if (usDistance[0] < CONTACT_DIST)
    {
        usEnemyForward = ENEMY_CONTACT;
    }
    else if (usDistance[0] < NEAR_DIST)
    {
        usEnemyForward = ENEMY_NEAR;
    }
    else if (usDistance[0] < SIGHT_DIST)
    {
        usEnemyForward = ENEMY_IN_SIGHT;
    }
    else {
        usEnemyForward = ENEMY_NOT_AROUND;
    }

    //UARTprintf("intelUSBuf[1] = %d   \t [2] = %d   \t [3] = %d\n", (int) intelUSBuf[1], (int) intelUSBuf[2], (int) intelUSBuf[3]);


   enemyBackwardL = ENEMY_NOT_AROUND;
   enemyBackwardR = ENEMY_NOT_AROUND;
   if (intelUSBuf[3] < 60) {
      if (bwlcounter > 3){
         //bwlcounter = 0;
         enemyBackwardL = ENEMY_NEAR;
//         UARTprintf("ENEMY ON THE LEFT!\n");
         if (intelUSBuf[3] < 50) {
            enemyBackwardL = ENEMY_CONTACT;
//            UARTprintf("CONTACT ON THE LEFT!\n");
         }
      }
      else {
         bwlcounter++;
      }
   }
   else {
       bwlcounter = 0;
   }
   if (intelUSBuf[2] < 60) {
      if (bwrcounter > 3) {
//         bwrcounter = 0;
         enemyBackwardR = ENEMY_NEAR;
//         UARTprintf("ENEMY ON THE RIGHT!\n");
         if (intelUSBuf[2] < 50) {
            enemyBackwardR = ENEMY_CONTACT;
//            UARTprintf("CONTACT ON THE RIGHT!\n");
         }
      }
      else {
         bwrcounter++;
      }
   }
   else {
       bwrcounter = 0;
   }
}

void sharpDetect()
{
    xSemaphoreTake(sharpBufSwitchMutex, portMAX_DELAY);

    if (sharpBufSwitchMutex)
    {
        for (unsigned int i = 0; i < 4; ++i)
            intelSharpBuf[i] = sharpValsBuf1[i];
    }
    else
    {
        for (unsigned int i = 0; i < 4; ++i)
            intelSharpBuf[i] = sharpValsBuf2[i];
    }

    xSemaphoreGive(sharpBufSwitchMutex);

    for (unsigned int i = 0; i < 4; ++i)
        sharpDistance[i] = sharp_convert(intelSharpBuf[i]);

    /// Sharp debug output
    //UARTprintf("SHARPDistance [center] = %d \t [left] = %d\t [right] = %d\n", (int) (sharpDistance[0]), (int) (sharpDistance[1]), (int) (sharpDistance[2]));

    if (sharpDistance[0] < 10){
      sharpEnemyForwardC = ENEMY_CONTACT;
    }
    else if (sharpDistance[0] < 20){
      sharpEnemyForwardC = ENEMY_NEAR;
    }
    else if (sharpDistance[0] < 40){
      sharpEnemyForwardC = ENEMY_IN_SIGHT;
    }
    else {
      sharpEnemyForwardC = ENEMY_NOT_AROUND;
    }

    if (sharpDistance[1] < 10){
      sharpEnemyForwardL = ENEMY_CONTACT;
    }
    else if (sharpDistance[1] < 20){
      sharpEnemyForwardL = ENEMY_NEAR;
    }
    else if (sharpDistance[1] < 40){
      sharpEnemyForwardL = ENEMY_IN_SIGHT;
    }
    else {
      sharpEnemyForwardL = ENEMY_NOT_AROUND;
    }

    if (sharpDistance[2] < 10){
      sharpEnemyForwardR = ENEMY_CONTACT;
    }
    else if (sharpDistance[2] < 20){
      sharpEnemyForwardR = ENEMY_NEAR;
    }
    else if (sharpDistance[2] < 40){
      sharpEnemyForwardR = ENEMY_IN_SIGHT;
    }
    else {
      sharpEnemyForwardR = ENEMY_NOT_AROUND;
    }
}


void refreshEnemyInfo()
{
   if (usEnemyForward == ENEMY_CONTACT && FWcontactcounter < 5) {
      FWcontactcounter++;
      //FWnearcounter = 0;
      FWsightcounter = 0;
   }
   else if (usEnemyForward == ENEMY_CONTACT && FWcontactcounter == 5) {
      FWnearcounter = 0;
      FWcontactcounter = 0;
      FWsightcounter = 0;
      extra_info = 0;
//      enemyForward = UNDECIDED;
      if ((sharpEnemyForwardC == ENEMY_CONTACT) || (sharpEnemyForwardR == ENEMY_CONTACT)) {
         extra_info ^= 0x01;
      }
      if ((sharpEnemyForwardC == ENEMY_CONTACT) || (sharpEnemyForwardL == ENEMY_CONTACT)) {
         extra_info ^= 0x02;
      }
      if (!outOfBoundaries(CONTACT_DIST)) {
         enemyForward = ENEMY_CONTACT;
      }
      //Puts enemyForward to ENEMY_CONTACT and sets extra_info to 0 if SHARPS are throwing shit, 1 if rightie did say something smart, 2 if that was the leftie, 3 if both or center.
   }
   else if (usEnemyForward == ENEMY_NEAR && FWnearcounter < 5) {
      //FWcontactcounter = 0;
      FWnearcounter++;
      //FWsightcounter = 0;
   }
   else if (usEnemyForward == ENEMY_NEAR && FWnearcounter == 5) {
      FWcontactcounter = 0;
      FWnearcounter = 0;
      FWsightcounter = 0;
      extra_info = 0;

      if ((sharpEnemyForwardC == ENEMY_CONTACT) || (sharpEnemyForwardR == ENEMY_CONTACT) || (sharpEnemyForwardC == ENEMY_NEAR) || (sharpEnemyForwardR == ENEMY_NEAR)) {
         extra_info ^= 0x01;
      }
      if ((sharpEnemyForwardC == ENEMY_CONTACT) || (sharpEnemyForwardL == ENEMY_CONTACT) || (sharpEnemyForwardC == ENEMY_NEAR) || (sharpEnemyForwardL == ENEMY_NEAR)) {
         extra_info ^= 0x02;
      }
      if (!outOfBoundaries(NEAR_DIST)) {
         enemyForward = ENEMY_NEAR;
      }
      //Puts enemyForward to ENEMY_NEAR and sets extra_info to 0 if SHARPS are throwing shit, 1 if rightie did say something smart, 2 if that was the leftie, 3 if both or center.
   }
   else if ((usEnemyForward == ENEMY_IN_SIGHT || usEnemyForward == ENEMY_NOT_AROUND) && (FWsightcounter < 5)) {
      FWcontactcounter = 0;
      //FWnearcounter = 0;
      FWsightcounter++;
   }
   else {
      FWnearcounter = 0;
      FWsightcounter = 0;
      enemyForward = ENEMY_NOT_AROUND;
      if (usEnemyForward == ENEMY_IN_SIGHT) {
         enemyForward = ENEMY_IN_SIGHT;
      }
   }
}

/**
  * Sets sharpEnemyForwardL/R to ENEMY_IN_SIGHT when something is being seen but it ain't sure.
  * Sets sharpEnemyForwardL/R to ENEMY_NEAR or ENEMY_CONTACT when there is something according to the SHARP.
  * Sets sharpEnemyForwardL/R to ENEMY_NOT_AROUND the SHARP sees nothing.
 **/
void getSharpAdditionalInfo() {

   /*float u1 = 2000.0 - robotState->x;
   float u2 = 1500.0 - robotState->y;

   float phi_cake = 0.0;
   if (custom_abs(u1) < 0.1) {
      if (u1*u2 >= 0.0) {
         phi_cake = PI/2;
      }
      else {
         phi_cake = -PI/2;
      }
   }
   else {
      phi_cake = custom_atan(u2/u1);
   }
//   UARTprintf("phi_cake_raw: %d millirad\n", (int) (phi_cake * 1000.0));

   if ((phi_cake > 0.0) && (u2 < 0.0)) {
      phi_cake = phi_cake - PI;
   }
   else if ((phi_cake < 0.0) && (u2 > 0.0)) {
      phi_cake = PI + phi_cake;
   }
   else if ((phi_cake == 0) && (u1 < 0.0)) {
      phi_cake = PI;
   }

   if ((custom_abs(phi_cake - robotState->phi) > PI/4) && !outOfBoundaries(30)) {
      if (((sharpEnemyForwardL == ENEMY_NEAR) || sharpEnemyForwardL == ENEMY_CONTACT) && (sharpLcounter < 7)) {
         sharpLcounter++;
         sharpEnemyForwardL = ENEMY_IN_SIGHT;
//         UARTprintf(("sharp LEFT = IN SIGHT  \t"));
      }
      else if ((sharpEnemyForwardL == ENEMY_NEAR) || sharpEnemyForwardL == ENEMY_CONTACT) {
            if (sharpEnemyForwardL == ENEMY_NEAR) {
//               UARTprintf(("sharp LEFT = NEAR  \t"));
            }
            else {
//               UARTprintf(("sharp LEFT = CONTACT  \t"));
            }
      }
      else {
         sharpLcounter = 0;
         sharpEnemyForwardL = ENEMY_NOT_AROUND;
//         UARTprintf("sharp LEFT = NOT AROUND   \t");
      }
      if (((sharpEnemyForwardR == ENEMY_NEAR) || sharpEnemyForwardR == ENEMY_CONTACT) && (sharpRcounter < 7)) {
         sharpRcounter++;
         sharpEnemyForwardR = ENEMY_IN_SIGHT;
//         UARTprintf("sharp RIGHT = IN SIGHT   \n");
      }
      else if ((sharpEnemyForwardR == ENEMY_NEAR) || sharpEnemyForwardR == ENEMY_CONTACT) {
            if (sharpEnemyForwardR == ENEMY_NEAR) {
//               UARTprintf(("sharp RIGHT = NEAR  \n"));
            }
            else {
//               UARTprintf(("sharp RIGHT = CONTACT  \n"));
            }
      }
      else {
         sharpRcounter = 0;
         sharpEnemyForwardR = ENEMY_NOT_AROUND;
//         UARTprintf("sharp RIGHT = NOT AROUND   \n");
      }
   }
   else {*/
      sharpEnemyForwardR = ENEMY_NOT_AROUND;
      sharpEnemyForwardL = ENEMY_NOT_AROUND;
      sharpEnemyForwardC = ENEMY_NOT_AROUND;
//      UARTprintf("sharp LEFT = NOT AROUND   \t");
//      UARTprintf("sharp RIGHT = NOT AROUND  (CAKE ON) \n");
   //}
}


bool outOfBoundaries(int distance) {
   //UARTprintf("in borders stub\n");
   float pointx = robotState->x + 10*distance*custom_cos(robotState->phi);
   float pointy = robotState->y + 10*distance*custom_sin(robotState->phi);
   //UARTprintf("in borders stub, %d (%d, %d) -> (%d, %d)\n", distance, (int) robotState->x, (int) robotState->y, (int) pointx, (int) pointy);
   float delta = 150.0; // 10 cm
   if ((pointx > (2000.0 - delta)) || (pointx < delta) || (pointy > (3000.0 - delta)) || (pointy < delta)) {
      //UARTprintf("out of borders\n");
      return true;
   }
   float distCake = custom_sqrt((2000.0 - pointx)*(2000.0 - pointx) + (1500.0 - pointy)*(1500.0 - pointy));
   if (distCake < (500.0 + delta)){
      //UARTprintf("that is the cake\n");
      return true;
   }
   return false;
}

