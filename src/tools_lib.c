#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "tools_lib.h"

#include "custom_lib.h"
#include "custom_math.h"

char servoParam[SERVO_BUFSIZ];
char flapParam[SERVO_BUFSIZ];

char servoBufferTx[SERVO_BUFSIZ];
char flapBufferTx[SERVO_BUFSIZ];

char servoBufferRx[SERVO_BUFSIZ];
char flapBufferRx[SERVO_BUFSIZ];

char* servoRxNbParams = servoBufferRx + 3;
char* flapRxNbParams = flapBufferRx + 3;

char* servoRxStatus = servoBufferRx + 4;
char* flapRxStatus = flapBufferRx + 4;

char* servoRxParams = servoBufferRx + 5;
char* flapRxParams = flapBufferRx + 5;

#define UART_RX_MS_WAIT 150
static unsigned long rx_servo_ms_wait = UART_RX_MS_WAIT; // Static here to avoid stack overflows
static unsigned long rx_flap_ms_wait = UART_RX_MS_WAIT; // Static here to avoid stack overflows

void servoLEDWrite()
{

    //char params[2];
    // ALLUMER LES LEDS
    servoParam[0] = 0x19;
    servoParam[1] = 0x01;

    servoCmdRAW(0, INST_WRITE, 2, SERVO_UART, SERVO_CMD_PIN_BASE, SERVO_CMD_PIN_NB, servoParam, servoBufferTx);

    servoCmdRAW(1, INST_WRITE, 2, SERVO_UART, SERVO_CMD_PIN_BASE, SERVO_CMD_PIN_NB, servoParam, servoBufferTx);

    // CONTROLE EN POSITION
    servoParam[0] = 0x1E;
    servoParam[1] = 0x00;
    servoParam[2] = 0x02;
    servoParam[3] = 0x00;
    servoParam[4] = 0x02;

    servoCmdRAW(1, INST_WRITE, 5, SERVO_UART, SERVO_CMD_PIN_BASE, SERVO_CMD_PIN_NB, servoParam, servoBufferTx);

}

void servoCmdRAW(char ID, char instruction, char paramLength,
                  unsigned long base, unsigned long ctrl_pin_base, unsigned long ctrl_pin_nb,
                  char* param, char* bufferTx)
{
    char packetLength = paramLength + 4 + 2;

    if (packetLength > SERVO_BUFSIZ)
    {
//        pln("Servo buffer overflow");
    }

    bufferTx[0] = 0xFF;
    bufferTx[1] = 0xFF;
    bufferTx[2] = ID;
    bufferTx[3] = paramLength + 2 ;
    bufferTx[4] = instruction;

    char bCount = 0;

    for(bCount = 0; bCount < paramLength; ++bCount)
    {
        bufferTx[bCount+5] = param[bCount];
    }

    char chkSum = 0;

    for(bCount = 2; bCount < packetLength - 1; ++bCount)
    {
        chkSum += bufferTx[bCount];
    }
    bufferTx[packetLength-1] = ~chkSum;  //Writing  Checksum  with  Bit Inversion

    GPIOPinWrite(ctrl_pin_base, ctrl_pin_nb, 0xFF); // Input mode ON

    unsigned int i = 0;
    for (bCount = 0; bCount < packetLength; ++bCount)
    {
        while (!UARTCharPutNonBlocking(base, bufferTx[bCount]) && i < WDOG_LIMIT)
        {
            ++i;
        }
    }

    if (i < WDOG_LIMIT)
    {
        while (UARTBusy(base) && i < WDOG_LIMIT)
        {
            ++i;
        }
    }

    GPIOPinWrite(ctrl_pin_base, ctrl_pin_nb, 0x00); // Input mode OFF

    servoRxBufferClrRAW(base);
}

void servoCmdParam(char ID, char instruction, char paramLength, char* servoParam)
{
    servoCmdRAW(ID, instruction, paramLength,
                    FLAP_UART, FLAP_CMD_PIN_BASE, FLAP_CMD_PIN_NB,
                    servoParam, flapBufferTx);
}

char servoListenRAW(portTickType* xLastWakeTime, unsigned long base, char* bufferTx, char* bufferRx, unsigned long* rx_ms_wait)
{
    char received = 0;

    while (received < 4)
    {
        while(!UARTCharsAvail(base) && *rx_ms_wait > 0)
        {
            --(*rx_ms_wait);
            vTaskDelayUntil (xLastWakeTime, (1 / portTICK_RATE_MS));
        }

        if (*rx_ms_wait == 0)
        {
            *rx_ms_wait = UART_RX_MS_WAIT;
            pln2("SERVO DOWN!!!");
            UARTprintf("SERVO NOT RESPONDING: %d ", bufferRx[2]);
            return SERVO_NOT_RESP;
        }

        *rx_ms_wait = UART_RX_MS_WAIT;

        unsigned int i = 0;
        bool ok = false;
        while (!ok && i < WDOG_LIMIT)
        {
            long tmp = UARTCharGet(base);

            if (tmp != -1)
            {
                bufferRx[received] = tmp;
                ok = true;
            }
            ++i;
        }

        if (!ok)
            return SERVO_NOT_RESP;

        ++received;
    }

    if (bufferRx[0] != 0xFF || bufferRx[1] != 0xFF)
    {
        pln("Servo Head error");
        servoRxBufferClrRAW(base);
        return SERVO_HEAD_ERROR; // Head error
    }

    if (bufferRx[2] == SERVO_BROADCAST)
    {
        pln("Servo multiple broadcast error");
        servoRxBufferClrRAW(base);
        return SERVO_MULTIPLE_BROADCAST; // Someone else is broadcasting?!
    }

    if (bufferRx[2] != bufferTx[2] && bufferTx[2] != SERVO_BROADCAST)
    {
        pln("Servo bad sender error");
        servoRxBufferClrRAW(base);
        return SERVO_WRONG_SENDER; // Wrong sender
    }


    char stop = 4 + bufferRx[3];
    while(received < stop)
    {
        while(!UARTCharsAvail(base) && *rx_ms_wait > 0)
        {
            --(*rx_ms_wait);
            vTaskDelayUntil (xLastWakeTime, (1 / portTICK_RATE_MS));
        }

        if (*rx_ms_wait == 0)
        {
            *rx_ms_wait = UART_RX_MS_WAIT;
            pln2("SERVO DOWN!!!");
            UARTprintf("SERVO NOT REPONDING: %d ", bufferRx[2]);
            return SERVO_NOT_RESP;
        }

        *rx_ms_wait = UART_RX_MS_WAIT;

        unsigned int i = 0;
        bool ok = false;
        while (!ok && i < WDOG_LIMIT)
        {
            long tmp = UARTCharGet(base);

            if (tmp != -1)
            {
                bufferRx[received] = tmp;
                ok = true;
            }
            ++i;
        }

        if (!ok)
            return SERVO_NOT_RESP;
        ++received;
    }

    char chkSum = 0;
    for(char bCount = 2; bCount < received; ++bCount)
    {
        chkSum += bufferRx[bCount];
    }

    if(chkSum != 0xFF)
    {
        servoRxBufferClrRAW(base);
        return SERVO_BAD_CHKSUM;
    }

    servoRxBufferClrRAW(base);
    return SERVO_RECEIVED_OK;
}

void servoCmd(char ID, char instruction, char paramLength)
{
    servoCmdRAW(ID, instruction, paramLength,
                 SERVO_UART, SERVO_CMD_PIN_BASE, SERVO_CMD_PIN_NB,
                 servoParam, servoBufferTx);
}

void flapCmd(char ID, char instruction, char paramLength, portTickType* xLastWakeTime)
{
    bool ok;
    do
    {
        flapCmdUnchecked(ID, instruction, paramLength);
        ok = flapCheck(xLastWakeTime);
    } while (!ok);
}

void flapCmdUnchecked(char ID, char instruction, char paramLength)
{
    servoCmdRAW(ID, instruction, paramLength,
                     FLAP_UART, FLAP_CMD_PIN_BASE, FLAP_CMD_PIN_NB,
                     flapParam, flapBufferTx);
}

bool servoCheck(portTickType* xLastWakeTime)
{
    bool retval = true;

    char rval = servoListen(xLastWakeTime);
    if (rval != SERVO_RECEIVED_OK)
    {
        retval = false;
        UARTprintf("Servo %d error: %d\n", servoBufferTx[2], rval);

        errorReport("Servo proto error!");
    }
    
    if (!servoRcvStatusOK())
    {
        retval = false;

        UARTprintf("Servo reception error: %d\n", servoRxStatus);

        errorReport("Servo ack error!");
    }

    return retval;
}

bool flapCheck(portTickType* xLastWakeTime)
{
    bool retval = true;

    char rval = flapListen(xLastWakeTime);
    if (rval != SERVO_RECEIVED_OK)
    {
        retval = false;
        UARTprintf("Flap error: %d\n", rval);

        errorReport("Flap proto error!");
    }

    if (!flapRcvStatusOK())
    {
        retval = false;

        UARTprintf("Flap reception error: %d\n", flapRxStatus);

        errorReport("Flap ack error!");
    }

    return retval;
}

char servoListen(portTickType* xLastWakeTime)
{
    return servoListenRAW(xLastWakeTime, SERVO_UART, servoBufferTx, servoBufferRx, &rx_servo_ms_wait);
}

char flapListen(portTickType* xLastWakeTime)
{
    return servoListenRAW(xLastWakeTime, FLAP_UART, flapBufferTx, flapBufferRx, &rx_flap_ms_wait);
}

char servoRcvStatusOK()
{
    return servoBufferRx[4] == 0; // checks error byte in the Rx buffer
}

char flapRcvStatusOK()
{
    return flapBufferRx[4] == 0; // checks error byte in the Rx buffer
}

void servoRxBufferClrRAW(unsigned long base)
{
    unsigned int i = 0;
    while(UARTCharsAvail(base) && i < WDOG_LIMIT)
    {
        ++i;
        UARTCharGetNonBlocking(base);
    }
}

/* ======== Move the robot ======== */

void robotForward(portTickType* xLastWakeTime, unsigned long duration){
    servoSetSpeed(xLastWakeTime, 0, -0.4);
    servoSetSpeed(xLastWakeTime, 1, 0.4);

    servoSync();

    vTaskDelayUntil (xLastWakeTime, (duration / portTICK_RATE_MS));
    servoSTOP();
}

void robotBackward(portTickType* xLastWakeTime, unsigned long duration){
    servoSetSpeed(xLastWakeTime, 0, 0.4);
    servoSetSpeed(xLastWakeTime, 1, -0.4);

    servoSync();

    vTaskDelayUntil (xLastWakeTime, (duration / portTICK_RATE_MS));
    servoSTOP();
}

/* ================================ */
    
char servoSetSpeed(portTickType* xLastWakeTime, char ID, float speed){
   char data[2]; // Contains the data to send (2 * 8 bits)
   int goalSpeed = (0x3FF * custom_abs(speed));
   // If speed is < 0, servo will turn clowkwise 
   if(speed < 0){
       goalSpeed |= (0x1 << 10); // Set the 10th bit to 1 
   }
   data[0] = goalSpeed & 0xFF; // "downval" : Bits 0->7
   data[1] = goalSpeed >> 8; // "upval" : Bits 8->15

   return servoForward(xLastWakeTime, ID, data[1], data[0]);
}

char servoForward(portTickType* xLastWakeTime, char ID, char upval, char downval)
{
    servoParam[0] = 0x20;
    servoParam[1] = downval;
    servoParam[2] = upval;
    servoCmd(ID, INST_REG_WRITE, 3); // CAUTION ! 'REG_' added for tests

    char rval = servoListen(xLastWakeTime);
    /*if (rval != SERVO_RECEIVED_OK)
        UARTprintf("Servo %d error: %d\n", servoBufferTx[2], rval);
    if (!servoRcvStatusOK())
        UARTprintf("Reception error: %d\n", *servoRxStatus);*/

    return rval;
}

char servoBackward(portTickType* xLastWakeTime, char ID, char upval, char downval)
{
    return servoForward(xLastWakeTime, ID, upval ^ 0x04, downval);
}

char servoForwardFULL(portTickType* xLastWakeTime, char ID)
{
    return servoForward(xLastWakeTime, ID, 3, 255);
}

char servoBackwardFULL(portTickType* xLastWakeTime, char ID)
{
    return servoBackward(xLastWakeTime, ID, 3, 255);
}

void servoFreeWheel()
{
    servoParam[0] = 0x06;
    servoParam[1] = 0x00;
    servoParam[2] = 0x00;
    servoParam[3] = 0x00;
    servoParam[4] = 0x00;
    servoCmd(SERVO_BROADCAST, INST_WRITE, 5);
}

void servoSTOP()
{
    servoParam[0] = 0x20;
    servoParam[1] = 0x00;
    servoParam[2] = 0x00;
    servoCmd(SERVO_BROADCAST, INST_WRITE, 3);
}

void flapSTOP()
{
    flapParam[0] = 0x20;
    flapParam[1] = 0x00;
    flapParam[2] = 0x00;
    flapCmdUnchecked(SERVO_BROADCAST, INST_WRITE, 3);
}

void flapLeftConfig(portTickType* xLastWakeTime)
{
    flapParam[0] = 0x06;
    flapParam[1] = FLAP_LEFT_DOWN_LOW;
    flapParam[2] = FLAP_LEFT_DOWN_HIGH;
    flapParam[3] = FLAP_LEFT_UP_LOW;
    flapParam[4] = FLAP_LEFT_UP_HIGH;
    flapCmd(3, INST_WRITE, 5, xLastWakeTime);
}

void flapLeftDown(portTickType* xLastWakeTime)
{
    flapParam[0] = 0x1E;
    flapParam[1] = FLAP_LEFT_DOWN_LOW;
    flapParam[2] = FLAP_LEFT_DOWN_HIGH;
    flapParam[3] = 0xFF;
    flapParam[4] = 0x03;
    flapCmd(3, INST_WRITE, 5, xLastWakeTime);
}

void flapLeftUp(portTickType* xLastWakeTime)
{
    flapRightDown(xLastWakeTime);

    flapParam[0] = 0x1E;
    flapParam[1] = FLAP_LEFT_UP_LOW;
    flapParam[2] = FLAP_LEFT_UP_HIGH;
    flapParam[3] = 0xFF;
    flapParam[4] = 0x07;
    flapCmd(3, INST_WRITE, 5, xLastWakeTime);
}

void flapLeftBall(portTickType* xLastWakeTime)
{
    flapParam[0] = 0x1E;
    flapParam[1] = FLAP_LEFT_BALL_LOW;
    flapParam[2] = FLAP_LEFT_BALL_HIGH;
    flapParam[3] = 0xFF;
    flapParam[4] = 0x07;
    flapCmd(3, INST_WRITE, 5, xLastWakeTime);
}

void flapLeftSecure(portTickType* xLastWakeTime)
{
    /*flapParam[0] = 0x1E;
    flapParam[1] = FLAP_LEFT_SECURE_LOW;
    flapParam[2] = FLAP_LEFT_SECURE_HIGH;
    flapParam[3] = 0xFF;
    flapParam[4] = 0x07;
    flapCmd(3, INST_WRITE, 5, xLastWakeTime);*/
}

void flapRightConfig(portTickType* xLastWakeTime)
{
    flapParam[0] = 0x06;
    flapParam[1] = FLAP_RIGHT_UP_LOW;
    flapParam[2] = FLAP_RIGHT_UP_HIGH;
    flapParam[3] = FLAP_RIGHT_DOWN_LOW;
    flapParam[4] = FLAP_RIGHT_DOWN_HIGH;
    flapCmd(4, INST_WRITE, 5, xLastWakeTime);
}

void flapRightDown(portTickType* xLastWakeTime)
{
    flapParam[0] = 0x1E;
    flapParam[1] = FLAP_RIGHT_DOWN_LOW;
    flapParam[2] = FLAP_RIGHT_DOWN_HIGH;
    flapParam[3] = 0xFF;
    flapParam[4] = 0x07;
    flapCmd(4, INST_WRITE, 5, xLastWakeTime);
}

void flapRightUp(portTickType* xLastWakeTime)
{

    flapParam[0] = 0x1E;
    flapParam[1] = FLAP_RIGHT_UP_LOW;
    flapParam[2] = FLAP_RIGHT_UP_HIGH;
    flapParam[3] = 0xFF;
    flapParam[4] = 0x03;
    flapCmd(4, INST_WRITE, 5, xLastWakeTime);
}

void flapRightBall(portTickType* xLastWakeTime)
{
    flapParam[0] = 0x1E;
    flapParam[1] = FLAP_RIGHT_BALL_LOW;
    flapParam[2] = FLAP_RIGHT_BALL_HIGH;
    flapParam[3] = 0xFF;
    flapParam[4] = 0x03;
    flapCmd(4, INST_WRITE, 5, xLastWakeTime);
}


/* ============== 2014 ============== */

static int servoConvertAngleToHex(int angle){
    if(angle < 0 || angle > 300)
        return 0;

    return (1023/300) * angle; // 1023 : Valeur max en hexadécimal - datasheet p16
}


void flapConfig(portTickType* xLastWakeTime, int angleDown, int angleUp){
    flapParam[0] = 0x06;

    int angleLimitUp = servoConvertAngleToHex(angleUp);
    int angleLimitDown = servoConvertAngleToHex(angleDown);

    // Up values -> CW
    flapParam[1] = angleLimitDown & 0xFF;
    flapParam[2] = angleLimitDown >> 8;

    // Down values -> CCW
    flapParam[3] = angleLimitUp & 0xFF;
    flapParam[4] = angleLimitUp >> 8;

    flapCmdUnchecked(FLAP_ID, INST_WRITE, 5);
}


void flapGoalAngle(portTickType* xLastWakeTime, int angle, float speed){
    flapParam[0] = 0x1E;
    int angleGoal = servoConvertAngleToHex(angle);

    flapParam[1] = angleGoal & 0xFF;
    flapParam[2] = angleGoal >> 8;

    /* Setting the speed also */
    int goalSpeed = (0x3FF * custom_abs(speed));

    // If speed is < 0, servo will turn clowkwise 
    if(speed < 0){
        goalSpeed |= (0x1 << 10); // Set the 10th bit to 1 
    }
    flapParam[3] = goalSpeed & 0xFF; // "downval" : Bits 0->7
    flapParam[4] = goalSpeed >> 8; // "upval" : Bits 8->15cm

    flapCmdUnchecked(FLAP_ID, INST_WRITE, 5);
}


void flapDown(portTickType* xLastWakeTime){
    flapGoalAngle(xLastWakeTime, 60, 0.5);
}

void flapUp(portTickType* xLastWakeTime){
    flapGoalAngle(xLastWakeTime, 150, 0.5);
}

/* NOT OPERATIONAL YET ! */

/* ================================== */


// Automatic control
/*
void servoLeft(portTickType* xLastWakeTime, char upval, char downval)
{
    servoParam[0] = 0x20;
    servoParam[1] = downval;
    servoParam[2] = upval;

    bool ok;
    do
    {
        servoCmd(SERVO_LEFT_ID, INST_REG_WRITE, 3);
        ok = servoCheck(xLastWakeTime);
    } while (!ok);
}

void servoRight(portTickType* xLastWakeTime, char upval, char downval)
{
    servoParam[0] = 0x20;
    servoParam[1] = downval;
    servoParam[2] = upval ^ 0x04;

    bool ok;
    do
    {
        servoCmd(SERVO_RIGHT_ID, INST_REG_WRITE, 3);
        ok = servoCheck(xLastWakeTime);
    } while (!ok);
}

*/

void servoLeft(portTickType* xLastWakeTime, char upval, char downval){
    servoSetSpeed(xLastWakeTime, 1, 0.4);
}

void servoRight(portTickType* xLastWakeTime, char upval, char downval){
    servoSetSpeed(xLastWakeTime, 0, -0.4);
}

void servoSync()
{
    servoCmd(SERVO_BROADCAST, INST_ACTION, 0);
}

float ultrason_convert(unsigned long value){

    float distance, voltage;
    voltage = (3.0/1023.0)*value;  // Calcul de la tension sur base du fait que la conversion entre le 0-3(V) et le 0-1024(numérique) est linéaire

    static const float dist_const = 2.54/(ULTRAS_VCC/512.0);
    distance = voltage * dist_const; // vcc/512 est le coefficient donné par la datasheet du capteur. Le 2,54 convertit la distance en centimètres

    //if(distance < 15.25)
        // On ne peut rien dire. En effet, le capteur renvoie les distances comprises entre 6 et 254 pouces (soit 15.24cm et 6.45m). Pour des objets plus proche que 15.24cm, il renverra 15.24. Il faut donc distinguer ce cas-ci.
        // De toute facon, 15cm semble etre une distance raisonnable pour commencer a changer de direction

    return distance;
}

float sharp_convert(unsigned long value)
{
    return 4800/(value - 20);
}




void throwSpear(portTickType* xLastWakeTime){

    GPIOPinWrite(CANON_PIN_BASE, CANON_PIN_NB, PIN_ON);

    //for (int i = 0; i < 700000; ++i); // Wait aporox 1 sec

    vTaskDelayUntil (xLastWakeTime, (150/ portTICK_RATE_MS));

    GPIOPinWrite(CANON_PIN_BASE, CANON_PIN_NB, PIN_OFF);
}


void throwSomeSpears(portTickType* xLastWakeTime, unsigned int num, unsigned long wait){

    for(unsigned int i = 0 ; i < num ; ++i){
        throwSpear(xLastWakeTime);
        vTaskDelayUntil (xLastWakeTime, (wait / portTICK_RATE_MS));
    }
}




extern xQueueHandle screenMsgQueue;

void servoBroadcast(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    //char* msg = pvPortMalloc(sizeof(char) * 21);

    //servoForwardFULL(&xLastWakeTime, 0);
    servoLEDWrite();

    // !!!!!!!! UART1 -> UART2  !!!!!!!!!
    /*for (int i = 0; i < 256; ++i)
    {
        if(2000000/(i+1) < 100000)
            break;

        UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 2000000/(i+1),
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // 8bit, stop1, no parity

        servoParam[0] = 0x19; // LED
        servoParam[1] = 0x01; // Value 1
        servoCmd(SERVO_BROADCAST, INST_WRITE, 2);

        m_itoa(i, msg, 10);
        xQueueSend(screenMsgQueue, (void*) &msg, 0);

        vTaskDelayUntil (&xLastWakeTime, (500 / portTICK_RATE_MS));
    }


    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 200000,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // 8bit, stop1, no parity



    m_itoa(256, msg, 10);
    xQueueSend(screenMsgQueue, (void*) &msg, 0);
    servoParam[0] = 0x19; // LED
    servoParam[1] = 0x01; // Value 1
    servoCmd(SERVO_BROADCAST, INST_WRITE, 2);
    */


    while(1){
        vTaskDelayUntil (&xLastWakeTime, (100000 / portTICK_RATE_MS));
    }


}
