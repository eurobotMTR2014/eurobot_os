#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "captors.h"

#define SWITCH_DELAY 1
#define DOUBLE_FETCH_DELAY 1

#define ADC_SAMPLE_SEQ_0 0
#define ADC_SAMPLE_SEQ_1 1
#define ADC_SAMPLE_SEQ_2 2

#define CAPTOR_SHARPS 1
#define CAPTOR_US 2

/**
 * @fn DELAY_SWITCH
 * Applies a delay of SWITCH_DELAY ms
 */
void DELAY_SWITCH()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil (&xLastWakeTime, (SWITCH_DELAY / portTICK_RATE_MS));
}

/**
 * @fn DELAY_FETCH
 * Applies a delay of DOUBLE_FETCH_DELAY ms
 */
void DELAY_FETCH()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil (&xLastWakeTime, (DOUBLE_FETCH_DELAY / portTICK_RATE_MS));
}

static void sharp_to_world();
static void captors_init();
extern xQueueHandle screenMsgQueue;
/*guigui
extern xSemaphoreHandle usBufSwitchMutex;
extern xSemaphoreHandle sharpBufSwitchMutex;

bool usBufSwitch = false;
bool sharpBufSwitch = false;

unsigned long ultraValsBuf1Init[4];
unsigned long ultraValsBuf2Init[4];

unsigned long sharpValsBuf1Init[4];
unsigned long sharpValsBuf2Init[4];
*/
unsigned long batteryBuf;
/*guigui
unsigned long* ultraValsBuf1 = ultraValsBuf1Init;
unsigned long* ultraValsBuf2 = ultraValsBuf2Init;
unsigned long* sharpValsBuf1 = sharpValsBuf1Init;
unsigned long* sharpValsBuf2 = sharpValsBuf2Init;
unsigned long* ultraVals = ultraValsBuf1Init;
*/

unsigned long ultraVals[2]; //guigui
unsigned long sharpVals[2]; //guigui

unsigned long ultraValsMed1[5];
unsigned long ultraValsMed2[5];
//unsigned long ultraValsMed3[5];
//unsigned long ultraValsMed4[5];
unsigned char ultraCount = 0;
//guigui unsigned long* sharpVals = sharpValsBuf1Init;

/**
 * @fn captorSelect
 * Selects the captors that is going to write on the input channel
 * @param the captor number (CAPTOR_US_FRONT, CAPTOR_US_BACK, CAPTOR_SHARP_FRONT, CAPTOR_SHARP_BACK)
 */
void captorSelect(char capt);

/**
 * @fn fetchChannelRAW
 * Fetches a given channel
 * @param the sample sequence number 
 * @return the digitalized signal 
 */
unsigned long fetchChannelRAW(unsigned long ch);

unsigned long fetchChannel(unsigned long ch) 
{
    fetchChannelRAW(ch); 
    DELAY_FETCH(); 
    return fetchChannelRAW(ch);
}

unsigned long fetchChan0() 
{
    return fetchChannel(ADC_SAMPLE_SEQ_0);
}

unsigned long fetchChan1() 
{
    return fetchChannel(ADC_SAMPLE_SEQ_1);
}

unsigned long fetchChan2() 
{
    return fetchChannel(ADC_SAMPLE_SEQ_2);
}

/**
 * @fn fetchSharp
 * Fetches the output of the sharps (front and back) and put the results in the buffer sharpVals
 */
void fetchSharp();

/**
 * @fn fetchUS
 * Fetches the output of the ultrason captors (front and back) and pu the results in the buffer ultraVals
 */
void fetchUS();

/**
 * @fn fetchBat
 * Fetches the battery voltage and send a batteryReport
 */
void fetchBat();

void stackOverflowSort(unsigned long* data, char N);

/**
 * ?????
 */
void sharpNewValue();

/**
 * ?????
 */
void USNewValue(char count);

void captorsTask(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    captors_init();

    while (true)
    {/*
        // Initial fetch
        fetchBat();
        fetchUS();

        DELAY_SWITCH();
        fetchSharp();
        sharpNewValue();

        // Second & Third US fetch (20ms and 40ms)
        for (char i = 0; i < 2; ++i)
        {
            vTaskDelayUntil (&xLastWakeTime, (20 / portTICK_RATE_MS));
            fetchUS();
        }

        // Second sharp fetch (50ms)
        vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
        fetchSharp();
        sharpNewValue();

        // Fourth US fetch (60ms)
        vTaskDelayUntil (&xLastWakeTime, (10 / portTICK_RATE_MS));
        fetchUS();

        // Fifth US fetch (80ms)
        vTaskDelayUntil (&xLastWakeTime, (20 / portTICK_RATE_MS));
        fetchUS();
        USNewValue(5);

        // Complete cycle (100ms)
        vTaskDelayUntil (&xLastWakeTime, (20 / portTICK_RATE_MS));
        */
        sharp_to_world();
        vTaskDelayUntil (&xLastWakeTime, (30 / portTICK_RATE_MS));
    }
}

void captorSelect(char capt)
{
    switch (capt)
    {
        case CAPTOR_SHARPS:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_OFF);  // Deux sharps avant
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_OFF);
            return;

        case CAPTOR_US:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_OFF);  // Ultrasons 
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_ON);
            return;
        /*
        case CAPTOR_SHARP_FRONT:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_ON);   // Sharps avant
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_OFF);
            return;

        case CAPTOR_SHARP_BACK:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_ON);   // Sharps arrière
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_ON);
            return;
        */

        default:
            return;
    }
}

void captors_init()
{
    GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_OFF);  // Deux sharps avant
    GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_OFF);
}

void sharp_to_world()
{
    sharpVals[0] = fetchChan0();
    sharpVals[1] = fetchChan1();
    world_set_sharp_vals(sharpVals);
}

unsigned long fetchChannelRAW(unsigned long ch)
{
    unsigned long ADCval = 0;

    ADCProcessorTrigger(ADC_BASE, ch);

    while(!ADCIntStatus(ADC_BASE, ch, false))
    {}

    ADCSequenceDataGet(ADC_BASE, ch, &ADCval);

    return ADCval;
}

void fetchSharp()
{
    captorSelect(CAPTOR_SHARPS); // Sharps
    DELAY_SWITCH();
    sharpVals[0] = fetchChan0();
    sharpVals[1] = fetchChan1();
}

void fetchUS()
{
    captorSelect(CAPTOR_US); // US avant
    DELAY_SWITCH();
//    ultraVals[0] += fetchChan0();
//    ultraVals[1] += fetchChan1();
    ultraValsMed1[ultraCount] = fetchChan0();
    ultraVals[0] += ultraValsMed1[ultraCount];
    ultraValsMed2[ultraCount] = fetchChan1();
    ultraVals[1] += ultraValsMed2[ultraCount];

    ultraCount++;
    if (ultraCount == 5){ // 5 mesures
        ultraCount = 0;
    }
}

void fetchBat()
{
    batteryBuf = fetchChan2();
    batteryReport(batteryBuf);
}

void sharpNewValue()
{
/*guigui
    //UARTprintf("SHARP [0] = %d  \t [1] = %d  \t [2] = %d \n", sharpVals[0], sharpVals[1], sharpVals[2]);
    xSemaphoreTake(sharpBufSwitchMutex, portMAX_DELAY);
    sharpBufSwitch = !sharpBufSwitch;
    xSemaphoreGive(sharpBufSwitchMutex);

    sharpVals = (sharpBufSwitch) ? sharpValsBuf2 : sharpValsBuf1;
*/
    world_set_sharp_vals(sharpVals); //guigui
}

void stackOverflowSort(unsigned long* data, char N) {
  int i, j;
  unsigned long v, t;

  if (N <= 1)
    return;

  // Partition elements
  v = data[0];
  i = 0;
  j = N;
  for(;;)
  {
    while(data[++i] < v && i < N) { }
    while(data[--j] > v) { }
    if( i >= j )
      break;
    t = data[i];
    data[i] = data[j];
    data[j] = t;
  }
  t = data[i-1];
  data[i-1] = data[0];
  data[0] = t;
  stackOverflowSort(data, i-1);
  stackOverflowSort(data+i, N-i);
}



void USNewValue(char count)
{

   //UARTprintf("UltraValsMed[0] = %d   \t[1] = %d \t [2] = %d \t [3] = %d \t [4] = %d\n", ultraValsMed1[0], ultraValsMed1[1], ultraValsMed1[2], ultraValsMed1[3], ultraValsMed1[4]);
   stackOverflowSort(ultraValsMed1, count);
//   UARTprintf("UltraValsMed[0] = %d   \t[1] = %d \t [2] = %d \t [3] = %d \t [4] = %d\n", ultraValsMed1[0], ultraValsMed1[1], ultraValsMed1[2], ultraValsMed1[3], ultraValsMed1[4]);
   stackOverflowSort(ultraValsMed2, count);

//    ultraVals[0] = ultraValsMed1[2];
//    ultraVals[1] = ultraValsMed2[2];
//    ultraVals[2] = ultraValsMed3[2];
    ultraVals[0] /= count;
    ultraVals[1] /= count;

//    UARTprintf("Average values[0] = %d   \t[1] = %d \t [2] = %d \t [3] = %d\n", ultraVals[0], ultraVals[1], ultraVals[2], ultraVals[3]);
//    UARTprintf("Median values[0] = %d   \t[1] = %d \t [2] = %d \t [3] = %d\n", ultraValsMed1[2], ultraValsMed2[2], ultraValsMed3[2], ultraVals[3]);

/*guigui    
    xSemaphoreTake(usBufSwitchMutex, portMAX_DELAY);
    usBufSwitch = !usBufSwitch;
    xSemaphoreGive(usBufSwitchMutex);

    ultraVals = (usBufSwitch) ? ultraValsBuf2 : ultraValsBuf1;
*/
    world_set_ultra_vals(ultraVals);//guigui

    ultraVals[0] = 0;
    ultraVals[1] = 0;

}
