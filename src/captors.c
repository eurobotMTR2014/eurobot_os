#pragma GCC diagnostic ignored "-Wchar-subscripts"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "captors.h"

#define SWITCH_DELAY 1
#define DOUBLE_FETCH_DELAY 1

void DELAY_SWITCH()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil (&xLastWakeTime, (SWITCH_DELAY / portTICK_RATE_MS));
}

void DELAY_FETCH()
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil (&xLastWakeTime, (DOUBLE_FETCH_DELAY / portTICK_RATE_MS));
}

extern xQueueHandle screenMsgQueue;
extern xSemaphoreHandle usBufSwitchMutex;
extern xSemaphoreHandle sharpBufSwitchMutex;

bool usBufSwitch = false;
bool sharpBufSwitch = false;
unsigned long ultraValsBuf1Init[4];
unsigned long ultraValsBuf2Init[4];

unsigned long sharpValsBuf1Init[4];
unsigned long sharpValsBuf2Init[4];

unsigned long batteryBuf;

unsigned long* ultraValsBuf1 = ultraValsBuf1Init;
unsigned long* ultraValsBuf2 = ultraValsBuf2Init;
unsigned long* sharpValsBuf1 = sharpValsBuf1Init;
unsigned long* sharpValsBuf2 = sharpValsBuf2Init;
unsigned long* ultraVals = ultraValsBuf1Init;
unsigned long ultraValsMed1[5];
unsigned long ultraValsMed2[5];
unsigned long ultraValsMed3[5];
unsigned long ultraValsMed4[5];
unsigned char ultraCount = 0;
unsigned long* sharpVals = sharpValsBuf1Init;

void captorSelect(char capt);

unsigned long fetchChannelRAW(unsigned long ch);
unsigned long fetchChannel(unsigned long ch) {fetchChannelRAW(ch); DELAY_FETCH(); return fetchChannelRAW(ch);}
unsigned long fetchChan0() {return fetchChannel(0);}
unsigned long fetchChan1() {return fetchChannel(1);}
unsigned long fetchChan2() {return fetchChannel(2);}

void fetchSharp();
void fetchUS();
void fetchBat();
void stackOverflowSort(unsigned long* data, char N);

void sharpNewValue();
void USNewValue(char count);

void captorsTask(void* pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
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
    }
}

void captorSelect(char capt)
{
    switch (capt)
    {
    	case 1:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_OFF);  // Ultrasons avant
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_OFF);
    		return;

    	case 2:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_OFF);  // Ultrasons arrière
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_ON);
            return;

        case 3:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_ON);   // Sharps avant
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_OFF);
            return;

        case 4:
            GPIOPinWrite(ANALOG_SELECT_HIGH_PIN_BASE, ANALOG_SELECT_HIGH_PIN_NB, PIN_ON);   // Sharps arrière
            GPIOPinWrite(ANALOG_SELECT_LOW_PIN_BASE, ANALOG_SELECT_LOW_PIN_NB, PIN_ON);
            return;

    	default:
    		return;
    }
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
    captorSelect(3); // Sharp avant
    DELAY_SWITCH();
    sharpVals[0] = fetchChan0();
    sharpVals[1] = fetchChan1();
    captorSelect(4); // Sharp arrière
    DELAY_SWITCH();
    sharpVals[2] = fetchChan0();
    sharpVals[3] = fetchChan1();
}

void fetchUS()
{
    captorSelect(1); // US avant
    DELAY_SWITCH();
//    ultraVals[0] += fetchChan0();
//    ultraVals[1] += fetchChan1();
    ultraValsMed1[ultraCount] = fetchChan0();
    ultraVals[0] += ultraValsMed1[ultraCount];
    ultraValsMed2[ultraCount] = fetchChan1();
    ultraVals[1] += ultraValsMed2[ultraCount];

    captorSelect(2); // US arrière
    DELAY_SWITCH();
    ultraValsMed3[ultraCount] = fetchChan0();
    ultraVals[2] += ultraValsMed3[ultraCount];
    ultraValsMed4[ultraCount] = fetchChan1();
    ultraVals[3] += ultraValsMed4[ultraCount];
    //ultraValsMed3[ultraCount] = fetchChan1();

    ultraCount++;
    if (ultraCount == 5){
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
    //UARTprintf("SHARP [0] = %d  \t [1] = %d  \t [2] = %d \n", sharpVals[0], sharpVals[1], sharpVals[2]);
    xSemaphoreTake(sharpBufSwitchMutex, portMAX_DELAY);
    sharpBufSwitch = !sharpBufSwitch;
    xSemaphoreGive(sharpBufSwitchMutex);

    sharpVals = (sharpBufSwitch) ? sharpValsBuf2 : sharpValsBuf1;
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
   stackOverflowSort(ultraValsMed3, count);

//    ultraVals[0] = ultraValsMed1[2];
//    ultraVals[1] = ultraValsMed2[2];
//    ultraVals[2] = ultraValsMed3[2];
    ultraVals[0] /= count;
    ultraVals[1] /= count;
    ultraVals[2] /= count;
    ultraVals[3] /= count;

//    UARTprintf("Average values[0] = %d   \t[1] = %d \t [2] = %d \t [3] = %d\n", ultraVals[0], ultraVals[1], ultraVals[2], ultraVals[3]);
//    UARTprintf("Median values[0] = %d   \t[1] = %d \t [2] = %d \t [3] = %d\n", ultraValsMed1[2], ultraValsMed2[2], ultraValsMed3[2], ultraVals[3]);

    xSemaphoreTake(usBufSwitchMutex, portMAX_DELAY);
    usBufSwitch = !usBufSwitch;
    xSemaphoreGive(usBufSwitchMutex);

    ultraVals = (usBufSwitch) ? ultraValsBuf2 : ultraValsBuf1;

    ultraVals[0] = 0;
    ultraVals[1] = 0;
    ultraVals[2] = 0;
    ultraVals[3] = 0;
}
