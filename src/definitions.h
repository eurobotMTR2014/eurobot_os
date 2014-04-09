#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include "ustdlib.h"
#include "uartstdio.h"

/// Board includes
#include "lm3s2965.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "rit128x96x4.h"

/// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/// Boolean type
typedef char bool;
#define true 1
#define false 0

#define PIN_ON 0xFF
#define PIN_OFF 0x00

#define STRAT_1 1
#define STRAT_2 0

#define WDOG_LIMIT 100000
#define INTEL_WDOG_LIMIT 100

/// Own includes
#include "IO_parameters.h"
#include "custom_math.h"
#include "world.h"
#include "tools_lib.h"
#include "cmdline.h"
#include "captors.h"
#include "intel.h"
#include "control.h"
#include "gametable.h"
#include "odometry.h"

void errorReport(char* msg);
void batteryReport(unsigned long bVolt);

#endif
