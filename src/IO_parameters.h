
#ifndef __IOPARAMETERS_H
#define __IOPARAMETERS_H



#define	    LedGreen    0x04
#define     LINE_SPACE  8
#define     LINE_WIDTH  21
#define     LINE_0      0
#define     LINE_1      8
#define     LINE_2      16
#define     LINE_3      24
#define     LINE_4      32
#define     LINE_5      40
#define     LINE_6      48

// A0 = UART0
// A1 = UART0

#define ANALOG_SELECT_LOW_PIN_BASE GPIO_PORTB_BASE
#define ANALOG_SELECT_LOW_PIN_NB GPIO_PIN_6

#define SERVO_CMD_PIN_BASE GPIO_PORTB_BASE
#define SERVO_CMD_PIN_NB   GPIO_PIN_5

#define ANALOG_SELECT_HIGH_PIN_BASE GPIO_PORTB_BASE
#define ANALOG_SELECT_HIGH_PIN_NB GPIO_PIN_4

// C4 = QEI

#define STARTUP_MODULE_PIN_BASE GPIO_PORTC_BASE
#define STARTUP_MODULE_PIN_NB GPIO_PIN_5

// D2 = UART1
// D3 = UART1

#define FLAP_CMD_PIN_BASE GPIO_PORTD_BASE
#define FLAP_CMD_PIN_NB   GPIO_PIN_4

#define BALLOON_PIN_BASE GPIO_PORTD_BASE
#define BALLOON_PIN_NB   GPIO_PIN_5

#define SECURITY_PIN_BASE GPIO_PORTD_BASE
#define SECURITY_PIN_NB   GPIO_PIN_6

// F4 = N/A, but known to work for reading + interrupt

// G2 = blinky
// G6 = QEI
// G7 = QEI
// H3 = QEI

#define SERVO_UART UART1_BASE
#define FLAP_UART  UART2_BASE

#endif		 /* __IOPARAMETERS_H */
