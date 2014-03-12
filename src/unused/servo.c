/*
 * The Example of Dynamixel Evaluation with Atmega128
 * Date : 2005.5.11
 * Author : BS KIM
 */

/*
 * included files
 */
#define ENABLE_BIT_DEFINITIONS
//#include <io.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#define cbi(REG8,BITNUM) REG8 &= ~(_BV(BITNUM))
#define sbi(REG8,BITNUM) REG8 |= _BV(BITNUM)


typedef unsigned char byte;
typedef unsigned int word;
#define ON 1
#define OFF 0
#define _ON 0
#define _OFF 1


//--- Control Table Address ---
//EEPROM AREA
#define P_MODEL_NUMBER_L      0
#define P_MODOEL_NUMBER_H     1
#define P_VERSION             2
#define P_ID                  3
#define P_BAUD_RATE           4
#define P_RETURN_DELAY_TIME   5
#define P_CW_ANGLE_LIMIT_L    6
#define P_CW_ANGLE_LIMIT_H    7
#define P_CCW_ANGLE_LIMIT_L   8
#define P_CCW_ANGLE_LIMIT_H   9
#define P_SYSTEM_DATA2        10
#define P_LIMIT_TEMPERATURE   11
#define P_DOWN_LIMIT_VOLTAGE  12
#define P_UP_LIMIT_VOLTAGE    13
#define P_MAX_TORQUE_L        14
#define P_MAX_TORQUE_H        15
#define P_RETURN_LEVEL        16
#define P_ALARM_LED           17
#define P_ALARM_SHUTDOWN      18
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION_L  20
#define P_DOWN_CALIBRATION_H  21
#define P_UP_CALIBRATION_L    22
#define P_UP_CALIBRATION_H    23

#define P_TORQUE_ENABLE         (24)
#define P_LED                   (25)
#define P_CW_COMPLIANCE_MARGIN  (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE   (28)
#define P_CCW_COMPLIANCE_SLOPE  (29)
#define P_GOAL_POSITION_L       (30)
#define P_GOAL_POSITION_H       (31)
#define P_GOAL_SPEED_L          (32)
#define P_GOAL_SPEED_H          (33)
#define P_TORQUE_LIMIT_L        (34)
#define P_TORQUE_LIMIT_H        (35)
#define P_PRESENT_POSITION_L    (36)
#define P_PRESENT_POSITION_H    (37)
#define P_PRESENT_SPEED_L       (38)
#define P_PRESENT_SPEED_H       (39)
#define P_PRESENT_LOAD_L        (40)
#define P_PRESENT_LOAD_H        (41)
#define P_PRESENT_VOLTAGE       (42)
#define P_PRESENT_TEMPERATURE   (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME            (45)
#define P_MOVING (46)
#define P_LOCK                  (47)
#define P_PUNCH_L               (48)
#define P_PUNCH_H               (49)

//--- Instruction ---
#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET          0x06
#define INST_DIGITAL_RESET  0x07
#define INST_SYSTEM_READ    0x0C
#define INST_SYSTEM_WRITE   0x0D
#define INST_SYNC_WRITE     0x83
#define INST_SYNC_REG_WRITE 0x84

#define CLEAR_BUFFER gbRxBufferReadPointer = gbRxBufferWritePointer
#define DEFAULT_RETURN_PACKET_SIZE 6
#define BROADCASTING_ID 0xfe

#define TxD8 TxD81
#define RxD8 RxD81

//Hardware Dependent Item
#define DEFAULT_BAUD_RATE 34   //57600bps at 16MHz

////// For CM-5
#define  RS485_TXD  PORTE  &=  ~_BV(PE3),PORTE  |=  _BV(PE2)
//PORT_485_DIRECTION = 1
#define  RS485_RXD  PORTE  &=  ~_BV(PE2),PORTE  |=  _BV(PE3)
//PORT_485_DIRECTION = 0
/*
////// For CM-2
#define RS485_TXD PORTE |= _BV(PE2); //_485_DIRECTION = 1
#define RS485_RXD PORTE &= ~_BV(PE2);//PORT_485_DIRECTION = 0
*/
//#define TXD0_FINISH  UCSR0A,6  //This bit is for checking TxD Buffer
in CPU is empty or not.
//#define TXD1_FINISH  UCSR1A,6

#define SET_TxD0_FINISH   sbi(UCSR0A,6)
#define RESET_TXD0_FINISH cbi(UCSR0A,6)
#define CHECK_TXD0_FINISH bit_is_set(UCSR0A,6)
#define SET_TxD1_FINISH  sbi(UCSR1A,6)
#define RESET_TXD1_FINISH cbi(UCSR1A,6)
#define CHECK_TXD1_FINISH bit_is_set(UCSR1A,6)

#define RX_INTERRUPT 0x01
#define TX_INTERRUPT 0x02
#define OVERFLOW_INTERRUPT 0x01
#define SERIAL_PORT0 0
#define SERIAL_PORT1 1
#define BIT_RS485_DIRECTION0  0x08  //Port E
#define BIT_RS485_DIRECTION1  0x04  //Port E

#define BIT_ZIGBEE_RESET               PD4  //out : default 1 //PORTD
#define BIT_ENABLE_RXD_LINK_PC         PD5  //out : default 1
#define BIT_ENABLE_RXD_LINK_ZIGBEE     PD6  //out : default 0
#define BIT_LINK_PLUGIN                PD7  //in, no pull up

void TxD81(byte bTxdData);
void TxD80(byte bTxdData);
void TxDString(byte *bData);
void TxD8Hex(byte bSentData);
void TxD32Dec(long lLong);
byte RxD81(void);
void MiliSec(word wDelayTime);
void PortInitialize(void);
void SerialInitialize(byte bPort, byte bBaudrate, byte bInterrupt);
byte TxPacket(byte bID, byte bInstruction, byte bParameterLength);
byte RxPacket(byte bRxLength);
void PrintBuffer(byte *bpPrintBuffer, byte bLength);

// --- Gloval Variable Number ---
volatile byte gbpRxInterruptBuffer[256];
byte gbpParameter[128];
byte gbRxBufferReadPointer;
byte gbpRxBuffer[128];
byte gbpTxBuffer[128];
volatile byte gbRxBufferWritePointer;

int main(void)
{
    byte bCount,bID, bTxPacketLength,bRxPacketLength;

    PortInitialize(); //Port In/Out Direction Definition
    RS485_RXD; //Set RS485 Direction to Input State.
    SerialInitialize(SERIAL_PORT0,1,RX_INTERRUPT);//RS485
    Initializing(RxInterrupt)
    SerialInitialize(SERIAL_PORT1,DEFAULT_BAUD_RATE,0);  //RS232
    Initializing(None Interrupt)

    gbRxBufferReadPointer  =  gbRxBufferWritePointer  =  0;    //RS485
    RxBuffer Clearing.

    sei();  //Enable Interrupt -- Compiler Function
    TxDString("\r\n  [The  Example  of  Dynamixel  Evaluation  with ATmega128,GCC-AVR]");

//Dynamixel Communication Function Execution Step.
// Step 1. Parameter Setting (gbpParameter[]). In case of no parameter
    instruction(Ex.  INST_PING),  this  step  is  not
    needed.
//  Step  2.  TxPacket(ID,INSTRUCTION,LengthOfParameter);  --Total
    TxPacket Length is returned
//  Step  3.  RxPacket(ExpectedReturnPacketLength);  --  Real  RxPacket
    Length is returned
// Step 4 PrintBuffer(BufferStartPointer,LengthForPrinting);

    bID = 1;
    TxDString("\r\n\n Example 1. Scanning Dynamixels(0~9). -- Any Key to Continue.");
    RxD8();
    for(bCount = 0; bCount < 0x0A; bCount++)
    {
        bTxPacketLength = TxPacket(bCount,INST_PING,0);
        bRxPacketLength = RxPacket(255);
        TxDString("\r\n TxD:");
        PrintBuffer(gbpTxBuffer,bTxPacketLength);
        TxDString(", RxD:");
        PrintBuffer(gbpRxBuffer,bRxPacketLength);
        if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE)
        {
            TxDString(" Found!! ID:");
            TxD8Hex(bCount);
            bID = bCount;
        }
    }

    TxDString("\r\n\n  Example  2.  Read  Firmware  Version.  --  Any  Key  to Continue.");
    RxD8();
    gbpParameter[0] = P_VERSION; //Address of Firmware Version
    gbpParameter[1] = 1; //Read Length
    bTxPacketLength = TxPacket(bID,INST_READ,2);
    bRxPacketLength  =
        RxPacket(DEFAULT_RETURN_PACKET_SIZE+gbpParameter
                 [1]);
    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);
    if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1])
    {
        TxDString("\r\n Return Error      : ");
        TxD8Hex(gbpRxBuffer[4]);
        TxDString("\r\n Firmware Version  : ");
        TxD8Hex(gbpRxBuffer[5]);
    }

    TxDString("\r\n\n  Example  3.  LED  ON  --  Any  Key  to  Continue.");
    RxD8();
    gbpParameter[0] = P_LED; //Address of LED
    gbpParameter[1] = 1; //Writing Data
    bTxPacketLength = TxPacket(bID,INST_WRITE,2);
    bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE);
    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);
    TxDString("\r\n\n  Example  4.  LED  OFF  --  Any  Key  to  Continue.");
    RxD8();
    gbpParameter[0] = P_LED; //Address of LED
    gbpParameter[1] = 0; //Writing Data
    bTxPacketLength = TxPacket(bID,INST_WRITE,2);
    bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE);
    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);

    TxDString("\r\n\n  Example  5.  Read  Control  Table.  --  Any  Key  to Continue.");
    RxD8();
    gbpParameter[0] = 0; //Reading Address
    gbpParameter[1] = 49; //Read Length
    bTxPacketLength = TxPacket(bID,INST_READ,2);
    bRxPacketLength  =
        RxPacket(DEFAULT_RETURN_PACKET_SIZE+gbpParameter
                 [1]);

    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);
    if(bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE+gbpParameter[1])
    {
        TxDString("\r\n");
        for(bCount = 0; bCount < 49; bCount++)
        {
            TxD8('[');
            TxD8Hex(bCount);
            TxDString("]:");
            TxD8Hex(gbpRxBuffer[bCount+5]);
            TxD8(' ');
        }
    }

    TxDString("\r\n\n Example 6. Go 0x200 with Speed 0x100 -- Any Key to
              Continue.");
    RxD8();
    gbpParameter[0] = P_GOAL_POSITION_L; //Address of Firmware Version
    gbpParameter[1] = 0x00; //Writing Data P_GOAL_POSITION_L
    gbpParameter[2] = 0x02; //Writing Data P_GOAL_POSITION_H
    gbpParameter[3] = 0x00; //Writing Data P_GOAL_SPEED_L
    gbpParameter[4] = 0x01; //Writing Data P_GOAL_SPEED_H
    bTxPacketLength = TxPacket(bID,INST_WRITE,5);
    bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE);
    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);

    TxDString("\r\n\n Example 7. Go 0x00 with Speed 0x40 -- Any Key to
              Continue.");
    RxD8();
    gbpParameter[0] = P_GOAL_POSITION_L; //Address of Firmware Version
    gbpParameter[1] = 0x00; //Writing Data P_GOAL_POSITION_L
    gbpParameter[2] = 0x00; //Writing Data P_GOAL_POSITION_H
    gbpParameter[3] = 0x40; //Writing Data P_GOAL_SPEED_L
    gbpParameter[4] = 0x00; //Writing Data P_GOAL_SPEED_H
    bTxPacketLength = TxPacket(bID,INST_WRITE,5);
    bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE);
    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);

    TxDString("\r\n\n Example 8. Go 0x3ff with Speed 0x3ff -- Any Key to
              Continue.");
    RxD8();
    gbpParameter[0] = P_GOAL_POSITION_L; //Address of Firmware Version
    gbpParameter[1] = 0xff; //Writing Data P_GOAL_POSITION_L
    gbpParameter[2] = 0x03; //Writing Data P_GOAL_POSITION_H
    gbpParameter[3] = 0xff; //Writing Data P_GOAL_SPEED_L
    gbpParameter[4] = 0x03; //Writing Data P_GOAL_SPEED_H
    bTxPacketLength = TxPacket(bID,INST_WRITE,5);
    bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE);
    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);

    TxDString("\r\n\n Example 9. Torque Off -- Any Key to Continue.");
    RxD8();
    gbpParameter[0] = P_TORQUE_ENABLE; //Address of LED
    gbpParameter[1] = 0; //Writing Data
    bTxPacketLength = TxPacket(bID,INST_WRITE,2);
    bRxPacketLength = RxPacket(DEFAULT_RETURN_PACKET_SIZE);
    TxDString("\r\n TxD:");
    PrintBuffer(gbpTxBuffer,bTxPacketLength);
    TxDString("\r\n RxD:");
    PrintBuffer(gbpRxBuffer,bRxPacketLength);

    TxDString("\r\n\n End. Push reset button for repeat");
    while(1);
}

void PortInitialize(void)
{
    DDRA  =  DDRB  =  DDRC  =  DDRD  =  DDRE  =  DDRF  =  0;    //Set  all  port  to input direction first.
    PORTB  =  PORTC  =  PORTD  =  PORTE  =  PORTF  =  PORTG  =  0x00;  //PortData initialize to 0
    cbi(SFIOR,2); //All Port Pull Up ready
    DDRE  |=  (BIT_RS485_DIRECTION0|BIT_RS485_DIRECTION1);  //set  output the bit RS485direction

    DDRD  |=
        (BIT_ZIGBEE_RESET|BIT_ENABLE_RXD_LINK_PC|BIT_ENA
         BLE_RXD_LINK_ZIGBEE);

    PORTD &= ~_BV(BIT_LINK_PLUGIN); // no pull up
    PORTD |= _BV(BIT_ZIGBEE_RESET);
    PORTD |= _BV(BIT_ENABLE_RXD_LINK_PC);
    PORTD |= _BV(BIT_ENABLE_RXD_LINK_ZIGBEE);
}

/*
TxPacket() send data to RS485.
TxPacket()  needs  3  parameter;  ID  of  Dynamixel,  Instruction  byte,
Length of parameters.
TxPacket() return length of Return packet from Dynamixel.
*/
byte TxPacket(byte bID, byte bInstruction, byte bParameterLength)
{
    byte bCount,bCheckSum,bPacketLength;

    gbpTxBuffer[0] = 0xff;
    gbpTxBuffer[1] = 0xff;
    gbpTxBuffer[2] = bID;
    gbpTxBuffer[3]  =  bParameterLength+2;
//Length(Paramter,Instruction,Checksum)
    gbpTxBuffer[4] = bInstruction;
    for(bCount = 0; bCount < bParameterLength; bCount++)
    {
        gbpTxBuffer[bCount+5] = gbpParameter[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength+4+2;
    for(bCount  =  2;  bCount  <  bPacketLength-1;  bCount++)  //except
        0xff,checksum
    {
        bCheckSum += gbpTxBuffer[bCount];
    }
    gbpTxBuffer[bCount]  =  ~bCheckSum;  //Writing  Checksum  with  Bit
    Inversion

    RS485_TXD;
    for(bCount = 0; bCount < bPacketLength; bCount++)
    {
        sbi(UCSR0A,6);//SET_TXD0_FINISH;
        TxD80(gbpTxBuffer[bCount]);
    }
    while(!CHECK_TXD0_FINISH); //Wait until TXD Shift register empty
    RS485_RXD;
    return(bPacketLength);
}

/*
RxPacket() read data from buffer.
RxPacket() need a Parameter; Total length of Return Packet.
RxPacket() return Length of Return Packet.
*/

byte RxPacket(byte bRxPacketLength)
{
#define RX_TIMEOUT_COUNT2   3000L
#define RX_TIMEOUT_COUNT1  (RX_TIMEOUT_COUNT2*10L)
    unsigned long ulCounter;
    byte bCount, bLength, bChecksum;
    byte bTimeout;
    bTimeout = 0;
    for(bCount = 0; bCount < bRxPacketLength; bCount++)
    {
        ulCounter = 0;
        while(gbRxBufferReadPointer == gbRxBufferWritePointer)
        {
            if(ulCounter++ > RX_TIMEOUT_COUNT1)
            {
                bTimeout = 1;
                break;
            }
        }
        if(bTimeout) break;
        gbpRxBuffer[bCount]  =
            gbpRxInterruptBuffer[gbRxBufferReadPointer++];
    }
    bLength = bCount;
    bChecksum = 0;

    if(gbpTxBuffer[2] != BROADCASTING_ID)
    {
        if(bTimeout && bRxPacketLength != 255)
        {
            TxDString("\r\n [Error:RxD Timeout]");
            CLEAR_BUFFER;
        }

        if(bLength > 3) //checking is available.
        {
            if(gbpRxBuffer[0] != 0xff || gbpRxBuffer[1] != 0xff )
            {
                TxDString("\r\n [Error:Wrong Header]");
                CLEAR_BUFFER;
                return 0;
            }
            if(gbpRxBuffer[2] != gbpTxBuffer[2] )
            {
                TxDString("\r\n [Error:TxID != RxID]");
                CLEAR_BUFFER;
                return 0;
            }
            if(gbpRxBuffer[3] != bLength-4)
            {
                TxDString("\r\n [Error:Wrong Length]");
                CLEAR_BUFFER;
                return 0;
            }
            for(bCount  =  2;  bCount  <  bLength;  bCount++)  bChecksum  +=
                    gbpRxBuffer[bCount];
            if(bChecksum != 0xff)
            {
                TxDString("\r\n [Error:Wrong CheckSum]");
                CLEAR_BUFFER;
                return 0;
            }
        }
    }
    return bLength;
}


/*
PrintBuffer() print data in Hex code.
PrintBuffer()  needs  two  parameter;  name  of  Pointer(gbpTxBuffer,
gbpRxBuffer)
*/
void PrintBuffer(byte *bpPrintBuffer, byte bLength)
{
    byte bCount;
    for(bCount = 0; bCount < bLength; bCount++)
    {
        TxD8Hex(bpPrintBuffer[bCount]);
        TxD8(' ');
    }
    TxDString("(LEN:");
    TxD8Hex(bLength);
    TxD8(')');
}

/*
TxD80() send data to USART 0.
*/
void TxD80(byte bTxdData)
{
    while(!TXD0_READY);
    TXD0_DATA = bTxdData;

}
/*
TXD81() send data to USART 1.
*/
void TxD81(byte bTxdData)
{
    while(!TXD1_READY);
    TXD1_DATA = bTxdData;
}
/*
TXD32Dex() change data to decimal number system
*/
void TxD32Dec(long lLong)
{
    byte bCount, bPrinted;
    long lTmp,lDigit;
    bPrinted = 0;
    if(lLong < 0)
    {
        lLong = -lLong;
        TxD8('-');
    }
    lDigit = 1000000000L;
    for(bCount = 0; bCount < 9; bCount++)
    {
        lTmp = (byte)(lLong/lDigit);
        if(lTmp)
        {
            TxD8(((byte)lTmp)+'0');
            bPrinted = 1;
        }
        else if(bPrinted) TxD8(((byte)lTmp)+'0');
        lLong -= ((long)lTmp)*lDigit;
        lDigit = lDigit/10;
    }
    lTmp = (byte)(lLong/lDigit);
    /*if(lTmp)*/
    TxD8(((byte)lTmp)+'0');
}
/*
TxDString() prints data in ACSII code.
*/
void TxDString(byte *bData)
{
    while(*bData)
    {
        TxD8(*bData++);
    }
}
/*
RxD81() read data from UART1.
RxD81() return Read data.
*/
byte RxD81(void)
{
    while(!RXD1_READY);
    return(RXD1_DATA);
}
/*
SIGNAL() UART0 Rx Interrupt - write data to buffer
*/
SIGNAL (SIG_UART0_RECV)
{
    gbpRxInterruptBuffer[(gbRxBufferWritePointer++)] = RXD0_DATA;
}
