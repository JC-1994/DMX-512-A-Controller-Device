/*
    Jesee Calzada
    CSE 4342
    Final Project
*/

//**************************************************************************
// Hardware Target
//**************************************************************************

// Target Platform: EK*TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED

// UART Interface:
//   U1TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//**************************************************************************
// Device includes, defines, and assembler directives
//**************************************************************************

#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdlib.h>

// The following libraries below are from Texas Instruments Incorporated and part of the tiva-c-ware
// libraries

#include "driverlib/eeprom.h"  // in arm linker file path must include
                               // driverlib/ccs/debug/driverlib.lib
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

// PIN PC5
// PIN PC4 IS RECIEVE PIN
// DE pin (PC6) setting to 1 enables transmission in controller mode

#define UART1_PC5_TX                (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define TRANSMITTER_ENABLE_PC6      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  //  transmit led pin PF1
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))  //  receive led pin PF3
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))  //  led to represent device 1 pin PF2

// if data is greater than or equal to this number blue led will be on
#define BLUE_ON_RANGE 125

#define BLUE_LED_MASK 4
#define RED_LED_MASK 2
#define GREEN_LED_MASK 8

#define MAX_CHARS 80      // max chars will recieve from terminal
#define MAX_DATA  255     // max value a address can recieve
#define MAX_ADDRESS 512   // max value an address can be

// EEPROM DEFINES:

#define CONTROLLER_MODE 0xF               // indicates in controller mode
#define DEVICE_MODE 0x2                   // indicates in device mode
#define MODE_ADDRESS_SETTING 0x0          // address to access setting in EEPROM to indicate what mode we are in
#define MODE_ADDRESS_SETTING_LENGTH 4     // number of bytes to read/write in eeprom to indicate mode
#define DEVICE_ADDRESS_SETTING 0x4        // address to access setting in EEPROM to determine our device address
#define DEVICE_ADDRESS_SETTING_LENGTH 12  // number of bytes to read/write in eeprom for our device address

// ASCII CODES USED:

#define DELETE 127    // backspace
#define CARRIAGE_RETURN 13

// ALSO, CONSIDER LINING UP THE 1ST LETTER OF EACH GLOBAL VARIABLE NAME SO THAT THEY'RE IN THE SAME COLUMN
// SIMILARLY FOR THE = SYMBOLS AND THE COMMENTS.
// GIVE ALL GLOBAL VARIABLES AN INITIAL VALUE (NOT JUST SOME OF THEM).

// Global Variables
uint8_t  TimeoutGreenLed        = 0;
uint16_t MaxAddress             = MAX_ADDRESS;  // max dmx address avaliable to send data to
uint8_t  TxDmxData[MAX_ADDRESS] = {0};          // data to send for each individual dmx address (512 of them default)
uint8_t  RxDmxData[MAX_ADDRESS] = {0};          // data to recieve for each individual dmx address (512 of them default)
uint16_t TxPhase = 0;                           // indicates what state of transmitting we are in for transmitting
uint16_t RxPhase = 0;                           // indicates what state of transmitting we are in for receiving
bool Send = true;                               // indicates whether DMX stream is sent continuously or not
bool ControllerMode = false;                    // indicates whether the device is in controller mode
                                                // if not it is in device mode, this variable is based of data
                                                // in EEPROM, default is in device mode but will
                                                // be overwritten if there exists a setting in EEPROM
bool RedLedOn = false;                          // indicates if red led was on or not
bool GreenLedOn = false;                        // indicates if green led was on or not

uint16_t DeviceAddress = 1;                     // address used to access this device when in device mode
                                                // default is 1 but will be overwritten based of setting in EEPROM

//**************************************************************************
// Subroutines
//**************************************************************************

// DG BE CONSISTENT AND MAKE SURE THAT ALL FUNCTIONS HAVE A FUNCTION HEADER
// WHICH DESCIBES FUNCTION PURPOSE, PARAMETER DESCRIPTIONS, RETURN VALUE,
// ANY ASSUMPTIONS, ETC.
// DON'T DO IT FOR SOME FUNCTIONS BUT NOT OTHERS.

//************************************************************************
// send a char to the terminal
// inputs:
//   char to send to the terminal
// outputs:
//   none
//************************************************************************

void putcUart0(char c)
{
    // UART flag register remains set until data is done being transfered. pg 897 datasheet
    // checking bit 5, if bit 5 set buffer is full and shouldn't write into data register
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

//************************************************************************
// Blocking function that writes a string when the UART buffer is not full
// inputs:
//   str to send to the terminal
// outputs:
//   none
//************************************************************************

void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

//************************************************************************
// Configures the hardware for the device
// inputs:
//   none
// outputs:
//   none
//**************************************************************************

void initHw(void)
{
    uint32_t eepromData;
    uint32_t ui32EEPROMInit;

    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    SYSCTL_GPIOHBCTL_R = 0;
    SYSCTL_RCGC2_R = (SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC);  // power on port F, A and C

    // set pin pc5 and pc6 to gpio mode
    // set pc5 and pc6 to output
    GPIO_PORTC_DIR_R = 96;
    GPIO_PORTC_DR2R_R = 96;
    GPIO_PORTC_DEN_R = 112;

    // pin 1 is an output others are inputs
    GPIO_PORTF_DIR_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    // set drive strength to 2mA
    GPIO_PORTF_DR2R_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    // digital enable pin 1
    GPIO_PORTF_DEN_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    RED_LED = 0;
    GREEN_LED = 0;
    BLUE_LED = 0;

    GPIO_PORTA_DEN_R |= 3;  // digital enable

    // controls the config for GPIO PA0 and PA1
    // pin 0 and 1 of port a
    // are given another function to be determined
    // by the gpio port control register(GPIOPCTL)
    GPIO_PORTA_AFSEL_R |= 3;
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // turn on UART0 and UART1, leave other uarts in same status
    SYSCTL_RCGCUART_R |= (SYSCTL_RCGCUART_R0 | SYSCTL_RCGCUART_R1);

    // delay after initialize uart
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");

    UART0_CTL_R = 0;                // turn off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz) to clock the uart
    UART0_IBRD_R = 21;              // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;              // round(fract(r)*64)=45

    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;             // configure for 8N1 w/ 16*level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // below code includes functions from Texas Instruments Incorporated eeprom.h

    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);               // enable clock eeprom0
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0));
    ui32EEPROMInit = EEPROMInit();

    if(EEPROM_INIT_ERROR == ui32EEPROMInit)  // DG TRY TO PUT CONSTANT VALUE BEFORE THE == AS THIS GUARDS AGAINST ACCIDENTAL =
    {
        putsUart0("EEPROM ERROR \r\n");
        while(1);  // continous loop so can debug error
    }

    EEPROMRead(&eepromData,MODE_ADDRESS_SETTING,MODE_ADDRESS_SETTING_LENGTH);

    if(CONTROLLER_MODE ==  eepromData )  // check if in controller mode or device mode
    {
        ControllerMode = true;
    }
    else if(DEVICE_MODE == eepromData)
    {
        ControllerMode = false;
    }

    // check if device address exists in EEPROM
    EEPROMRead(&eepromData,DEVICE_ADDRESS_SETTING,DEVICE_ADDRESS_SETTING_LENGTH);
    DeviceAddress = eepromData;

    // DG CONSIDER LINKIN UP THE = SYMBOLS TO MAKE THE ASSIGNMENTS EASIER TO SKIM DOWN
    // set up timer 1:
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;    // turn on timer
    TIMER1_CTL_R       &= ~TIMER_CTL_TAEN;        // turn off timer before configure
    TIMER1_CFG_R        = TIMER_CFG_32_BIT_TIMER; // 32 bit timer a+b
    TIMER1_TAMR_R       = TIMER_TAMR_TAMR_PERIOD; // periodic mode (count down)
    TIMER1_IMR_R        = TIMER_IMR_TATOIM;       // turn on interrupts
    NVIC_EN0_R         |= 1 <<(INT_TIMER1A-16);   // turn on interrupt 37


    // Timer 2 Config for command received led green
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;     // turn on timer
    TIMER2_CTL_R       &= ~TIMER_CTL_TAEN;         // turn-off timer before reconfiguring
    TIMER2_CFG_R       |= TIMER_CFG_32_BIT_TIMER;  // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R      |= TIMER_TAMR_TAMR_PERIOD;  // set to periodic mode (count down)
    TIMER2_TAILR_R      = 8000000;                 // 200ms interrupts
    TIMER2_IMR_R       |= TIMER_IMR_TATOIM;        // turn-on interrupts
    NVIC_EN0_R         |= 1 << (INT_TIMER2A-16);
    TIMER2_CTL_R       |= TIMER_CTL_TAEN;          // turn-on timer


}

//**************************************************************************
// elapse approximately 1 microsecond of time
// inputs:
//   us number of microseconds to wait
//**************************************************************************

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ***
                                                // 40 clocks/us + error
}

//************************************************************************
// get a char from terminal
// inputs:
//   none
// outputs:
//   char recieved from uart
//************************************************************************

char getcUart0(void)
{
    // (flag register & recieve fifo empty bit)
    while (UART0_FR_R & UART_FR_RXFE);
                                // read data register
    return (UART0_DR_R & 0xFF); // only return lower 8 bits ignore errors
}

//************************************************************************
// used to get string from terminal in array
// inputs:
//   *str will be the array that will hold the string from terminal
//   maxChars is max size for *str
// outputs:
//   *str will be complete string from terminal
//   char that is size of str
//************************************************************************

uint8_t getStringUart0(char *str, uint8_t maxChars)
{
    uint8_t charPosition = 0;  // num of char in string we are currently processing
    uint8_t currentChar;       // current char we are processing
    while(true)
    {
        currentChar = getcUart0();
        putcUart0(currentChar);
        if (DELETE == currentChar )  // if user presses backspace
        {
            if (charPosition > 0)    // only deincrement counter if exists
            {
                charPosition = charPosition - 1;

                // delete previous character replace with space
                putcUart0('\b');
                putcUart0(' ');
                putcUart0('\b');
            }

        }
        else if(CARRIAGE_RETURN == currentChar)  // enter was pressed
        {

            putsUart0("\n\r");
            str[charPosition] = 0;               // add null terminator

            if(true == ControllerMode)           // in controller mode blink green led when carriage return recieved
            {

                RED_LED = 0;
                GREEN_LED = 1;
                GreenLedOn = true;
                TimeoutGreenLed = 4;             // 4 * 200ms = 800ms
            }
            return charPosition;
        }
        else if(currentChar >= ' ')              // is it a printable character?
        {
            currentChar = tolower(currentChar);  // set to lowercase
            str[charPosition++] = currentChar;   // store char in string

            if (charPosition == maxChars)        // indicates string can only hold 1 more char
            {
               str[charPosition] = 0;            // add null terminator
               putsUart0("\r\n");
               return charPosition;
            }
        }

    }
}

//**************************************************************************
// converts all delimiters in string to nulls and
// parses string based on following rules:
// position of command and arguements are saved based on the events:
// delimiter to letter a*z ('a')
// delimiter to number 0*9 ('n')
// delimiters are anything that is not a number or letter
// inputs:
//   *str, string to parse
//   pos[] array to hold position of important stuff
//   *fieldCount size of pos[] array
//   type[] what type is the object in pos[]
// outputs:
//   pos[] position of important stuff in *str (always indicates start of # if more
//   than one digit)
//   *fieldCount size of pos[] and type[] array
//   type[x] what type is the important stuff in *str at pos[x]
//**************************************************************************

void parseStringUart0(char *str, uint8_t pos[], uint8_t *fieldCount,char type[],uint8_t strSize)
{
    uint8_t i;
    *fieldCount = 0;

    for(i = 0; i < strSize; i++)                     // replace delimiters with nulls
    {
        if( ( (str[i] < 'a') || (str[i] > 'z') ) )
        {
            if (((str[i] < '0') || (str[i] > '9')))  // means it is a delimiter
            {
                str[i] = 0;
            }
        }

    }

    for(i = 1; i < strSize; i++)                      // start at 2nd char, find important char transitions
    {
        if((0 == str[i-1]) && ( 0 != str[i]))        // means important info as unimportant things are not 0
        {
            if( (str[i] >= 'a') && (str[i] <= 'z') )  // type is letter 'a' in alphabet
            {
                type[((int)*fieldCount)] = 'a';
            }
            else                                      // type is number 'n'
            {
                type[((int)*fieldCount)] = 'n';
            }
            pos[((int)*fieldCount)] = i;
            (*fieldCount)++;
        }
    }

}

//**************************************************************************
// determines if user typed in command held in cmdName, that it contains
// the min number of arguements needed for the command and that
// the arguements are valid (of type number and not letter)
// inputs:
//   *cmdName  command to determine if user typed in
//   numArgs number of arguements this command needs to have
//   userInput what user typed in from terminal
//   fieldCount number of important information in userInput, size of pos[] or type[]
//   argIndex index of where the arguements begin in userInput
//   inputSize size of userInput
// returns:
//   bool value true or false
//**************************************************************************

bool isCommand(char *cmdName, uint8_t numArgs,char *userInput, uint8_t fieldCount,uint8_t argIndex, uint8_t inputSize)
{
    uint8_t i;
    if(numArgs > 0)                            // check that arguements are numbers and not letters
    {


        for(i = argIndex; i < inputSize; i++)  // make sure there are no letters in arguement
        {
            if((userInput[i] < '0') || (userInput[i] > '9'))
            {
                if((userInput[i] != 0))        // if a not a delimiter then its a letter
                {
                    return 0;
                }
            }
        }
    }

    return((0 == strcmp(userInput,cmdName)) && ((fieldCount) == numArgs));
}

//**************************************************************************
// returns interger value from string based on argNum
// inputs:
//   argNum is which arguement to get from string
//   string is user input

uint16_t getValue(uint8_t argNum, char *string)
{
   // send part of string starting at where the arguement begins
   return atoi(&(string[argNum]));

}

//**************************************************************************
//  given integer converts to string  and formats correctly
// does not work with negative numbers
// inputs:
//   num number to convert to string
//   *str string to hold output
//   length of str
// outputs
//   str string of converted number
//**************************************************************************

void myitoa(uint16_t num, char *str, uint8_t length)
{
    uint8_t i;
    uint8_t j;
    uint16_t numToConvert = num;

    for(i=1;i<=length;i++)
    {
        str[length-i] = ((num%10) + '0');
        num = num / 10;
    }
    str[i-1] = 0;

    if(numToConvert != 0)            // only do if num to convert isnt 0
    {
        i = 0;
        while('0' == str[i])         // counts 0 at beginning of string
        {
            j++;
            i++;
        }

        for(i=0; i<(length-j); i++)  // moves string over in array
        {
            str[i] = str[i+j];
        }
        str[i] = 0;
    }
    else
    {
        str[0] = '0';
        str[1] = 0;
    }
}

//**************************************************************************
// load timer1 with a value
// inputs:
//   TimeValue to load
// outputs
//   none
//**************************************************************************

void loadTimer1(uint32_t TimeValue)
{
    TIMER1_CTL_R  &= ~TIMER_CTL_TAEN; // turn of timer
    TIMER1_TAILR_R = TimeValue;       // load timer
    TIMER1_CTL_R  |= TIMER_CTL_TAEN;  // turn on timer
}

//**************************************************************************
// turn off uart 1
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void disableUart1(void)
{
    UART1_CTL_R = 0; // turn off uart1
    UART1_IM_R = 0;  // turn off transmit buffer
}

//**************************************************************************
// starts transmission of data in controller mode
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void startDataTransmission(void)
{
        if((0 == TxPhase ) && (true == Send ))
        {
            RedLedOn = true;
            if (false == GreenLedOn)
            {
                RED_LED = 1;             // indicates data is being transmitted
            }

            TRANSMITTER_ENABLE_PC6 = 1;  // DE pin setting to 1 enables tranmission
            disableUart1();
            // remove transmit and recieve configs for
            // pc4 and pc5
            GPIO_PORTC_PCTL_R &= ~ GPIO_PCTL_PC5_U1TX;
            GPIO_PORTC_PCTL_R &= ~ GPIO_PCTL_PC4_U1RX;
            GPIO_PORTC_AFSEL_R &= 207; // clear bits 4 and 5 leave others the same
            UART1_PC5_TX = 0;          // Transmit pin set to 0 sending a break

            loadTimer1(7040);          // load value of 7040
                                       // 7040/40mhz = .000176 or 176us
            TxPhase = 1;
        }
}

//**************************************************************************
// process rxData send to devices should be less than 44us so can keep
// receiving data around 1600 clocks
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void processData(void)
{
    if(0 == DeviceAddress)
    {
        DeviceAddress = 1;
    }

    if(RxDmxData[DeviceAddress-1] < BLUE_ON_RANGE) // DG AVOID USE OF "MAGIC NUMBERS" IN FAVOUR OF #DEFINED VALUES. BUT IF KEPT EXPLAIN FULLY WITH A COMMENT.
    {
        BLUE_LED = 0;
    }
    else if(RxDmxData[DeviceAddress-1] >= BLUE_ON_RANGE)
    {
        BLUE_LED = 1;
    }
}

//**************************************************************************
// configure uart1 for transmission or receiving
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void configUart1(void)
{
    GPIO_PORTC_AFSEL_R |= 48;

    // enable TX, RX, and module
    // NEED |= OR WILL RUIN JTAG PINS AND CAN NO LONGER CONNECT
    GPIO_PORTC_PCTL_R |= (GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX);

    UART1_CTL_R = 0;                //  turn off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz) to clock the uart

    // Configure UART1 to 250000 baud
    // ((250000*16) = 4mhz, 40mhz/4mhz = 10
    UART1_IBRD_R = 10;
    UART1_FBRD_R = 0;  // uart fractional baud rate divisor


    if(true == ControllerMode)  // DG TRY TO PUT LITERAL/CONSTANT VALUES BEFORE THE == RATHER THAN AFTER THE == AS THIS GUARDS AGAINST ACCIDENTAL USE OF = INSTEAD OF ==
    {
        // ENABLE UART1 ON PIN BASED ON DMX512-A PROTOCOL
        // want 8 bit data, 1 startbit and 2 stop bits (line control register length = 8)
        // as the 2 stop bits are make after break and break
        // 16 lvl fifo buffer enabled set bit.
        UART1_LCRH_R |= (UART_LCRH_WLEN_8 | UART_LCRH_STP2 |UART_LCRH_FEN);

        // turn on transmitter and whole uart module.
        // enable TX, module
        // EOT enabled cause interupt to fire as soon as fifo is empty
        UART1_CTL_R = (UART_CTL_TXE  | UART_CTL_UARTEN | UART_CTL_EOT);

        UART1_IM_R |= UART_IM_TXIM;        // turn on transmit buffer interrupt
        UART1_IM_R &= ~UART_IM_RXIM;       // turn off receive interrupt
        UART1_IM_R &= ~UART_IM_BEIM;       // turn-off break interrupt
        NVIC_EN0_R |= 1 <<(INT_UART1-16);  // turn on interrupt 22
    }
    else  // else we are in device mode
    {
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;

        // turn on reciever and whole uart module.
        UART1_CTL_R = (UART_CTL_RXE| UART_CTL_UARTEN | UART_CTL_TXE);
        UART1_IM_R &= ~UART_IM_TXIM;  // turn off transmit interrupt
        UART1_IM_R |= UART_IM_RXIM;  // turn on receive interrupt

        NVIC_EN0_R |= 1 <<(INT_UART1-16);  // turn on interrupt 22
        TimeoutGreenLed = 10;              // 2 sec interrupts if break not detected green led should blink
        GREEN_LED = 1;                     // dmx signal detected
        GreenLedOn = true;
    }
    // delay 4 cycles after initialize uart
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");
}
//**************************************************************************
// uart1 function to transmit data up to max address
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void Uart1Isr(void)
{
    if(true == ControllerMode)
    {
        UART1_ICR_R |= UART_ICR_TXIC;  // clear transmit interrupt flag

        // when sending 1 start bit of logic 0 is sent(automatically) 8 data bits of data(configured)
        // and 2 stop bits of logic 1 (configured)
        if(3 == TxPhase)
        {
            while((TxPhase-3)< MaxAddress)
            {

                    while (UART1_FR_R & UART_FR_TXFF);
                    UART1_DR_R = TxDmxData[(TxPhase-3)];
                    TxPhase++;
            }


            while((UART1_FR_R & UART_FR_BUSY) == UART_FR_BUSY);  // wait while uart is busy
            TxPhase = 0;
            startDataTransmission();
        }
    }
    else                               // else we are in device mode
    {
        if((UART_DR_BE & UART1_DR_R))  // if break error has occured
        {
            RxPhase = 1;
            processData();
            UART1_ICR_R = UART_ICR_RXIC;

        }
        else if(1 == RxPhase )
        {
            // 2 sec interrupts if break not detected green led should blink
            // if data keeps being detected renew timeout
            TimeoutGreenLed = 10;

            // and with data register to ignore error flags
            if ((0 == UART1_DR_R & UART_DR_DATA_M))
            {
                // received start code move on to next phase
                RxPhase = 2;
            }
        }
        else if ((RxPhase >= 2) && ((RxPhase - 2) < MaxAddress))  // recieve data put in array
        {

            RxDmxData[(RxPhase - 2)] = (UART1_DR_R & UART_DR_DATA_M);
            RxPhase++;

        }
        if (MAX_ADDRESS == (RxPhase - 2))  // start phase over
        {
            RxPhase = 0;

        }

    } // DG TRY TO AVOID FUNCTIONS WITH MANY LEVELS OF {}. BUT IF NECESSARY THEN CONSIDER PUTTING A COMMENT AFTER THE } TO MAKE IT OBVIOUS WHICH { IT CORRESPONDS TO; E.G. } // END-IF, } // END-WHILE, ETC
}

//**************************************************************************
// when timer reachs 0 it will jump to here must be configured in startup_ccs.c
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void Timer1Isr(void)
{
    TIMER1_ICR_R = TIMER_ICR_TATOCINT; //clear interrupt flag
    if(1 == TxPhase)
    {
        UART1_PC5_TX = 1; // sending make after break
        loadTimer1(480);  // load value
                          // 480/40mhz = .000012 or 12us
        TxPhase = 2;
    }
    else if(TxPhase == 2)
    {
        UART1_PC5_TX = 0;                 // turn off pin before going back to uart to avoid errors in data tranmission

        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;  // turn off timer
        configUart1();
        TxPhase = 3;
        UART1_DR_R = 0;                   // start character for uart transmission
    }
}

//**************************************************************************
// configured for 200ms interrupts used for blinking leds
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void Timer2Isr(void)
{
    TIMER2_ICR_R |= TIMER_ICR_TATOCINT;  // clear interrupt flag
    if(TimeoutGreenLed > 0)
    {
        TimeoutGreenLed --;
    }
    else if(false == ControllerMode)  // device mode need to toggle led
    {
        TimeoutGreenLed = 3;          // will turn off/on green led every 600ms
        GREEN_LED       = ~GREEN_LED;
        if(1 == GREEN_LED)
        {
            GreenLedOn = true;
        }
        else
        {
            GreenLedOn = false;
        }
    }
    else  // controller mode
    {
        GREEN_LED  = 0;
        GreenLedOn = false;

        if(RedLedOn == true) // turn red led back on used to avoid green and red being on at same time
        {
            RED_LED = 1;
        }

    }

}


//**************************************************************************
// runs startup code for the microcontroller
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void startUpService(void)
{

    initHw();
    TxPhase = 0;

    // step 1 on powerup flash red led for 1s
    RED_LED = 1;
    waitMicrosecond(1000000); // wait 1s
    RED_LED = 0;
    waitMicrosecond(1000000); // wait 1s
}

//**************************************************************************
// runs code for microcontroller that is in controller mode
// inputs:
//   input[] input received from terminal
//   fieldCount size of pos array
//   pos[] position of important stuff in input
//   sizeOfInput length of input including terminating null
// outputs
//   whether a command was executed or not
//**************************************************************************
bool controllerModeService(char input[], uint8_t fieldCount, uint8_t pos[], uint8_t sizeOfInput)
{
    char    tempStr[5];              // used to hold conversions from string to integer
    bool commandExecuted    = false; // indicates if a command has been executed
    uint8_t tempStrSize     = (sizeof(tempStr)/sizeof(tempStr[0]));
    uint16_t data           = 0;     // data that address x will contain
    uint16_t i              = 0;
    uint16_t address        = 0;     // dmx address x to send data to
    uint32_t eepromData;             // temp variable store data to read/write to EEPROM


    putsUart0("Configured as controller\r\n");


    TxPhase = 0;
    startDataTransmission();

    if((isCommand("clear", 0, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted = true;
        for(i = 0; i < MaxAddress; i++)
        {
            TxDmxData[i] = 0;
        }
        putsUart0("Dmx Data Cleared Successfully\r\n");
    }
    else if((isCommand("set", 2, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted = true;
        address         = getValue(pos[0],input);


        if((0 == address))  // did not check - numbers as impossible to input them
        {
            putsUart0("Address value must be greater than 0");
            putsUart0("\r\n");
        }

        data = getValue(pos[1],input);

        // range check 0 to 255
        if(data > MAX_DATA)  // did not check - numbers as impossible to input them
        {
            putsUart0("Data value exceeds the max allowable value of ");
            myitoa(MAX_DATA,tempStr,tempStrSize);
            putsUart0(tempStr);
            putsUart0("\r\n");
        }

        TxDmxData[address-1] = data;
    }
    else if((isCommand("get", 1, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted = true;
        address         = getValue(pos[0],input);
        if((0 == address))
        {
            putsUart0("Address value must be greater than 0");
            putsUart0("\r\n");
        }
        putsUart0("Value is ");
        myitoa((TxDmxData[address-1]),tempStr,tempStrSize);
        putsUart0(tempStr);
        putsUart0("\r\n");
    }
    else if((isCommand("on", 0, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted        = true;
        Send                   = true;
        RedLedOn               = true;
        TRANSMITTER_ENABLE_PC6 = 1;

        if(false == GreenLedOn)
        {
            RED_LED = 1;  // indicates data is being transmitted
        }
        putsUart0("DMX stream set to be sent continuously\r\n");
    }
    else if((isCommand("off", 0, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted        = true;
        Send                   = false;
        RED_LED                = 0;  // data not being transmitted
        RedLedOn               = false;
        TRANSMITTER_ENABLE_PC6 = 0;

        putsUart0("DMX stream set to not be sent\r\n");
    }
    else if((isCommand("max", 1, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted = true;
        MaxAddress      = getValue(pos[0],input);

       if(((getValue(pos[0],input)) > MAX_ADDRESS) || ((getValue(pos[0],input)) == 0))  // did not check - numbers as impossible to input them
        {
            putsUart0("Max address value is not in the range of 1 to ");
            myitoa(MAX_ADDRESS,tempStr,tempStrSize);
            putsUart0(tempStr);
            putsUart0("\r\n");
        }

        putsUart0("New max address updated\r\n");
    }
    else if((isCommand("device", 0, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted = true;
        eepromData      = DEVICE_MODE;

        EEPROMProgram(&eepromData,MODE_ADDRESS_SETTING,MODE_ADDRESS_SETTING_LENGTH); // function from Texas Instruments Incorporated eeprom.h

        ControllerMode = false;
        RED_LED        = 0;  // not transmitting
        RedLedOn       = false;
        putsUart0("Device mode enabled\r\n");
    }
    return commandExecuted;
}

//**************************************************************************
// runs code for microcontroller that is in device mode
// inputs:
//   input[] input received from terminal
//   fieldCount size of pos array
//   pos[] position of important stuff in input
//   sizeOfInput length of input including terminating null
// outputs
//   whether a command was executed or not
//**************************************************************************

bool deviceModeService(char input[], uint8_t fieldCount, uint8_t pos[], uint8_t sizeOfInput)
{
    bool    commandExecuted = false; // indicates if a command has been executed
    char    tempStr[5];              // used to hold conversions from string to integer
    uint8_t tempStrSize     = (sizeof(tempStr)/sizeof(tempStr[0]));
    uint16_t address        = 0;     // device modes address if in device mode
    uint32_t eepromData;             // temp variable store data to read/write to EEPROM

    putsUart0("Configured as device\r\n");


    TRANSMITTER_ENABLE_PC6 = 0;
    configUart1();
    if(TimeoutGreenLed == 0)
    {
        TimeoutGreenLed = 10; // 2 sec interrupts if break not detected green led should blink
    }

    if((isCommand("controller", 0, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted = true;
        eepromData      = CONTROLLER_MODE;

        EEPROMProgram(&eepromData,MODE_ADDRESS_SETTING,MODE_ADDRESS_SETTING_LENGTH); // function from Texas Instruments Incorporated eeprom.h

        ControllerMode = true;
        RedLedOn       = true;

        if( false == GreenLedOn )
        {
            RED_LED = 1;  // transmitting
        }
        putsUart0("Controller mode enabled\r\n");
    }
    else if((isCommand("address", 1, input, fieldCount,pos[0],sizeOfInput)))
    {
        commandExecuted = true;
        address         = getValue(pos[0],input);

        if((address > MaxAddress) || (0 == address))  // did not check negative numbers as impossible to input them
        {
            putsUart0("Address value is not in the range of 1 to ");
            myitoa(MaxAddress,tempStr,tempStrSize);
            putsUart0(tempStr);
            putsUart0("\r\n");
        }

        DeviceAddress = address;
        eepromData    = DeviceAddress;
        EEPROMProgram(&eepromData,DEVICE_ADDRESS_SETTING,DEVICE_ADDRESS_SETTING_LENGTH); // function from Texas Instruments Incorporated eeprom.h
    }

    return commandExecuted;
}

//**************************************************************************
// Main starting point of program
// inputs:
//   none
// outputs
//   none
//**************************************************************************

void main(void)
{
    // DG CONSIDER LINING UP THE 1ST LETTER OF THE VARIABLE NAMES INTO THE SAME COLUMN. SAME FOR THE = SYMBOL.
    char    input[MAX_CHARS + 1];    // add 1 for terminating null
    char    type[MAX_CHARS];         // type of important stuff in array
    uint8_t pos[MAX_CHARS];          // position of important stuff in array
    uint8_t fieldCount      = 0;     // size of pos and type arrays
    uint8_t sizeOfInput     = 0;     // length of input includeing terminating null
    bool    commandExecuted = false; // indicates if a command has been executed

    while (true)
    {

        putsUart0("Type in your command\r\nEverything other than characters and letters will be ignored.\r\n");
        commandExecuted = false;
        sizeOfInput     = getStringUart0(input, MAX_CHARS);
        parseStringUart0(input, pos, &fieldCount, type, sizeOfInput);

        if (true == ControllerMode)
        {
            commandExecuted = controllerModeService(input, fieldCount, pos, sizeOfInput);
        }
        else
        {
            commandExecuted = deviceModeService(input, fieldCount, pos, sizeOfInput);
        }

        if (false == commandExecuted)
        {
            putsUart0("Error command not understood\r\n");
        }
    }
}
