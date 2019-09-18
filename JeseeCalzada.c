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

//The following libraries below are from Texas Instruments Incorporated and part of the tiva-c-ware
//libraries
 
#include "driverlib/eeprom.h"  //in arm linker file path must include
                               //driverlib/ccs/debug/driverlib.lib
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"


#define UART1_PC5_TX      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
//PIN PC5
//PIN PC4 IS RECIEVE PIN
// ^ used equation for stop_go_bitband.c in evernote 400063FC same as GPIO_PORTC_DATA_R number

#define TRANSMITTER_ENABLE_PC6      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
//DE pin (PC6) setting to 1 enables transmission in controller mode

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  //transmit led pin pf1
#define RED_LED_MASK 2

#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))  //receive led pin pf3
#define GREEN_LED_MASK 8

#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))//led to control to represent device 1 pin PF2
#define BLUE_LED_MASK 4

#define MAX_CHARS 80  //max chars will recieve from terminal
#define MAX_DATA  255  //max value a address can recieve
#define MAX_ADDRESS 512 //max value an address can be


//EEPROM DEFINES:

#define CONTROLLER_MODE 0xF  //indicates in controller mode
#define DEVICE_MODE 0x2 //indicates in device mode
#define MODE_ADDRESS_SETTING 0x0 //address to access setting in EEPROM to indicate what mode we are in
#define MODE_ADDRESS_SETTING_LENGTH 4  //number of bytes to read/write in eeprom to indicate mode
#define DEVICE_ADDRESS_SETTING 0x4  //address to access setting in EEPROM to determine our device address
#define DEVICE_ADDRESS_SETTING_LENGTH 12  //number of bytes to read/write in eeprom for our device address



//ASCII CODES USED:

#define DELETE 127    //backspace
#define CARRIAGE_RETURN 13


#define TRUE 1
#define FALSE 0

//Global Variables
uint8_t timeoutGreenLed = 0;
uint16_t maxAddress = MAX_ADDRESS;  //max dmx address avaliable to send data to
uint8_t txDmxData[MAX_ADDRESS] = {0};  //data to send for each individual dmx address (512 of them default)
uint8_t rxDmxData[MAX_ADDRESS] = {0};  //data to recieve for each individual dmx address (512 of them default)
uint16_t txPhase; //indicates what state of transmitting we are in for transmitting
uint16_t rxPhase; //indicates what state of transmitting we are in for receiving
bool send = TRUE;  // indicates whether DMX stream is sent continuously or not
bool controllerMode = FALSE;  //indicates whether the device is in controller mode
                      //if not it is in device mode, this variable is based of data
                      //in EEPROM, default is in device mode but will
                      //be overwritten if there exists a setting in EEPROM
bool redLedOn = FALSE; //indicates if red led was on or not
bool greenLedOn = FALSE;  //indicates if green led was on or not

uint16_t deviceAddress = 1;//address used to access this device when in device mode
                           //default is 1 but will be overwritten based of setting in EEPROM



//**************************************************************************
// Subroutines
//**************************************************************************



void putcUart0(char c)
{
    //V used for flow control because can write to registers alot faster than
    //uart can transfer it.
    //UART0 flag register & uart flag register transfer fifo full
    //UART flag register remains set until data is done being transfered. pg 897 datasheet
    //checking bit 5, if bit 5 set buffer is full and shouldn't write into data register
    //so while fifo is full keep waiting, because if keep writing data into register
    //going to lose it, if buffer not full will send it out 1 char at a time.
    while (UART0_FR_R & UART_FR_TXFF);

    //V data register we want to trasmit just put in register
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}






//**************************************************************************
// Configures Hardware
//**************************************************************************
void initHw()
{
    uint32_t eepromData;
    uint32_t ui32EEPROMInit;



    //V Configure HW to use 16 MHz crystal| use of main oscillator|
    //V use system divider | system divider specified as 4 + 1 = 5
    //V (1 always added to value we configure for sysdivider)
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    //V Set GPIO ports to use advanced peripheral bus not high speed
    SYSCTL_GPIOHBCTL_R = 0;

    //V power on port F, A and C
    SYSCTL_RCGC2_R = (SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC);

    //set pin pc5 and pc6 to gpio mode and = 0
    //set pc5 and pc6 to output
    GPIO_PORTC_DIR_R = 96; //binary 01100000
    GPIO_PORTC_DR2R_R = 96;  //binary 011000000 drive strength 2mA
    GPIO_PORTC_DEN_R = 112;//1110000 pc6, pc5,pc4


    //V pin 1 is an output others are inputs
    GPIO_PORTF_DIR_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    //V set drive strength to 2mA pin 1
    GPIO_PORTF_DR2R_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    //V digital enable, enable pin 1
    GPIO_PORTF_DEN_R = RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;

    //V clear LED make sure its off
    RED_LED = 0;
    GREEN_LED = 0;
    BLUE_LED = 0;

    //digital enable V
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity



    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    //^alternative function select bit pg 671 datasheet
    //3 in binary is 0011 so pin 0 and 1 of port a
    //are given another function to be determined
   //by the gpio port control register(GPIOPCTL)
    //gpio port control pg 688 datasheet
    //= 10000 | 0001 = 10001
    //controls the config for GPIO PA0 and PA1
    //pg 1351 signal configurations
    //pa0 = 1 so function call is U0Rx UART RECIEVER (pg 895)
    //pa1 = 1 configure for U0Tx uart 0 transmit(pg 895)
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;


    SYSCTL_RCGCUART_R |= (SYSCTL_RCGCUART_R0 | SYSCTL_RCGCUART_R1);
    // pg 344 datasheet ^
    // turn on UART0 and UART1, leave other uarts in same status
    //uart runmode clock gating control
    //place a 1 in position 0 of 8 to enable uart module 0
    //V Configure UART 0 pins
        // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
        //fastest normal baudrate is 115200. means 115200 bits per sec is transfered
        //baudrate usually chosen based on how fast we need to send data and what
        //the system clock can handle sending and recieving while the system still being stable
        //system clock determines how fast we can send data(the baudrate)
        //8*N*1 means for every 8 bits of data 10 are sent over the link
        //1 start bit 8 bits of data and 1 stop bit
        // V uart interger baud rate divisor

    //delay after initialize uart
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");



    UART0_CTL_R = 0;                 // turn off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz) to clock the uart

    // V uart interger baud rate divisor
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    //V uart fractional baud rate divisor
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45

    //V want 8 bit data on here (line control register length = 8) |
    // 16 lvl fifo buffer enabled set bit.
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16*level FIF

    //V turn on transmitter, reciever and whole uart module.
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module


    //configure eeprom functions from Texas Instruments Incorporated sysctl.h
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);  //enable clock eeprom0

    // Wait for the EEPROM module to be ready function from Texas Instruments Incorporated sysctl.h
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0));

    ui32EEPROMInit = EEPROMInit();//function from Texas Instruments Incorporated eeprom.h

    if(ui32EEPROMInit == EEPROM_INIT_ERROR)
    {
        putsUart0("EEPROM ERROR \r\n");
        while(1);  //continous loop so can debug error
    }

    //check if in controller mode or device mode
    EEPROMRead(&eepromData,MODE_ADDRESS_SETTING,MODE_ADDRESS_SETTING_LENGTH); //function from Texas Instruments Incorporated eeprom.h

    if((eepromData) == CONTROLLER_MODE )
    {
        controllerMode = TRUE;
    }
    else if((eepromData) == DEVICE_MODE)
    {
        controllerMode = FALSE;
    }

    //check if device address exists in EEPROM
    EEPROMRead(&eepromData,DEVICE_ADDRESS_SETTING,DEVICE_ADDRESS_SETTING_LENGTH); //function from Texas Instruments Incorporated eeprom.h
    deviceAddress = eepromData;


    //set up timer 1:
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;  //turn on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;  //turn off timer before configure
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;  //32 bit timer a+b
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;  //periodic mode (count down)
    TIMER1_IMR_R = TIMER_IMR_TATOIM;  //turn on interrupts
    NVIC_EN0_R |= 1 <<(INT_TIMER1A-16);  //turn on interrupt 37


    // Timer 2 Config for command received led green
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // turn on timer
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R |= TIMER_CFG_32_BIT_TIMER;          // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R |= TIMER_TAMR_TAMR_PERIOD;         //set to periodic mode (count down)
    TIMER2_TAILR_R = 8000000;                          //200ms interrupts
    TIMER2_IMR_R |= TIMER_IMR_TATOIM;                //turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER2A-16);
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer


}


//**************************************************************************
//elapse approximately 1 microsecond of time
//inputs:
//    us number of microseconds to wait
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
//get a char from terminal
//inputs:
//  none
//outputs:
//  char recieved from uart
//************************************************************************
char getcUart0()
{

    //need flow control because can read data alot faster than can recieve.
    //(flag register & recieve fifo empty bit)
    //only read if something is in fifo because that means there is something
    //new otherwise keep looping
    while (UART0_FR_R & UART_FR_RXFE);

    //V read data register
    return (UART0_DR_R & 0xFF); // only return lower 8 bits ignore errors
    //return UART0_DR_R;
}




//************************************************************************
//used to get string from terminal in array
//inputs:
//  *str will be the array that will hold the string from terminal
//  maxChars is max size for *str
//outputs:
//  *str will be complete string from terminal
//  char that is size of str
//************************************************************************
uint8_t getStringUart0(char *str, uint8_t maxChars)
{
    uint8_t charPosition = 0;  // num of char in string we are currently processing
    uint8_t currentChar;  //current char we are processing
    while(true)
    {
        currentChar = getcUart0();
        putcUart0(currentChar);
        if (currentChar == DELETE)  //if user presses backspace
        {
            if (charPosition > 0)  //only deincrement counter if exists
            {
                charPosition = charPosition - 1;

                //delete previous character replace with space
                putcUart0('\b');
                putcUart0(' ');
                putcUart0('\b');
            }

        }
        else if(currentChar == CARRIAGE_RETURN)  //enter was pressed
        {

            putsUart0("\n\r");
            str[charPosition] = 0;//add null terminator

            if(controllerMode == TRUE)  //in controller mode blink green led when carriage return recieved
            {

                RED_LED = 0;
                GREEN_LED = 1;
                greenLedOn = TRUE;
                timeoutGreenLed = 4; // 4 * 200ms = 800ms


            }

            return charPosition;  //exit
        }
        else if(currentChar >= ' ')  //is it a printable character?
        {
            currentChar = tolower(currentChar);  //set to lowercase
            str[charPosition++] = currentChar;  //store char in string

            if (charPosition == maxChars)  //indicates string can only hold 1 more char
            {

               str[charPosition] = 0;//add null terminator
               putsUart0("\r\n");
               return charPosition; //exit
            }
        }

    }
}

//**************************************************************************
//converts all delimiters in string to nulls and
//parses string based on following rules:
//position of command and arguements are saved based on the events:
//delimiter to letter a*z ('a')
//delimiter to number 0*9 ('n')
//delimiters are anything that is not a number or letter

//inputs:
//  *str, string to parse
//  pos[] array to hold position of important stuff
//  *fieldCount size of pos[] array
//  type[] what type is the object in pos[]
//outputs:
//  pos[] position of important stuff in *str (always indicates start of # if more
//  than one digit)
//  *fieldCount size of pos[] and type[] array
//  type[x] what type is the important stuff in *str at pos[x]
//**************************************************************************
void parseStringUart0(char *str, uint8_t pos[], uint8_t *fieldCount,char type[],uint8_t strSize)
{
    uint8_t i;
    *fieldCount = 0;

    for(i = 0; i < strSize; i++)  //replace delimiters with nulls
    {
        if( ( (str[i] < 'a') || (str[i] > 'z') ) )
        {
            if (((str[i] < '0') || (str[i] > '9')))  //means it is a delimiter
            {
                str[i] = 0;
            }
        }

    }

    for(i = 1; i < strSize; i++)  //start at 2nd char, find important char transitions
    {
        if( (str[i-1] == 0) && (str[i] != 0) )  //means important info as unimportant things are not 0
        {
            if( (str[i] >= 'a') && (str[i] <= 'z') )  //type is letter 'a' in alphabet
            {
                type[((int)*fieldCount)] = 'a';
            }
            else  //type is number 'n'
            {
                type[((int)*fieldCount)] = 'n';
            }
            pos[((int)*fieldCount)] = i;
            (*fieldCount)++;
        }
    }

}


//**************************************************************************
//determines if user typed in command held in cmdName, that it contains
//the min number of arguements needed for the command and that
//the arguements are valid (of type number and not letter)
//inputs:
//  *cmdName  command to determine if user typed in
//  numArgs number of arguements this command needs to have
//  userInput what user typed in from terminal
//  fieldCount number of important information in userInput, size of pos[] or type[]
//  argIndex index of where the arguements begin in userInput
//  inputSize size of userInput
//returns:
//  bool value true or false
bool isCommand(char *cmdName, uint8_t numArgs,char *userInput, uint8_t fieldCount,uint8_t argIndex, uint8_t inputSize)
{
    uint8_t i;
    if(numArgs > 0)  //check that arguements are numbers and not letters
    {


        for(i = argIndex; i < inputSize; i++)  //make sure there are no letters in arguement
        {
            if((userInput[i] < '0') || (userInput[i] > '9'))
            {
                if((userInput[i] != 0))//if a not a delimiter then its a letter
                {
                    return 0;
                }

             }

         }


    }

    return((strcmp(userInput,cmdName) == 0) && ((fieldCount) == numArgs));
    //strcmp compares up to terminating null so this works to compare the command
    //the user typed in and leave the arguements out of it
    //ex userInput get012 0 is terminating null
    }
//**************************************************************************
//returns interger value from string based on argNum
//inputs:
//  argNum is which arguement to get from string
//  string is user input
uint16_t getValue(uint8_t argNum, char *string)
{
   //send part of string starting at where the arguement begins
   return atoi(&(string[argNum]));

}

//**************************************************************************
//  given integer converts to string  and formats correctly
 //does not work with negative numbers
//inputs:
//  num number to convert to string
//  *str string to hold output
//  length of str
//outputs
//  str string of converted number
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

    if(numToConvert != 0)  //only do if num to convert isnt 0
    {
        i = 0;
        while((str[i]) == '0')//counts 0 at beginning of string
        {
            j++;
            i++;
        }

        for(i=0; i<(length-j); i++)//moves string over in array
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
void loadTimer1(uint32_t TimeValue)// load timer1 with a value
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;  //turn of timer
    TIMER1_TAILR_R = TimeValue;  //load timer
    TIMER1_CTL_R |= TIMER_CTL_TAEN;  //turn on timer
}

void disableUart1()
{
    UART1_CTL_R = 0; //turn off uart1
    UART1_IM_R = 0;  //turn off transmit buffer

}


void startDataTransmission()  //starts transmission of data in controller mode
{

        if((txPhase == 0) && (send == TRUE) )
        {
            redLedOn = TRUE;
            if (greenLedOn == FALSE)
            {
                RED_LED = 1;  //indicates data is being transmitted
            }

            TRANSMITTER_ENABLE_PC6 = 1;  //DE pin setting to 1 enables tranmission

            disableUart1();
            //remove transmit and recieve configs for
            //pc4 and pc5
            GPIO_PORTC_PCTL_R &= ~ GPIO_PCTL_PC5_U1TX;
            GPIO_PORTC_PCTL_R &= ~ GPIO_PCTL_PC4_U1RX;
            GPIO_PORTC_AFSEL_R &= 207; //clear bits 4 and 5 leave others the same
            UART1_PC5_TX = 0;  //Transmit pin set to 0 sending a break

            loadTimer1(7040);//load value of 7040
                            //7040/40mhz = .000176 or 176us
            txPhase = 1;


        }

}
void processData()  //process rxData send to devices , should be less than 44us so can keep
{                   //receiving data around 1600 clocks

    if(deviceAddress == 0)
    {
        deviceAddress = 1;
    }

    if(rxDmxData[deviceAddress-1] < 125)
    {
        BLUE_LED = 0;
    }
    else if(rxDmxData[deviceAddress-1] >= 125)
    {
        BLUE_LED = 1;
    }
}
void configUart1()//configure uart1 for transmission or receiving
{
    GPIO_PORTC_AFSEL_R |= 48;

    // enable TX, RX, and module
    GPIO_PORTC_PCTL_R |= (GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX);
    //NEED |= OR WILL RUIN JTAG PINS AND CAN NO LONGER CONNECT

    UART1_CTL_R = 0;     // turn off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz) to clock the uart

    // Configure UART1 to 250000 baud

    UART1_IBRD_R = 10; // ((250000*16) = 4mhz, 40mhz/4mhz = 10 no remainder serial.c example evernote

    //V uart fractional baud rate divisor
    UART1_FBRD_R = 0;


    if(controllerMode == TRUE)
    {
        //ENABLE UART1 ON PIN BASED ON DMX512-A PROTOCOL

        //V want 8 bit data, 1 startbit and 2 stop bits (line control register length = 8)
               // as the 2 stop bits are make after break and break
               // 16 lvl fifo buffer enabled set bit.
        UART1_LCRH_R |= (UART_LCRH_WLEN_8 | UART_LCRH_STP2 |UART_LCRH_FEN);
           // configure for 16*level FIFO 8 data
           //enables 2 stop bits after frame(8 bits in this case)

            //V turn on transmitter and whole uart module.
            UART1_CTL_R = (UART_CTL_TXE  | UART_CTL_UARTEN | UART_CTL_EOT);
            // enable TX, module
            //EOT enabled cause interupt to fire as soon as fifo is empty

            //turn on transmit buffer interrupt
             UART1_IM_R |= UART_IM_TXIM;
             UART1_IM_R &= ~UART_IM_RXIM;  //turn off receive interrupt
             UART1_IM_R &= ~UART_IM_BEIM;  //turn-off break interrupt
             NVIC_EN0_R |= 1 <<(INT_UART1-16);  //turn on interrupt 22

    }
    else  //else we are in device mode
    {
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;

        //V turn on reciever and whole uart module.
        UART1_CTL_R = (UART_CTL_RXE| UART_CTL_UARTEN | UART_CTL_TXE);


        UART1_IM_R &= ~UART_IM_TXIM;  // turn off transmit interrupt
        UART1_IM_R |= UART_IM_RXIM;  //turn on receive interrupt


        NVIC_EN0_R |= 1 <<(INT_UART1-16);  //turn on interrupt 22
        timeoutGreenLed = 10; //2 sec interrupts if break not detected green led should blink
        GREEN_LED = 1;  //dmx signal detected
        greenLedOn = TRUE;
    }
    //delay 4 cycles after initialize uart
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");
    __asm(" NOP ");


}



void Uart1Isr(void)
{
    if(controllerMode == TRUE)
    {
        UART1_ICR_R |= UART_ICR_TXIC;  //clear transmit interrupt flag
        if(txPhase == 3)  //when sending 1 start bit of logic 0 is sent(automatically) 8 data bits of data(configured)
        {                //and 2 stop bits of logic 1 (configured)

            while((txPhase-3)< maxAddress)
            {

                    while (UART1_FR_R & UART_FR_TXFF);

                    UART1_DR_R = txDmxData[(txPhase-3)];
                    txPhase++;
            }


            while((UART1_FR_R & UART_FR_BUSY) == UART_FR_BUSY);  //wait while uart is busy
            txPhase = 0;

            startDataTransmission();

        }
    }
    else  //else we are in device mode
    {

        if((UART_DR_BE & UART1_DR_R))  //if break error has occured
        {
            rxPhase = 1;

            processData();
            UART1_ICR_R = UART_ICR_RXIC;


        }
        else if(rxPhase == 1)
        {
            timeoutGreenLed = 10; //2 sec interrupts if break not detected green led should blink
            //if data keeps being detected renew timeout

            if ((UART1_DR_R & UART_DR_DATA_M) == 0) //and with data register to ignore error flags
            {
                //received start code move on to next phase
                rxPhase = 2;
            }
        }
        else if ((rxPhase >= 2) && ((rxPhase - 2) < maxAddress))  //recieve data put in array
        {

            rxDmxData[(rxPhase - 2)] = (UART1_DR_R & UART_DR_DATA_M);
            rxPhase++;

        }
        if ((rxPhase - 2) == 512)  //start phase over
        {
            rxPhase = 0;

        }

    }


}
//**************************************************************************
//when timer reachs 0 it will jump to here must be configured in startup_ccs.c
void Timer1Isr()
{
    TIMER1_ICR_R = TIMER_ICR_TATOCINT; //clear interrupt flag
    if(txPhase == 1)
    {

        UART1_PC5_TX = 1; //sending make after break
        loadTimer1(480);//load value
                        //480/40mhz = .000012 or 12us
        txPhase = 2;
    }
    else if(txPhase == 2)
    {
        UART1_PC5_TX = 0; //turn off pin before going back to uart to avoid errors in data tranmission

        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;  //turn off timer

        configUart1();

        txPhase = 3;
        UART1_DR_R = 0;  //start character for uart transmission

    }
}

void Timer2Isr()  //configured for 200ms interrupts used for blinking leds
{
    TIMER2_ICR_R |= TIMER_ICR_TATOCINT;  //clear interrupt flag

    if(timeoutGreenLed > 0)
    {
        timeoutGreenLed --;
    }
    else if(controllerMode == FALSE)  //device mode need to toggle led
    {
        timeoutGreenLed = 3; //will turn off/on green led every 600ms
        GREEN_LED = ~GREEN_LED;
        if(GREEN_LED == 1)
        {
            greenLedOn = TRUE;
        }
        else
        {
            greenLedOn = FALSE;
        }
    }
    else  //controller mode
    {
        GREEN_LED = 0;
        greenLedOn = FALSE;

        if(redLedOn == TRUE) // turn red led back on used to avoid green and red being on at same time
        {
            RED_LED = 1;
        }

    }

}



//**************************************************************************
// Main
//**************************************************************************


int main(void)
{
    char input[MAX_CHARS + 1];  //add 1 for terminating null
    uint8_t pos[MAX_CHARS];  //position of important stuff in array
    char type[MAX_CHARS];  //type of important stuff in array
    char tempStr[5];  //used to hold conversions from string to integer
    uint8_t tempStrSize = (sizeof(tempStr)/sizeof(tempStr[0]));
    uint8_t fieldCount = 0;  //size of pos and type arrays
    uint8_t sizeOfInput = 0;  //length of input includeing terminating null
    bool commandExecuted = FALSE; //indicates if a command has been executed
    uint16_t i = 0;
    uint16_t address = 0; //dmx address x to send data to if in controller mode
                         //device modes address if in device mode
    uint16_t data = 0; //data that address x will contain
    uint32_t eepromData;  //temp variable store data to read/write to EEPROM

    initHw();
    txPhase = 0;


    //step 1 on powerup flash red led for 1s

    RED_LED = 1;
    waitMicrosecond(1000000); //wait 1s
    RED_LED = 0;
    waitMicrosecond(1000000); //wait 1s



    while(TRUE)
    {

        if(controllerMode == TRUE)
        {
            putsUart0("Configured as controller\r\n");
        }
        else
        {
            putsUart0("Configured as device\r\n");
        }

        while(true)
        {
            commandExecuted = FALSE;
            if((controllerMode == TRUE))
            {

                txPhase = 0;
                startDataTransmission();

            }
            else  //we are in device mode
            {

                TRANSMITTER_ENABLE_PC6 = 0;
                configUart1();
                if(timeoutGreenLed == 0)
                {
                    timeoutGreenLed = 10; //2 sec interrupts if break not detected green led should blink

                }
            }

            putsUart0("Type in your command\r\nEverything other than characters and letters will be ignored.\r\n");


            sizeOfInput = getStringUart0(input, MAX_CHARS);
            parseStringUart0(input,pos, &fieldCount,type,sizeOfInput);

            //commands specified in project requirements with num of arguements
            if(controllerMode == TRUE)  //the following commands we can run in controller mode only
            {
                if((isCommand("clear", 0, input, fieldCount,pos[0],sizeOfInput)))
                {
                    commandExecuted = TRUE;
                    for(i = 0; i < maxAddress; i++)
                    {
                        txDmxData[i] = 0;
                    }
                    putsUart0("Dmx Data Cleared Successfully\r\n");
                }
                else if((isCommand("set", 2, input, fieldCount,pos[0],sizeOfInput)))
                {
                    commandExecuted = TRUE;
                    address = getValue(pos[0],input);


                    if( (address == 0))  //did not check - numbers as impossible to input them
                    {
                        putsUart0("Address value must be greater than 0");
                        putsUart0("\r\n");
                        break;
                    }

                    data = getValue(pos[1],input);

                    //range check 0 to 255

                    if(data > MAX_DATA)  //did not check - numbers as impossible to input them
                    {
                        putsUart0("Data value exceeds the max allowable value of ");
                        myitoa(MAX_DATA,tempStr,tempStrSize);
                        putsUart0(tempStr);
                        putsUart0("\r\n");
                        break;
                    }

                    txDmxData [address-1] = data;

                }
                else if((isCommand("get", 1, input, fieldCount,pos[0],sizeOfInput)))
                {
                    commandExecuted = TRUE;
                    address = getValue(pos[0],input);
                    if((address == 0))
                    {
                        putsUart0("Address value must be greater than 0");
                        putsUart0("\r\n");
                        break;
                    }
                    putsUart0("Value is ");
                    myitoa((txDmxData[address-1]),tempStr,tempStrSize);
                    putsUart0(tempStr);
                    putsUart0("\r\n");

                }
                else if((isCommand("on", 0, input, fieldCount,pos[0],sizeOfInput)))
                {
                    commandExecuted = TRUE;
                    send = TRUE;

                    redLedOn = TRUE;
                    if(greenLedOn == FALSE)
                    {
                        RED_LED = 1;  //indicates data is being transmitted
                    }

                    putsUart0("DMX stream set to be sent continuously\r\n");
                    TRANSMITTER_ENABLE_PC6 = 1;

                }
                else if((isCommand("off", 0, input, fieldCount,pos[0],sizeOfInput)))
                {
                    commandExecuted = TRUE;
                    send = FALSE;
                    RED_LED = 0; //data not being transmitted
                    redLedOn = FALSE;
                    putsUart0("DMX stream set to not be sent\r\n");
                    TRANSMITTER_ENABLE_PC6 = 0;

                }
                else if((isCommand("max", 1, input, fieldCount,pos[0],sizeOfInput)))
                {
                    commandExecuted = TRUE;
                    maxAddress = getValue(pos[0],input);

                   if(((getValue(pos[0],input)) > MAX_ADDRESS) || ((getValue(pos[0],input)) == 0))  //did not check - numbers as impossible to input them
                    {
                        putsUart0("Max address value is not in the range of 1 to ");
                        myitoa(MAX_ADDRESS,tempStr,tempStrSize);
                        putsUart0(tempStr);
                        putsUart0("\r\n");
                        break;
                    }


                    putsUart0("New max address updated\r\n");

                }
            }
            if((isCommand("device", 0, input, fieldCount,pos[0],sizeOfInput)))
            {
                commandExecuted = TRUE;
                eepromData = DEVICE_MODE;

                EEPROMProgram(&eepromData,MODE_ADDRESS_SETTING,MODE_ADDRESS_SETTING_LENGTH); //function from Texas Instruments Incorporated eeprom.h

                controllerMode = FALSE;
                RED_LED = 0;  //not transmitting
                redLedOn = FALSE;
                putsUart0("Device mode enabled\r\n");


            }
            else if((isCommand("controller", 0, input, fieldCount,pos[0],sizeOfInput)))
            {
                commandExecuted = TRUE;
                eepromData = CONTROLLER_MODE;

                EEPROMProgram(&eepromData,MODE_ADDRESS_SETTING,MODE_ADDRESS_SETTING_LENGTH); //function from Texas Instruments Incorporated eeprom.h

                controllerMode = TRUE;

                redLedOn = TRUE;
                if(greenLedOn == FALSE)
                {
                    RED_LED = 1;  //transmitting

                }

                putsUart0("Controller mode enabled\r\n");

            }
            else if((isCommand("address", 1, input, fieldCount,pos[0],sizeOfInput)) && (controllerMode == FALSE))
            {
                commandExecuted = TRUE;
                address = getValue(pos[0],input);

                if((address > maxAddress) || (address == 0))  //did not check negative numbers as impossible to input them
                {
                    putsUart0("Address value is not in the range of 1 to ");
                    myitoa(maxAddress,tempStr,tempStrSize);
                    putsUart0(tempStr);
                    putsUart0("\r\n");
                    break;
                }
                deviceAddress = address;

                eepromData = deviceAddress;
                EEPROMProgram(&eepromData,DEVICE_ADDRESS_SETTING,DEVICE_ADDRESS_SETTING_LENGTH); //function from Texas Instruments Incorporated eeprom.h



            }
            else if(commandExecuted == FALSE)
            {
                putsUart0("Error command not understood\r\n");
            }
        }

    }
}
