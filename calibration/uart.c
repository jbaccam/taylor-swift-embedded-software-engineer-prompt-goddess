/*
*
*   uart_extra_help.c
* Description: This is file is meant for those that would like a little
*              extra help with formatting their code, and followig the Datasheet.
*/

#include "uart.h"
#include "timer.h"
#include "stdio.h"
#define REPLACE_ME 0x00
volatile  char flag = 0;       // Your UART interrupt can update this flag


void uart_init(int baud)
{
    SYSCTL_RCGCGPIO_R |= 0b00000010;      // enable clock GPIOB (page 340)
    SYSCTL_RCGCUART_R |= 0b00000010;      // enable clock UART1 (page 344)
    GPIO_PORTB_AFSEL_R = 0b00000011;      // sets PB0 and PB1 as peripherals (page 671)
    GPIO_PORTB_PCTL_R  = 0x00000011;       // pmc0 and pmc1       (page 688)  also refer to page 650
    GPIO_PORTB_DEN_R   = 0b00000011;        // enables pb0 and pb1
    GPIO_PORTB_DIR_R   = 0b00000010;        // sets pb0 as output, pb1 as input

    //compute baud values [UART clock= 16 MHz] 
    double fbrd;
    int    ibrd;

    fbrd = .166667; // page 903
    ibrd = 104;
    fbrd = REPLACE_ME;

    UART1_CTL_R &= 0xFFFFFFFE;      // disable UART1 (page 918)
    UART1_IBRD_R = 0x0008;        // write integer portion of BRD to IBRD
    UART1_FBRD_R = 0x002C;   // write fractional portion of BRD to FBRD
    UART1_LCRH_R = 0b01100000;        // write serial communication parameters (page 916) * 8bit and no parity
    UART1_CC_R   = 0x0;          // use system clock as clock source (page 939)
    UART1_CTL_R |= 0x1;        // enable UART1

}

void uart_sendChar(char data)
{
   //TODO
   while (UART1_FR_R & 0b00100000){

   }
   UART1_DR_R = data;

}



char uart_receive(void)
{
 //TODO
    char data = 0;

    while (UART1_FR_R & 0x00000010){

    }
    data = (char)(UART1_DR_R & 0xFF);
    return data;
 
}

void uart_sendStr(const char *data)
{

    short j  = 0;

    while (*(data+j) != '\0'){
        uart_sendChar(*(data+j));
        j++;
    }
	
}

// _PART3


void uart_interrupt_init()
{
    UART1_CTL_R &= 0xFFFFFFFE;

    UART1_ICR_R &= 0x00000030;
    // Enable interrupts for receiving bytes through UART1
    UART1_IM_R |= 0x00000010; //enable interrupt on receive - page 924

    // Find the NVIC enable register and bit responsible for UART1 in table 2-9
    // Note: NVIC register descriptions are found in chapter 3.4
    NVIC_EN0_R |= 0x00000040; //enable uart1 interrupts - page 104

    // Find the vector number of UART1 in table 2-9 ! UART1 is 22 from vector number page 104
    IntRegister(INT_UART1, uart_interrupt_handler); //give the microcontroller the address of our interrupt handler - page 104 22 is the vector number
    IntMasterEnable();

    UART1_CTL_R |= 0x1;

}

void uart_interrupt_handler()
{
// STEP1: Check the Masked Interrup Status

    if (UART1_MIS_R & 0x00000010){

        uart_data = uart_receive();

        UART1_ICR_R &= 0x00000010;
    }

//STEP2:  Copy the data 

//STEP3:  Clear the interrupT
    flag = 1;

}

void printDouble(double printMe, bool endLine){

    short j = 0;
    char printDistance[10];
    sprintf(printDistance, "%f" ,printMe);
    while(printDistance[j] != 0){
        uart_sendChar(printDistance[j]);
        j++;
    }

    j = 0;

    if (endLine){
        uart_sendChar('\t');
    } else{
        uart_sendChar('\n');
        uart_sendChar('\r');
    }

}

void printInt(int printMe, bool endLine){


    short j = 0;
    char printDistance[10];
    sprintf(printDistance, "%d" ,printMe);
    while(printDistance[j] != 0){
        uart_sendChar(printDistance[j]);
        j++;
    }

    j = 0;

    if (endLine){
        uart_sendChar('\t');
    } else{
        uart_sendChar('\n');
        uart_sendChar('\r');
    }

}

void printStr(const char* printMe, bool endLine){

    short j = 0;

    while(printMe[j] != 0){
        uart_sendChar(printMe[j]);
        j++;
    }

    j = 0;

    if (endLine){
        uart_sendChar('\t');
    } else{
        uart_sendChar('\n');
        uart_sendChar('\r');
    }

}
