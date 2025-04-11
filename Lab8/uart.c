/*
*   uart-interrupt.c
*
*   Minimal example matching the lab template, for UART1 interrupts at 115200 baud,
*   8 data bits, no parity, 1 stop bit, no FIFO.  The interrupt handler echoes
*   each received character.  No calls to uart_receive() in the main program.
*
*   @author
*   @date
*/

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <uart.h>
#include "driverlib/interrupt.h" // for IntRegister, IntMasterEnable

// Example global variables you can use if implementing a special command:
volatile char command_byte = -1;   // e.g. 's' for stop
volatile int  command_flag = 0;    // set to 1 in ISR if command_byte received

void uart_interrupt_init(void){
    //enable clock to GPIO port B
    SYSCTL_RCGCGPIO_R |= 0x02;  // enable clock to Port B (bit 1) page 340

    //enable clock to UART1
    SYSCTL_RCGCUART_R |= 0x02;  // enable clock to UART1 (bit 1)

    //wait for GPIOB and UART1 peripherals to be ready
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};
    while ((SYSCTL_PRUART_R & 0x02) == 0) {};

    //enable digital functionality on port B pins
    GPIO_PORTB_DEN_R   |= 0x03;  // pb0, pb1

    //enable alternate functions on port B pins
    GPIO_PORTB_AFSEL_R |= 0x03;  // pb0, pb1, page 671

    //enable UART1 Rx and Tx on port B pins
    // for PB0=Rx, PB1=Tx => bits [3:0] and [7:4] = 1 => 0x11
    GPIO_PORTB_PCTL_R |= 0x00000011;

    //calculate baud rate
    // With 16 MHz clock:  Baud = 115,200 => IBRD=8, FBRD=44
    uint16_t iBRD = 8;
    uint16_t fBRD = 44;

    //turn off UART1 while setting it up
    UART1_CTL_R &= ~0x01;  //disable bit0 = UARTEN

    //set baud rate (must be before LCRH write)
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    //set frame, 8 data bits, 1 stop bit, no parity, no FIFO => 0x60
    UART1_LCRH_R |= 0x60; // write serial communication parameters, page 916, * 8bit and no parity

    //use system clock as source
    UART1_CC_R = 0x0;  // use system clock as clock source (page 939)

    //////Enable interrupts

    //first clear RX interrupt flag (clear by writing 1 to bit4 in ICR)
    UART1_ICR_R |= 0x10;  // page 933, bit 4 corresponds to rx interrupt clear, setting it to 1 clears
    // 0x10 -> 0b00010000

    //enable RX raw interrupts in interrupt mask register => also bit4
    UART1_IM_R |= 0x10; // page 924, bit 4 corresponds to the rx interrupt mask,
    // when its 1, An interrupt is sent to the interrupt controller when the RXRIS
    // bit in the UARTRIS register is set.


    //NVIC setup: set priority of UART1 interrupt to 1 in bits 21-23 of PRI1 (page 153)
    // PRI1 is a 32 bit register, and bits [23:21] control interrupt #6’s priority which is uart1
    /*
     Each interrupt can be assigned priority level, when multiple interrupts go off,
     or one is running, the interrupt with a higher priority (lower number) will take over
     We choose a priority by writing to the NVIC_PRIx_R registers
     Each of these registers holds the priorities for four interrupts (page 124)

     NVIC_PRI1_R manages the priorities of interrupt numbers 4 through 7 (page 135)
     uart1 is interrupt 6 (page 104) so its pri1
     */
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

    //NVIC setup: enable interrupt for UART1, IRQ #6 => bit6 in EN0
    // enable 0–31 register, bit 6 is for interrupt #6
    // this means the CPU will accept interrupt requests from UART1
    NVIC_EN0_R |= (1 << 6); //page 134

    //tell CPU to use ISR handler for UART1
    IntRegister(INT_UART1, UART1_Handler);

    //globally allow CPU to service interrupts
    IntMasterEnable();

    //re-enable UART1 and also enable RX, TX (three bits => bit0=UARTEN, bit8=RXE, bit9=TXE)
    UART1_CTL_R = 0x301; // UARTCTL page 918, same thing as 0b1100000001
    /*
     bit0 (UARTEN) = 1: master enable for entire uart
     bit8 (RXE) = 1  enables receive
     bit9 (TXE) = 1: enables transmit

     */
}

// For Lab 6, we typically do not call uart_receive(), so it can be left empty or commented out.
// "DO NOT USE this busy-wait function if using RX interrupt"
char uart_receive(void) {
    // Wait while RXFE (bit 4) is set => FIFO is empty
    while (UART1_FR_R & 0x10) {}
    return (char) (UART1_DR_R & 0xFF);
}

void uart_sendChar(char data){
    // wait while TXFF (bit5)
    while(UART1_FR_R & 0x20){}
    UART1_DR_R = data;
}

void uart_sendStr(const char *data){
    int i=0;
    while(data[i] != '\0'){
        uart_sendChar(data[i]);
        i++;
    }
}

void UART1_Handler(void)
{
    char byte_received;
    //check if handler called due to RX event => bit4 in MIS
    if (UART1_MIS_R & 0x10)  // bit 4 indicates rx interrupt
    {
        //clear the RX trigger flag => write 1 to bit4 in ICR
        // tells the hardware we are handling the interrupt so we can clear the flax
        UART1_ICR_R |= 0x10;

        //read the byte from UART1_DR_R (ignore error bits)
        //read the character from the data register, mask the lower 8 bits so we can ignore the error/status bits
        byte_received = (char)(UART1_DR_R & 0xFF);  // (????? ->  (char)(UART1_DR_R & 0xFF) )

        //echo it back
        uart_sendChar(byte_received);

        //if byte is carriage return => also send newline
        if (byte_received == '\r'){
            uart_sendChar('\n');
        }
        else {
            // As needed: handle special commands
            if (byte_received == command_byte){
                command_flag = 1; // let main know
            }
        }
    }
}
