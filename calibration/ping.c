/*
 * ping.c
 *
 *  Created on: Oct 28, 2021
 *      Author: tdbolton
 */

#include "ping.h"
#include "math.h"

volatile enum {LOW, HIGH, DONE} state; // set by ISR
volatile unsigned int rising_time; //Pulse start time: Set by ISR
volatile unsigned int falling_time; //Pulse end time: Set by ISR

void ping_init(void)
{
    SYSCTL_RCGCGPIO_R |= 0b000010;
    GPIO_PORTB_DEN_R |= 0b00001000;
    GPIO_PORTB_PCTL_R &= 0xFFFF0FFF;
    GPIO_PORTB_PCTL_R |= 0x00007000;
    SYSCTL_RCGCTIMER_R |= 0b001000;

    TIMER3_CTL_R &= 0b1111111011111111;
    TIMER3_CFG_R |= 0b100;
    TIMER3_TBMR_R |= 0b000000000111;
    TIMER3_TBMR_R &= 0b111111101111;
    TIMER3_CTL_R |= 0b0000110000000000;
    TIMER3_TBILR_R = 0x0000FFFE;
    TIMER3_TBPR_R = 0x000000FF;
    TIMER3_IMR_R |= 0b010000000000;
    TIMER3_CTL_R |= 0b0000000100000000;

    NVIC_EN1_R |= 0b10000;

    IntRegister(INT_TIMER3B, TIMER3B_HANDLER);
}

void send_pulse(void)
{
    GPIO_PORTB_AFSEL_R &= 0b11110111;
    GPIO_PORTB_DIR_R |= 0b00001000;// Set PB3 as output
    GPIO_PORTB_DATA_R |= 0b00001000;// Set PB3 to high
    timer_waitMicros(10);
    GPIO_PORTB_DATA_R &= 0b11110111;// Set PB3 to low
    GPIO_PORTB_DIR_R &= 0b11110111;// Set PB3 as input
    GPIO_PORTB_AFSEL_R |= 0b00001000;
}

float ping_read(void)
{
    TIMER3_IMR_R &= ~0b010000000000;
    state = LOW;
    send_pulse();
    TIMER3_ICR_R |= 0b010000000000;
    TIMER3_IMR_R |= 0b010000000000;

    while (state != DONE) {

    }

    unsigned int time = rising_time - falling_time;
    if (falling_time > rising_time) {
        time = (rising_time + pow(2,24) - 1) - falling_time;
    }

    float distance = time*0.0010625;

    return distance;
}

void TIMER3B_HANDLER(void)
{
    if((TIMER3_MIS_R & TIMER3_IMR_R) == TIMER3_IMR_R) {
         if(state == LOW){
             rising_time = TIMER3_TBR_R;
             state = HIGH;
         }
         else if(state == HIGH){
             falling_time = TIMER3_TBR_R;
             state = DONE;
         }
    }
    TIMER3_ICR_R |= 0b010000000000;

}



