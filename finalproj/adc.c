/*
 * adc.c
 *
 *  Created on: Apr 3, 2025
 *      Author: jjbaccam
 */

#include "adc.h"
#include "timer.h"

#include "adc.h"
#include "timer.h"

void adc_init(void){
    // Enable clock for Port B and ADC0
    SYSCTL_RCGCGPIO_R |= 0b00000010;  // Port B clock enable
    SYSCTL_RCGCADC_R  |= 0b01;         // ADC0 clock enable
    timer_waitMillis(1);              // Allow clocks to stabilize

    // Configure PB4 as analog input (for AIN10)
    GPIO_PORTB_AFSEL_R |= 0x10;       // Enable alternate function on PB4
    GPIO_PORTB_DIR_R   &= ~0x10;      // Set PB4 as input
    GPIO_PORTB_DEN_R   &= ~0x10;      // Disable digital function on PB4
    GPIO_PORTB_AMSEL_R |= 0x10;       // Enable analog function on PB4
    GPIO_PORTB_ADCCTL_R = 0x00;       // Clear ADC control bits for Port B

    ADC0_SAC_R = 0x4;                // Set hardware averaging to 16x

    // ADC Configuration for Sample Sequencer 0 (SS0)
    ADC0_ACTSS_R &= ~0x01;            // Disable SS0 for configuration
    ADC0_EMUX_R  &= ~0x000F;          // Clear trigger selection for SS0 (software trigger)
    ADC0_SSMUX0_R = (ADC0_SSMUX0_R & 0xFFFFFFF0) | 0xA;  // Set channel 10 (0xA) for SS0
    ADC0_SSCTL0_R = (ADC0_SSCTL0_R & 0xFFFFFFF0) | 0x06;   // Set IE0 and END0 bits for SS0
    ADC0_ACTSS_R |= 0x01;             // Re-enable SS0
}

int adc_read(void){
    // Initiate a conversion on SS0
    ADC0_PSSI_R = ADC_PSSI_SS0;

    // Wait for conversion to complete
    while (!(ADC0_RIS_R & ADC_RIS_INR0)) { }

    // Read the result from FIFO
    int data = ADC0_SSFIFO0_R;

    // Clear the interrupt flag for SS0
    ADC0_ISC_R = ADC_ISC_IN0;

    // Optional: delay for stability (remove if not needed)
    timer_waitMillis(50);

    return data;
}





