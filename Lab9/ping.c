/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping.h"
#include "Timer.h"
#include "driverlib/interrupt.h"

// Global shared variables - only used within ping.c
volatile enum {LOW, HIGH, DONE} state = LOW; // State of ping echo pulse
volatile unsigned int g_start_time  = 0;  // Timer value at rising edge
volatile unsigned int g_end_time  = 0;  // Timer value at falling edge
volatile unsigned int overflow_count = 0;  // Count of timer overflows

void ping_init(void)
{
    // Enable clock to GPIO port B and Timer 3
    SYSCTL_RCGCGPIO_R |= 0x02;  // enable clock to Port B (bit 1)
    SYSCTL_RCGCTIMER_R |= 0x08;  // enable clock to Timer 3 (bit 3)

    // Wait for GPIOB and Timer3 peripherals to be ready
    while ((SYSCTL_PRGPIO_R & 0x02) == 0)
    {
    };
    while ((SYSCTL_PRTIMER_R & 0x08) == 0)
    {
    };

    // Enable digital I/O on PB3
    GPIO_PORTB_DEN_R |= 0x08;  // PB3

    // Set up PB3 as Timer3B capture pin
    GPIO_PORTB_PCTL_R &= 0xFFFF0FFF;  // Clear PCTL bits for PB3
    GPIO_PORTB_PCTL_R |= 0x00007000;  // Set PB3 as T3CCP1 (function 7)

    // Disable Timer3B during configuration
    TIMER3_CTL_R &= ~0x0100;  // Clear bit 8 to disable Timer3B

    // Configure Timer3B
    TIMER3_CFG_R = 0x04;  // 16-bit timer configuration

    // Configure Timer3B for input capture mode
    TIMER3_TBMR_R = 0x07;  // Capture mode (bits 0-1=3), edge-time mode (bit 2=1)
    TIMER3_TBMR_R &= ~0x10;  // Count down (clear bit 4)

    // Configure to capture both edges
    TIMER3_CTL_R |= 0x0C00;  // Bits 10-11 = 3 to capture both edges

    // Set up timer values for 24-bit timer
    TIMER3_TBILR_R = 0xFFFF;  // Maximum load value for 16-bit timer
    TIMER3_TBPR_R = 0xFF;     // Maximum prescale value for additional 8 bits

    // Enable interrupt for Timer3B capture events
    TIMER3_IMR_R |= 0x400;  // Bit 10 enables capture event interrupt

    // Enable Timer3B
    TIMER3_CTL_R |= 0x0100;  // Set bit 8 to enable Timer3B

    // Register and enable the interrupt handler
    IntRegister(INT_TIMER3B, TIMER3B_Handler);
    NVIC_EN1_R |= 0x10;  // Bit 4 in EN1 enables interrupt for Timer3B

    // Enable global interrupts
    IntMasterEnable();
}

void ping_trigger(void)
{
    // Disable Timer3B interrupt during trigger
    TIMER3_IMR_R &= ~0x400;  // Disable capture event interrupt

    // Reset state for new measurement
    state = LOW;

    // Disable alternate function to use PB3 as GPIO
    GPIO_PORTB_AFSEL_R &= ~0x08;

    // Configure PB3 as output for trigger pulse
    GPIO_PORTB_DIR_R |= 0x08;

    // Generate trigger pulse (LOW-HIGH-LOW)
    GPIO_PORTB_DATA_R &= ~0x08;  // Set PB3 low
    GPIO_PORTB_DATA_R |= 0x08;   // Set PB3 high
    timer_waitMicros(5);         // 5µs trigger pulse
    GPIO_PORTB_DATA_R &= ~0x08;  // Set PB3 low

    // Configure PB3 as input for capture
    GPIO_PORTB_DIR_R &= ~0x08;

    // Enable alternate function to use PB3 as timer input
    GPIO_PORTB_AFSEL_R |= 0x08;

    // Clear any pending interrupts
    TIMER3_ICR_R = 0x400;

    // Re-enable Timer3B capture interrupt
    TIMER3_IMR_R |= 0x400;
}

void TIMER3B_Handler(void)
{
    // Check if this is a capture event
    if ((TIMER3_MIS_R & 0x400) == 0x400) {
        // Clear the interrupt
        TIMER3_ICR_R = 0x400;

        // Process based on current state
        if (state == LOW) {
            // Rising edge detected
            g_start_time  = TIMER3_TBR_R;
            state = HIGH;
        }
        else if (state == HIGH) {
            // Falling edge detected
            g_end_time  = TIMER3_TBR_R;
            state = DONE;
        }
    }

    // Check for timer overflow
    if ((TIMER3_MIS_R & 0x100) == 0x100) {
        // Clear the timer overflow interrupt
        TIMER3_ICR_R = 0x100;
        overflow_count++;
    }
}

float ping_getDistance(void)
{
    // Wait until a complete echo pulse has been measured
    while (state != DONE) {
        // Wait for capture to complete
    }

    // Calculate pulse width in clock cycles
    unsigned int time;

    // Check for timer overflow
    if (g_end_time  > g_start_time ) {
        // Timer wrapped around
        time = (g_start_time  + 0xFFFFFF) - g_end_time; // caps
    } else {
        // Normal case
        time = g_start_time  - g_end_time;
    }

    // Convert to distance in cm
    float distance;

    distance = time * 0.0010625;

    return distance;
}

// Additional functions to provide read-only access to pulse information
unsigned int ping_getPulseTime(void)
{
    // Wait for pulse to be complete
    while (state != DONE) {
    }

    // Calculate pulse width in clock cycles
    if (g_end_time  > g_start_time ) {
        // Timer wrapped around
        return (g_start_time  + 0xFFFFFF) - g_end_time;
    } else {
        // Normal case
        return g_start_time  - g_end_time;
    }
}

float ping_getPulseMillis(void)
{
    // Convert pulse time to milliseconds
    return ping_getPulseTime() * 0.0000625;
}

unsigned int ping_getOverflowCount(void)
{
    return overflow_count;
}
