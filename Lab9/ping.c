/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping.h"
#include "Timer.h"
#include "driverlib/interrupt.h"

// Global shared variables
volatile uint32_t g_start_time = 0;
volatile uint32_t g_end_time = 0;
volatile enum
{
    LOW, HIGH, DONE
} g_state = LOW; // State of ping echo pulse

void ping_init(void)
{
    // enable clock to GPIO port B
    SYSCTL_RCGCGPIO_R |= 0x02;  // enable clock to Port B (bit 1) page 340
    SYSCTL_RCGCTIMER_R |= 0x08;  // enable clock to bit 3 for timer

    // wait for GPIOB peripherals to be ready
    while ((SYSCTL_PRGPIO_R & 0x02) == 0)
    {
    };
    while ((SYSCTL_PRTIMER_R & 0x08) == 0)
    {
    };

    // enable digital output for pb3
    GPIO_PORTB_DEN_R |= 0x08;  // pb3

    // Register the interrupt handler
    IntRegister(INT_TIMER3B, TIMER3B_Handler);

    // Enable NVIC for Timer3B
    NVIC_EN1_R |= 0x10;         // bit 4 in EN1 for Timer3B

    IntMasterEnable();

    // Configure and enable the timer
    TIMER3_CTL_R &= 0xFEFF;    // Disable Timer3B (clear bit 8)

    TIMER3_CFG_R = 0x4;        // 16-bit timer configuration

    // Configure Timer3B for input capture mode
    TIMER3_TBMR_R = 0;         // Clear all mode bits first
    TIMER3_TBMR_R |= 0x3;      // Set bits 0-1 for capture mode
    TIMER3_TBMR_R |= 0x4;      // Set bit 2 for edge-time mode

    // Set prescaler to extend timer to 24 bits
    TIMER3_TBPR_R = 0xFF;      // 8-bit prescaler (upper 8 bits)
    TIMER3_TBILR_R = 0xFFFF;   // 16-bit load value (lower 16 bits)

    // Configure to capture both edges
    TIMER3_CTL_R |= 0x0C00;    // bits 10-11 set to 3 (capture both edges)

    // Enable interrupt for capture events
    TIMER3_IMR_R |= 0x400;     // bit 10 enables capture event interrupt

    // Finally, enable Timer3B
    TIMER3_CTL_R |= 0x0100;    // Enable Timer3B (set bit 8)
}

void ping_trigger(void)
{
    // First, disable Timer3B and its interrupt
    TIMER3_CTL_R &= ~0x0100;   // Disable Timer3B
    TIMER3_IMR_R &= ~0x400;    // Disable capture event interrupt

    // Disable alt function on pb3 for part 1
    GPIO_PORTB_AFSEL_R &= ~0x08;

    // set direction as output for pb3
    GPIO_PORTB_DIR_R |= 0x08;

    // Reset state for new measurement
    g_state = LOW;

    // drive pb3 low->high->low
    GPIO_PORTB_DATA_R &= ~0x08;  // ensure low - 0
    GPIO_PORTB_DATA_R |= 0x08;   // drive high - 1
    timer_waitMicros(5);         // 5µs as per datasheet
    GPIO_PORTB_DATA_R &= ~0x08;  // back low - 0

    // Configure PB3 for timer capture (T3CCP1)
    GPIO_PORTB_DIR_R &= ~0x08;         // Set PB3 as input for echo
    GPIO_PORTB_PCTL_R &= ~0x0000F000;  // Clear PCTL bits for PB3
    GPIO_PORTB_PCTL_R |= 0x00007000;   // Set PB3 as T3CCP1 (function 7)

    // re-enable alt function on pb3 for timer
    GPIO_PORTB_AFSEL_R |= 0x08;

    // Clear any pending interrupts
    TIMER3_ICR_R = 0x400;

    // Re-enable Timer3B and its interrupt
    TIMER3_IMR_R |= 0x400;     // Enable capture event interrupt
    TIMER3_CTL_R |= 0x0100;    // Enable Timer3B
}

void TIMER3B_Handler(void)
{
    // Check if capture event triggered the interrupt
    if (TIMER3_MIS_R & 0x400) {
        // Clear the interrupt
        TIMER3_ICR_R = 0x400;

        // Read the captured timer value (24-bit)
        uint32_t timer_value = TIMER3_TBR_R;

        // Update based on current state
        if (g_state == LOW) {
            // Rising edge detected
            g_start_time = timer_value;
            g_state = HIGH;
        }
        else if (g_state == HIGH) {
            // Falling edge detected
            g_end_time = timer_value;
            g_state = DONE;
        }
    }
}

float ping_getDistance(void)
{
    // Wait until echo pulse is captured
    while (g_state != DONE) {}

    // Calculate pulse width
    uint32_t pulse_width;

    // Since we're counting down, check for timer wrap-around
    if (g_start_time < g_end_time) {
        // Timer wrapped around (overflow occurred)
        pulse_width = (0xFFFFFF - g_end_time) + g_start_time + 1;
    } else {
        // No overflow
        pulse_width = g_start_time - g_end_time;
    }

    // Convert to time in milliseconds
    // Each timer tick is 62.5ns (1/16MHz)
    float pulse_time_ms = pulse_width * 0.0000625;

    // Calculate distance in cm
    // Distance = (time in seconds * speed of sound) / 2
    // Speed of sound = 343 m/s = 34300 cm/s
    float distance_cm = pulse_time_ms * 17.15;  // 17.15 = 0.001 * 34300 / 2

    // Reset state for next measurement
    g_state = LOW;

    return distance_cm;
}
