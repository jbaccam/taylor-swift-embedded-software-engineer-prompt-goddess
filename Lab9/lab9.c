/**
 * @file lab9_main.c
 * @author Your Name
 * Main file for CprE 288 Lab 9 - PING))) sensor distance measurement
 */
#include "Timer.h"
#include "lcd.h"
#include "ping.h"
#include <stdbool.h>

int main(void) {
    // Initialize hardware components
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    ping_init();

    // Variables to track measurements
    uint32_t pulse_width_cycles = 0;
    float pulse_width_ms = 0.0;
    float distance_cm = 0.0;
    unsigned int overflow_count = 0;

    // Main measurement loop
    while(1)
    {
        // Trigger the PING))) sensor
        ping_trigger();

        // Get the distance measurement
        distance_cm = ping_getDistance();

        // Calculate pulse width in clock cycles
        if (g_start_time < g_end_time) {
            // Timer wrapped around (overflow occurred)
            pulse_width_cycles = (0xFFFFFF - g_end_time) + g_start_time + 1;
            overflow_count++; // Increment overflow counter
        } else {
            // No overflow
            pulse_width_cycles = g_start_time - g_end_time;
        }

        // Calculate pulse width in milliseconds
        // Each timer tick is 62.5ns (1/16MHz)
        pulse_width_ms = pulse_width_cycles * 0.0000625;

        // Display the results on LCD
        lcd_clear();
        lcd_printf("Pulse: %u cycles\n"
                   "Time: %.3f ms\n"
                   "Dist: %.1f cm\n"
                   "Oflows: %u",
                   pulse_width_cycles,
                   pulse_width_ms,
                   distance_cm,
                   overflow_count);

        // Wait before next measurement (as specified in lab)
        timer_waitMillis(300);
    }
}
