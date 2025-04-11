/**
 * @file lab9_main.c
 * @author Your Name
 * Main file for CprE 288 Lab 9 - PING))) sensor distance measurement
 */
#include "Timer.h"
#include "lcd.h"
#include "ping.h"
#include "cyBot_Scan.h"  // For CyBot scanner functions
#include "open_interface.h"  // For Open Interface functions

#define LEFT_CAL 1177750
#define RIGHT_CAL 264250

int main(void) {
    // Initialize hardware components
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    ping_init();

    // Initialize the CyBot scanner
    cyBOT_init_Scan(0b0111);  // Initialize servo and other components

    // Position the servo to face forward (90 degrees)
    cyBOT_Scan_t scan;
    cyBOT_Scan(90, &scan);

    // Initialize Open Interface
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    // Main measurement loop
    while(1)
    {
        // Trigger the PING))) sensor
        ping_trigger();

        // Get the distance measurement
        float distance_cm = ping_getDistance();

        // Get pulse information using helper functions
        unsigned int pulse_width_cycles = ping_getPulseTime();
        float pulse_width_ms = ping_getPulseMillis();
        unsigned int overflow_count = ping_getOverflowCount();

        // Display the results on LCD
        lcd_clear();
        lcd_printf("Pulse: %u c\n"
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

    // This code is never reached, but included for completeness
    oi_free(sensor_data);
    return 0;
}
