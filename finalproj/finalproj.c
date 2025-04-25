/*
 * finalproj.c - Mars Rover (CyBot) Simple Implementation
 *
 * Basic implementation similar to Lab 7:
 * - Scans environment using servo and sensors
 * - Detects objects and identifies the smallest one
 * - Navigates to the smallest object
 *
 * Created on: Apr 25, 2025
 * Author: Team SE2 "Ping Patrol"
 */

#include "adc.h"
#include "timer.h"
#include "lcd.h"
#include "uart.h"
#include "open_interface.h"
#include "ping.h"
#include "servo.h"
#include "button.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

// Threshold for IR detection
#define IR_THRESHOLD 400

// Function prototypes
void scan_environment(oi_t *sensor_data);
void move_to_object(oi_t *sensor_data, int angle, float distance);

void main(void)
{
    // Initialize hardware components
    timer_init();
    lcd_init();
    uart_interrupt_init();
    adc_init();
    ping_init();
    servo_init();
    button_init();

    // Initialize CyBot
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    // Display welcome message
    lcd_printf("Mars Rover\nInitialized");

    // UART communication
    char toPutty[100];
    sprintf(toPutty, "Mars Rover Ready\r\nPress Enter to begin scanning\r\n");
    uart_sendStr(toPutty);

    // Wait for user to press Enter
    char inputCharacter;
    do {
        inputCharacter = uart_receive();
    } while (inputCharacter != '\r');

    // Scan environment and navigate to smallest object
    scan_environment(sensor_data);

    // End program
    oi_free(sensor_data);
}

void scan_environment(oi_t *sensor_data)
{
    int i;
    float pingDistance;
    int IRmeasurement;

    // Arrays to store scan data
    int angles[90];
    float distances[90];
    int count = 0;

    // Display scanning message
    lcd_printf("Scanning...");

    char toPutty[100];
    sprintf(toPutty, "Beginning scan...\r\n");
    uart_sendStr(toPutty);
    sprintf(toPutty, "Angle\tDistance\tIR Value\r\n");
    uart_sendStr(toPutty);

    // Scan from 0 to 180 degrees
    for (i = 0; i <= 180; i += 2) {
        servo_move(i);
        timer_waitMillis(50);  // Wait for servo to settle

        // Using ping functions from your code
        ping_trigger();
        pingDistance = ping_getDistance(); // Using ping_getDistance instead of ping_read
        IRmeasurement = adc_read();

        // If object detected (IR value above threshold)
        if (IRmeasurement > IR_THRESHOLD) {
            // Store object data
            angles[count] = i;
            distances[count] = pingDistance;
            count++;

            // Send data to base station
            sprintf(toPutty, "%d\t%.1f\t%d\r\n", i, pingDistance, IRmeasurement);
            uart_sendStr(toPutty);
        }
    }

    // Process scan results
    if (count > 0) {
        // Find object with shortest distance (simplest approach)
        int closest_idx = 0;
        float min_distance = distances[0];

        for (i = 1; i < count; i++) {
            if (distances[i] < min_distance) {
                min_distance = distances[i];
                closest_idx = i;
            }
        }

        // Display result
        lcd_printf("Object found\nAngle: %d\nDist: %.1f", angles[closest_idx], distances[closest_idx]);

        sprintf(toPutty, "\r\nClosest object found at angle %d, distance %.1f cm\r\n",
                angles[closest_idx], distances[closest_idx]);
        uart_sendStr(toPutty);

        // Move to the closest object
        move_to_object(sensor_data, angles[closest_idx], distances[closest_idx]);
    } else {
        // No objects found
        lcd_printf("No objects\ndetected");
        sprintf(toPutty, "No objects detected\r\n");
        uart_sendStr(toPutty);
    }
}

void move_to_object(oi_t *sensor_data, int angle, float distance)
{
    // Display movement message
    char toPutty[100];
    sprintf(toPutty, "Moving to object at angle %d, distance %.1f cm\r\n", angle, distance);
    uart_sendStr(toPutty);

    // Simple turn to face object
    if (angle < 90) {
        // Turn right
        sprintf(toPutty, "Turning right...\r\n");
        uart_sendStr(toPutty);
        oi_setWheels(-50, 50);
        timer_waitMillis((90 - angle) * 20);  // Simple time-based turning
    } else if (angle > 90) {
        // Turn left
        sprintf(toPutty, "Turning left...\r\n");
        uart_sendStr(toPutty);
        oi_setWheels(50, -50);
        timer_waitMillis((angle - 90) * 20);  // Simple time-based turning
    }

    // Stop turning
    oi_setWheels(0, 0);
    timer_waitMillis(500);

    // Move toward object (stopping 10cm away for safety)
    if (distance > 15) {
        sprintf(toPutty, "Moving forward...\r\n");
        uart_sendStr(toPutty);

        // Convert distance to movement time (simple approximation)
        int travel_distance = (distance - 15) * 10;  // Distance in mm
        int travel_time = travel_distance / 50;      // Time at ~50mm/s

        // Move forward
        oi_setWheels(100, 100);
        timer_waitMillis(travel_time);
    }

    // Stop moving
    oi_setWheels(0, 0);

    // Complete
    sprintf(toPutty, "Reached destination\r\n");
    uart_sendStr(toPutty);
    lcd_printf("Destination\nreached");
}
