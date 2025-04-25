/*
 * finalproj.c - Mars Rover (CyBot) Implementation based on Lab 7
 *
 * Comprehensive implementation matching Lab 7:
 * - Scans environment using servo and sensors
 * - Filters and processes sensor data
 * - Detects objects using edge detection
 * - Identifies the smallest width object
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

// Constants for scanning
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define STEP 2                    // Scan every 2 degrees
#define NUM_POINTS (((MAX_ANGLE - MIN_ANGLE) / STEP) + 1)

// Thresholds for edge detection
#define IR_THRESHOLD 400          // Threshold for IR value differences
#define MIN_OBJECT_WIDTH 6.0f     // Minimum width in degrees to consider as an object
#define BOUNDARY_MARGIN 10        // Ignore objects with edges this close to scan limits

// Function prototypes
void get_angle_array(float *angles);
void scan_all_angles(float *angles, float *ping_values, int *ir_values);
void filter_sensor_data(float *ping_values, int *ir_values, float *ping_filtered, int *ir_filtered);
float median_of_3_float(float a, float b, float c);
int median_of_3_int(int a, int b, int c);
void compute_ir_diff(int *ir_filtered, int *ir_diff);
void detect_objects(float *angles, float *ping_filtered, int *ir_filtered, int *ir_diff);
float calculate_linear_width(float radialWidth, float distance);
void navigate_to_smallest_object(oi_t *sensor_data);

// Object information structure
typedef struct {
    float startAngle;
    float endAngle;
    float centerAngle;
    float radialWidth;
    float distance;
    float linearWidth;
} Object;

#define MAX_OBJECTS 10
Object objects[MAX_OBJECTS];
int objectCount = 0;

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
    sprintf(toPutty, "=== Mars Rover - Lab 7 Implementation ===\r\n");
    uart_sendStr(toPutty);
    sprintf(toPutty, "Press Enter to begin scanning\r\n");
    uart_sendStr(toPutty);

    // Wait for user to press Enter
    char inputCharacter;
    do {
        inputCharacter = uart_receive();
    } while (inputCharacter != '\r');

    // Perform scan and navigation
    navigate_to_smallest_object(sensor_data);

    // End program
    oi_free(sensor_data);
}

/**
 * Main navigation function that finds and goes to the smallest width object
 */
void navigate_to_smallest_object(oi_t *sensor_data)
{
    char buffer[100];
    objectCount = 0;

    // Arrays for storing data
    float angles[NUM_POINTS];
    float ping_values[NUM_POINTS];
    float ping_filtered[NUM_POINTS];
    int ir_values[NUM_POINTS];
    int ir_filtered[NUM_POINTS];
    int ir_diff[NUM_POINTS];

    sprintf(buffer, "Performing initial scan...\r\n");
    uart_sendStr(buffer);

    // Get angles array
    get_angle_array(angles);

    // Perform full scan
    scan_all_angles(angles, ping_values, ir_values);

    // Filter data
    filter_sensor_data(ping_values, ir_values, ping_filtered, ir_filtered);

    // Compute differences
    compute_ir_diff(ir_filtered, ir_diff);

    // Detect objects
    detect_objects(angles, ping_filtered, ir_filtered, ir_diff);

    // If no objects found, report and exit
    if (objectCount == 0) {
        sprintf(buffer, "No objects detected.\r\n");
        uart_sendStr(buffer);
        return;
    }

    // Find the object with the smallest linear width
    int smallestIndex = 0;
    float smallestWidth = objects[0].linearWidth;
    int i;
    for (i = 1; i < objectCount; i++) {
        if (objects[i].linearWidth < smallestWidth) {
            smallestWidth = objects[i].linearWidth;
            smallestIndex = i;
        }
    }

    // Get target position
    float targetAngle = objects[smallestIndex].centerAngle;
    float targetDistance = objects[smallestIndex].distance;

    // Display information about the target
    sprintf(buffer, "Found smallest object (Object %d):\r\n", smallestIndex + 1);
    uart_sendStr(buffer);

    sprintf(buffer, "  - Center angle: %.1f degrees\r\n", targetAngle);
    uart_sendStr(buffer);

    sprintf(buffer, "  - Distance: %.1f cm\r\n", targetDistance);
    uart_sendStr(buffer);

    sprintf(buffer, "  - Linear width: %.1f cm\r\n\r\n", objects[smallestIndex].linearWidth);
    uart_sendStr(buffer);

    sprintf(buffer, "Navigating to smallest object...\r\n");
    uart_sendStr(buffer);

    // Move to the object
    lcd_printf("Moving to obj\nAngle: %.1f\nDist: %.1f", targetAngle, targetDistance);

    // Turn to face object
    if (targetAngle < 90) {
        // Turn right
        sprintf(buffer, "Turning right to angle %.1f...\r\n", targetAngle);
        uart_sendStr(buffer);
        oi_setWheels(-50, 50);
        timer_waitMillis((90 - targetAngle) * 20);  // Simple time-based turning
    } else if (targetAngle > 90) {
        // Turn left
        sprintf(buffer, "Turning left to angle %.1f...\r\n", targetAngle);
        uart_sendStr(buffer);
        oi_setWheels(50, -50);
        timer_waitMillis((targetAngle - 90) * 20);  // Simple time-based turning
    }

    // Stop turning
    oi_setWheels(0, 0);
    timer_waitMillis(500);

    // Move toward object (stopping 10cm away for safety)
    if (targetDistance > 15) {
        sprintf(buffer, "Moving forward %.1fcm...\r\n", targetDistance - 15);
        uart_sendStr(buffer);

        // Convert distance to movement time (simple approximation)
        int travel_distance = (targetDistance - 15) * 10;  // Distance in mm
        int travel_time = travel_distance / 50;            // Time at ~50mm/s

        // Move forward
        oi_setWheels(100, 100);
        timer_waitMillis(travel_time);
    }

    // Stop moving
    oi_setWheels(0, 0);

    // Complete
    sprintf(buffer, "Reached destination!\r\n");
    uart_sendStr(buffer);
    lcd_printf("Destination\nreached");
}

// Populate the angles[] array from MIN_ANGLE to MAX_ANGLE in STEP increments
void get_angle_array(float *angles)
{
    float angleVal = MIN_ANGLE;
    int i;
    for (i = 0; i < NUM_POINTS; i++) {
        angles[i] = angleVal;
        angleVal += STEP;
    }
}

// Perform a full scan and collect ping and IR data at each angle
void scan_all_angles(float *angles, float *ping_values, int *ir_values)
{
    char buffer[100];

    sprintf(buffer, "Scanning...\r\n");
    uart_sendStr(buffer);
    sprintf(buffer, "Angle\tDistance\tIR Value\r\n");
    uart_sendStr(buffer);

    // Perform scan from MIN_ANGLE to MAX_ANGLE in STEP increments
    int i;
    for (i = 0; i < NUM_POINTS; i++) {
        int angle = (int)angles[i];

        // Move servo to position
        servo_move(angle);
        timer_waitMillis(50);  // Give servo time to reach position

        // Get sensor readings
        ping_trigger();
        ping_values[i] = ping_getDistance();
        ir_values[i] = adc_read();

        // Display data for ALL angles
        sprintf(buffer, "%d\t%.1f\t%d\r\n", angle, ping_values[i], ir_values[i]);
        uart_sendStr(buffer);
    }

    sprintf(buffer, "\r\n");
    uart_sendStr(buffer);
}

// Apply median filters to both ping and IR data to reduce noise
void filter_sensor_data(float *ping_values, int *ir_values, float *ping_filtered, int *ir_filtered)
{
    char buffer[100];
    sprintf(buffer, "Filtering sensor data...\r\n");
    uart_sendStr(buffer);

    // Copy boundaries directly (no filtering for first and last points)
    ping_filtered[0] = ping_values[0];
    ping_filtered[NUM_POINTS - 1] = ping_values[NUM_POINTS - 1];

    ir_filtered[0] = ir_values[0];
    ir_filtered[NUM_POINTS - 1] = ir_values[NUM_POINTS - 1];

    // Apply median filter of 3 to interior points
    int i;
    for (i = 1; i < NUM_POINTS - 1; i++) {
        // Ping filter (float values)
        ping_filtered[i] = median_of_3_float(
            ping_values[i - 1],
            ping_values[i],
            ping_values[i + 1]
        );

        // IR filter (integer values)
        ir_filtered[i] = median_of_3_int(
            ir_values[i - 1],
            ir_values[i],
            ir_values[i + 1]
        );
    }

    sprintf(buffer, "Filtering complete.\r\n\r\n");
    uart_sendStr(buffer);
}

// Median filter for float values (ping distances)
float median_of_3_float(float a, float b, float c)
{
    if (a > b) {
        if (b > c) return b;       // a > b > c
        else if (a > c) return c;  // a > c >= b
        else return a;             // c >= a > b
    }
    else {
        if (a > c) return a;       // b >= a > c
        else if (b > c) return c;  // b > c >= a
        else return b;             // c >= b >= a
    }
}

// Median filter for integer values (IR readings)
int median_of_3_int(int a, int b, int c)
{
    if (a > b) {
        if (b > c) return b;       // a > b > c
        else if (a > c) return c;  // a > c >= b
        else return a;             // c >= a > b
    }
    else {
        if (a > c) return a;       // b >= a > c
        else if (b > c) return c;  // b > c >= a
        else return b;             // c >= b >= a
    }
}

// Compute differences between consecutive IR readings for edge detection
void compute_ir_diff(int *ir_filtered, int *ir_diff)
{
    ir_diff[0] = 0;  // First element has no difference
    int i;
    for (i = 1; i < NUM_POINTS; i++) {
        ir_diff[i] = ir_filtered[i] - ir_filtered[i - 1];
    }
}

// Detect objects using edge detection on IR values
void detect_objects(float *angles, float *ping_filtered, int *ir_filtered, int *ir_diff)
{
    char buffer[100];
    sprintf(buffer, "Detecting objects...\r\n\r\n");
    uart_sendStr(buffer);

    objectCount = 0;

    // Parameters for edge detection
    int EDGE_THRESHOLD = IR_THRESHOLD / 4;  // Threshold for edge detection

    // Track object detection state
    bool inObject = false;
    int startIndex = 0;
    int i;

    // Detect edges based on ir_diff values
    for (i = 1; i < NUM_POINTS - 1; i++) {
        // Start of object: rising edge (positive diff exceeds threshold)
        if (!inObject && ir_diff[i] > EDGE_THRESHOLD) {
            startIndex = i;
            inObject = true;

            sprintf(buffer, "Leading edge detected at angle %.1f (IR diff: %d)\r\n",
                   angles[i], ir_diff[i]);
            uart_sendStr(buffer);
        }
        // End of object: falling edge (negative diff exceeds threshold)
        else if (inObject && ir_diff[i] < -EDGE_THRESHOLD) {
            int endIndex = i - 1;
            inObject = false;

            sprintf(buffer, "Trailing edge detected at angle %.1f (IR diff: %d)\r\n",
                   angles[i], ir_diff[i]);
            uart_sendStr(buffer);

            // Calculate object properties
            float startAngle = angles[startIndex];
            float endAngle = angles[endIndex];
            float radialWidth = endAngle - startAngle;
            float centerAngle = (startAngle + endAngle) / 2.0f;

            // Skip objects that are too narrow (likely noise)
            if (radialWidth < MIN_OBJECT_WIDTH) {
                sprintf(buffer, "Object too narrow (%.1f deg), skipping...\r\n", radialWidth);
                uart_sendStr(buffer);
                continue;
            }

            // Find minimum ping distance within object boundaries
            float minDist = 999.0f;
            int j;
            for (j = startIndex; j <= endIndex; j++) {
                if (ping_filtered[j] < minDist && ping_filtered[j] > 0) {
                    minDist = ping_filtered[j];
                }
            }

            // Calculate linear width using trigonometry
            float linearWidth = calculate_linear_width(radialWidth, minDist);

            // Store the object if we have space
            if (objectCount < MAX_OBJECTS) {
                objects[objectCount].startAngle = startAngle;
                objects[objectCount].endAngle = endAngle;
                objects[objectCount].centerAngle = centerAngle;
                objects[objectCount].radialWidth = radialWidth;
                objects[objectCount].distance = minDist;
                objects[objectCount].linearWidth = linearWidth;

                // Display object info
                sprintf(buffer, "Object %d:\r\n", objectCount + 1);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Start angle: %.1f\r\n", startAngle);
                uart_sendStr(buffer);

                sprintf(buffer, "  - End angle: %.1f\r\n", endAngle);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Center angle: %.1f\r\n", centerAngle);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Radial width: %.1f degrees\r\n", radialWidth);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Distance: %.1f cm\r\n", minDist);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Linear width: %.1f cm\r\n\r\n", linearWidth);
                uart_sendStr(buffer);

                objectCount++;
            }
        }
    }

    // If we ended while still in an object
    if (inObject) {
        int endIndex = NUM_POINTS - 1;
        float startAngle = angles[startIndex];
        float endAngle = angles[endIndex];
        float radialWidth = endAngle - startAngle;

        // Skip objects that are too narrow
        if (radialWidth >= MIN_OBJECT_WIDTH) {
            float centerAngle = (startAngle + endAngle) / 2.0f;

            // Find minimum ping distance within object
            float minDist = 999.0f;
            int j;
            for (j = startIndex; j <= endIndex; j++) {
                if (ping_filtered[j] < minDist && ping_filtered[j] > 0) {
                    minDist = ping_filtered[j];
                }
            }

            // Calculate linear width
            float linearWidth = calculate_linear_width(radialWidth, minDist);

            // Store the object if we have space
            if (objectCount < MAX_OBJECTS) {
                objects[objectCount].startAngle = startAngle;
                objects[objectCount].endAngle = endAngle;
                objects[objectCount].centerAngle = centerAngle;
                objects[objectCount].radialWidth = radialWidth;
                objects[objectCount].distance = minDist;
                objects[objectCount].linearWidth = linearWidth;

                // Display object info
                sprintf(buffer, "Object %d (end of scan):\r\n", objectCount + 1);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Start angle: %.1f\r\n", startAngle);
                uart_sendStr(buffer);

                sprintf(buffer, "  - End angle: %.1f\r\n", endAngle);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Center angle: %.1f\r\n", centerAngle);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Radial width: %.1f degrees\r\n", radialWidth);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Distance: %.1f cm\r\n", minDist);
                uart_sendStr(buffer);

                sprintf(buffer, "  - Linear width: %.1f cm\r\n\r\n", linearWidth);
                uart_sendStr(buffer);

                objectCount++;
            }
        }
    }

    // Summary of detected objects
    if (objectCount > 0) {
        sprintf(buffer, "Object Summary:\r\n");
        uart_sendStr(buffer);

        for (i = 0; i < objectCount; i++) {
            sprintf(buffer, "Obj %d | Center: %3.1f | Distance: %5.1f | Linear Width: %5.1f\r\n",
                   i+1, objects[i].centerAngle, objects[i].distance, objects[i].linearWidth);
            uart_sendStr(buffer);
        }
        sprintf(buffer, "\r\n");
        uart_sendStr(buffer);
    } else {
        sprintf(buffer, "No objects detected.\r\n\r\n");
        uart_sendStr(buffer);
    }
}

// Calculate linear width using trigonometry: 2 * distance * sin(angle/2)
float calculate_linear_width(float radialWidth, float distance)
{
    // Convert from degrees to radians
    float radialWidth_rad = radialWidth * (M_PI / 180.0f);

    // Calculate width using 2 * distance * sin(angle/2)
    return 2.0f * distance * sin(radialWidth_rad / 2.0f);
}
