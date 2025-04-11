/**
 * lab7.c - Drive to the smallest width object
 *
 * Uses IR sensor for edge detection and PING for distance measurement
 * Navigates to smallest width object while going around obstacles
 *
 * @author Your Name
 * @date March 25, 2025
 */

#include "Timer.h"
#include "lcd.h"
#include <string.h>
#include <math.h>
#include "uart.h"
#include "cyBot_Scan.h"
#include "open_interface.h"
#include "movement.h"

// Constants for scanning
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define STEP 2     // Scan every 1 degree for better precision
#define NUM_POINTS (((MAX_ANGLE - MIN_ANGLE) / STEP) + 1)

// Thresholds for edge detection
#define IR_THRESHOLD 150          // Threshold for IR value differences
#define IR_HIGH_VALUE 1000        // Threshold for high IR values
#define MIN_OBJECT_WIDTH 6.0f     // Minimum width in degrees to consider as an object
#define BOUNDARY_MARGIN 10        // Ignore objects with edges this close to scan limits

// CyBot calibration values
#define LEFT_CAL 1188250
#define RIGHT_CAL 238000

// Arrays for storing data
float angles[NUM_POINTS];
float ping_values[NUM_POINTS];
float ping_filtered[NUM_POINTS];
int ir_values[NUM_POINTS];
int ir_filtered[NUM_POINTS];
int ir_diff[NUM_POINTS];          // Differences between consecutive IR values

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

// Function declaration
void get_angle_array(void);
void scan_all_angles(void);
void filter_sensor_data(void);
float median_of_3_float(float a, float b, float c);
int median_of_3_int(int a, int b, int c);
void compute_ir_diff(void);
void detect_objects(void);
float calculate_linear_width(float radialWidth, float distance);
void send_uart_string(const char *str);
void clear_terminal(void);
void navigate_to_smallest_object(oi_t *sensor_data);

int main(void)
{
    // Initialize robot
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    // Initialize hardware
    timer_init();
    lcd_init();

    // Initialize UART
    uart_interrupt_init();

    // Initialize CyBot scan with proper calibration values
    cyBOT_init_Scan(0b0111);  // Enable servo, PING, and IR
    right_calibration_value = RIGHT_CAL;
    left_calibration_value = LEFT_CAL;

    // Position servo at 0 degrees to start
    cyBOT_Scan_t scan;
    cyBOT_Scan(0, &scan);
    timer_waitMillis(500);

    // Display welcome message
    clear_terminal();
    send_uart_string("=== Lab 7: Drive to Smallest Width Object ===\r\n\r\n");
    send_uart_string("Commands:\r\n");
    send_uart_string("  m - Start scan and navigation\r\n");
    send_uart_string("  q - Quit\r\n\r\n");

    while (1)
    {
        // Wait for user command
        char cmd = uart_receive();

        if (cmd == 'q') {
            // Quit program
            send_uart_string("\r\nExiting program.\r\n");
            break;
        }
        else if (cmd == 'm')
        {
            // Start mission
            clear_terminal();
            send_uart_string("Starting mission: Drive to smallest width object\r\n\r\n");

            // Navigate to the smallest object
            navigate_to_smallest_object(sensor_data);

            send_uart_string("\r\nPress 'm' to scan again or 'q' to quit.\r\n");
        }
    }

    // Clean up
    oi_free(sensor_data);
    return 0;
}

/**
 * Main navigation function that finds and goes to the smallest width object
 */
void navigate_to_smallest_object(oi_t *sensor_data)
{
    char buffer[100];

    // Initial scan
    send_uart_string("Performing initial scan...\r\n");
    objectCount = 0;

    // Get angles array
    get_angle_array();

    // Perform full scan
    scan_all_angles();

    // Filter data
    filter_sensor_data();

    // Compute differences
    compute_ir_diff();

    // Detect objects
    detect_objects();

    // If no objects found, move forward and try again
    if (objectCount == 0) {
        send_uart_string("No objects detected. Moving forward and rescanning...\r\n");

        // Move forward 10cm
        move_forward_smart(sensor_data, 100);

        // Rescan
        objectCount = 0;
        get_angle_array();
        scan_all_angles();
        filter_sensor_data();
        compute_ir_diff();
        detect_objects();
    }

    // If we found objects, navigate to the smallest one
    if (objectCount > 0) {
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
        send_uart_string(buffer);

        sprintf(buffer, "  - Center angle: %.1f degrees\r\n", targetAngle);
        send_uart_string(buffer);

        sprintf(buffer, "  - Distance: %.1f cm\r\n", targetDistance);
        send_uart_string(buffer);

        sprintf(buffer, "  - Linear width: %.1f cm\r\n\r\n", objects[smallestIndex].linearWidth);
        send_uart_string(buffer);

        send_uart_string("Navigating to smallest object...\r\n");

        // Go to the object, handling obstacles by going around them if needed
        int rescan_needed = go_to_position(sensor_data, targetAngle, targetDistance);

        // If a bump occurred during navigation, rescan to find objects
        if (rescan_needed) {
            send_uart_string("Rescanning after navigating around obstacle...\r\n");

            // Scan again
            objectCount = 0;
            get_angle_array();
            scan_all_angles();
            filter_sensor_data();
            compute_ir_diff();
            detect_objects();

            if (objectCount > 0) {
                // Find the smallest object again
                smallestIndex = 0;
                smallestWidth = objects[0].linearWidth;
                int i;
                for (i = 1; i < objectCount; i++) {
                    if (objects[i].linearWidth < smallestWidth) {
                        smallestWidth = objects[i].linearWidth;
                        smallestIndex = i;
                    }
                }

                // Display updated information
                sprintf(buffer, "Found target again (Object %d):\r\n", smallestIndex + 1);
                send_uart_string(buffer);

                sprintf(buffer, "  - Center angle: %.1f degrees\r\n", objects[smallestIndex].centerAngle);
                send_uart_string(buffer);

                sprintf(buffer, "  - Distance: %.1f cm\r\n", objects[smallestIndex].distance);
                send_uart_string(buffer);

                sprintf(buffer, "  - Linear width: %.1f cm\r\n\r\n", objects[smallestIndex].linearWidth);
                send_uart_string(buffer);

                // Go to the object without allowing another rescan
                go_to_position(sensor_data, objects[smallestIndex].centerAngle, objects[smallestIndex].distance);
            } else {
                send_uart_string("No objects detected. Moving forward and rescanning...\r\n");
                // Move forward 15cm
                move_forward_smart(sensor_data, 150);
            }
        }
        send_uart_string("Navigation complete!\r\n");

    }

}

// Populate the angles[] array from MIN_ANGLE to MAX_ANGLE in STEP increments
void get_angle_array(void)
{
    float angleVal = MIN_ANGLE;
    int i;
    for (i = 0; i < NUM_POINTS; i++) {
        angles[i] = angleVal;
        angleVal += STEP;
    }
}

// Perform a full scan and collect PING and IR data at each angle
void scan_all_angles(void)
{
    right_calibration_value = RIGHT_CAL;
    left_calibration_value = LEFT_CAL;
    cyBOT_Scan_t scan;
    char buffer[100];

    send_uart_string("Scanning...\r\n");
    send_uart_string("Angle   PING Distance (cm)   IR Value\r\n");
    send_uart_string("----------------------------------------\r\n");

    // Perform scan from MIN_ANGLE to MAX_ANGLE in STEP increments
    int i;
    for (i = 0; i < NUM_POINTS; i++) {
        int angle = (int)angles[i];

        // Get sensor readings at this angle
        cyBOT_Scan(angle, &scan);

        // Store the values
        ping_values[i] = scan.sound_dist;
        ir_values[i] = scan.IR_raw_val;

        // Display data for every angle
        sprintf(buffer, "%3d        %6.1f                %4d\r\n",
                angle, ping_values[i], ir_values[i]);
        send_uart_string(buffer);

        // Small delay to prevent UART buffer overrun
        timer_waitMillis(2);
    }

    send_uart_string("\r\n");
}

// Apply median filters to both PING and IR data to reduce noise
void filter_sensor_data(void)
{
    send_uart_string("Filtering sensor data...\r\n");

    // Copy boundaries directly (no filtering for first and last points)
    ping_filtered[0] = ping_values[0];
    ping_filtered[NUM_POINTS - 1] = ping_values[NUM_POINTS - 1];

    ir_filtered[0] = ir_values[0];
    ir_filtered[NUM_POINTS - 1] = ir_values[NUM_POINTS - 1];

    // Apply median filter of 3 to interior points
    int i;
    for (i = 1; i < NUM_POINTS - 1; i++) {
        // PING filter (float values)
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

    send_uart_string("Filtering complete.\r\n\r\n");
}

// Median filter for float values (PING distances)
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
void compute_ir_diff(void)
{
    ir_diff[0] = 0;  // First element has no difference
    int i;
    for (i = 1; i < NUM_POINTS; i++) {
        ir_diff[i] = ir_filtered[i] - ir_filtered[i - 1];
    }
}

// Detect objects using IR for edge detection and PING for distance
void detect_objects(void)
{
    send_uart_string("Detecting objects...\r\n\r\n");
    objectCount = 0;

    // IR detection thresholds - adjusted based on your scan data
    #define IR_OBJECT_THRESHOLD 950   // IR values above this are considered objects

    // Track object detection state
    int onObject = 0;
    int startIndex = 0;
    char buffer[100];
    int i;
    int j;

    // Find objects using absolute IR values
    for (i = 1; i < NUM_POINTS - 1; i++) {
        // Start of object: IR value exceeds threshold
        if (!onObject && ir_filtered[i] > IR_OBJECT_THRESHOLD) {
            startIndex = i;
            onObject = 1;

            sprintf(buffer, "Leading edge detected at angle %.1f (IR value: %d)\r\n",
                    angles[i], ir_filtered[i]);
            send_uart_string(buffer);
        }
        // End of object: IR value drops below threshold
        else if (onObject && ir_filtered[i] < IR_OBJECT_THRESHOLD) {
            int endIndex = i - 1;
            onObject = 0;

            sprintf(buffer, "Trailing edge detected at angle %.1f (IR value: %d)\r\n",
                    angles[endIndex], ir_filtered[endIndex]);
            send_uart_string(buffer);

            // Calculate object properties
            float startAngle = angles[startIndex];
            float endAngle = angles[endIndex];
            float radialWidth = endAngle - startAngle;
            float centerAngle = (startAngle + endAngle) / 2.0f;

            // Skip objects that are too narrow (likely noise)
            if (radialWidth < MIN_OBJECT_WIDTH) {
                send_uart_string("Object too narrow, skipping...\r\n");
                continue;
            }

            // Find minimum PING distance within object boundaries
            float minDist = 999.0f;
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
                send_uart_string(buffer);

                sprintf(buffer, "  - Start angle: %.1f\r\n", startAngle);
                send_uart_string(buffer);

                sprintf(buffer, "  - End angle: %.1f\r\n", endAngle);
                send_uart_string(buffer);

                sprintf(buffer, "  - Center angle: %.1f\r\n", centerAngle);
                send_uart_string(buffer);

                sprintf(buffer, "  - Radial width: %.1f degrees\r\n", radialWidth);
                send_uart_string(buffer);

                sprintf(buffer, "  - Distance: %.1f cm\r\n", minDist);
                send_uart_string(buffer);

                sprintf(buffer, "  - Linear width: %.1f cm\r\n\r\n", linearWidth);
                send_uart_string(buffer);

                objectCount++;
            }
        }
    }

    // If still on an object at end of scan, properly end it
    if (onObject) {
        int endIndex = NUM_POINTS - 1;
        float startAngle = angles[startIndex];
        float endAngle = angles[endIndex];
        float radialWidth = endAngle - startAngle;

        // Skip objects that are too narrow
        if (radialWidth >= MIN_OBJECT_WIDTH) {
            float centerAngle = (startAngle + endAngle) / 2.0f;

            // Find minimum PING distance within object
            float minDist = 999.0f;
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
                send_uart_string(buffer);

                sprintf(buffer, "  - Start angle: %.1f\r\n", startAngle);
                send_uart_string(buffer);

                sprintf(buffer, "  - End angle: %.1f\r\n", endAngle);
                send_uart_string(buffer);

                sprintf(buffer, "  - Center angle: %.1f\r\n", centerAngle);
                send_uart_string(buffer);

                sprintf(buffer, "  - Radial width: %.1f degrees\r\n", radialWidth);
                send_uart_string(buffer);

                sprintf(buffer, "  - Distance: %.1f cm\r\n", minDist);
                send_uart_string(buffer);

                sprintf(buffer, "  - Linear width: %.1f cm\r\n\r\n", linearWidth);
                send_uart_string(buffer);

                objectCount++;
            }
        }
    }

    // Summary of detected objects
    if (objectCount > 0) {
        send_uart_string("Object Summary:\r\n");

        for (i = 0; i < objectCount; i++) {
            sprintf(buffer, "Obj %d | Center: %3.1f | Distance: %5.1f | Linear Width: %5.1f\r\n",
                    i+1, objects[i].centerAngle, objects[i].distance, objects[i].linearWidth);
            send_uart_string(buffer);
        }
        send_uart_string("\r\n");
    } else {
        send_uart_string("No objects detected.\r\n\r\n");
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

// Send a string to UART
void send_uart_string(const char *str)
{
    uart_sendStr(str);
}

// Clear terminal using ANSI escape sequences
void clear_terminal(void)
{
    // ANSI escape sequence to clear screen and move cursor home
    char clearSeq[] = "\033[2J\033[H";
    uart_sendStr(clearSeq);
}
