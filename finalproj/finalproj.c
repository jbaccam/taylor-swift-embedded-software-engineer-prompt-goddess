//
// finalproj.c - mars rover "ping patrol" - 180 scan
//
// fully c90 compatible, clear and simple
//

#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "open_interface.h"
#include "servo.h"
#include "adc.h"
#include "movement.h"
#include "ping.h" // adding ping header for ping scan functionality

#include <math.h>
#include <stdio.h>

// constants
#define STEP_DEGREE 1
#define IR_THRESHOLD 950
#define MIN_OBJECT_WIDTH_DEGREE 6
#define FRONT_SCAN_POINTS ((180 / STEP_DEGREE) + 1)
#define BLUE_THRESHOLD 2800  // for blue paper water samples returns 2800-2900
#define WHITE_THRESHOLD 2600  // for white tape boundaries returns 2750-2800
#define FLOOR_THRESHOLD 2200  // regular floor baseline values
#define BLACK_THRESHOLD 200   // black boxes or craters values below this are craters
#define MAX_OBJECTS 10        // Maximum number of objects to track

// Object information structure
typedef struct {
    float startAngle;
    float endAngle;
    float centerAngle;
    float radialWidth;
    float distance;
    float linearWidth;
} Object;

// globals
static int ir_readings[FRONT_SCAN_POINTS];
static int ir_filtered[FRONT_SCAN_POINTS];  // Added for filtered IR values
static int ir_diff[FRONT_SCAN_POINTS];      // Added for IR edge detection
static float distances_cm[FRONT_SCAN_POINTS];
static float ping_distances_cm[FRONT_SCAN_POINTS]; // added for ping scan results
static Object objects[MAX_OBJECTS];  // Added to store detected objects
static int object_count = 0;         // Added to track number of objects detected
char buffer[100];                    // Global buffer for string formatting

// function prototypes
float calculate_distance(int ir_value);
void ir_scan_front(oi_t *sensor_data); // renamed to specify ir scan
void ping_scan_front(oi_t *sensor_data); // added ping scan function
void detect_objects();
void detect_objects_ping(); // added for ping scan detection
void safety_check(oi_t *sensor_data);
int get_numeric_input();
void blue_sample_check(oi_t *sensor_data);
void filter_sensor_data();  // Added for filtering IR values
float median_of_3_float(float a, float b, float c);  // Added for filtering
int median_of_3_int(int a, int b, int c);            // Added for filtering
void compute_ir_diff();     // Added for edge detection
float calculate_linear_width(float radialWidth, float distance); // Added for object width

// main program
int main(void)
{
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    servo_init();
    ping_init(); // initialize ping sensor

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    servo_move(90); // center servo at start

    uart_sendStr("\r\n*** Mars Rover 'Ping Patrol' Control Interface ***\r\n");
    uart_sendStr("Commands: (i)r scan, (p)ing scan, (m)ove, (b)lue sample check, (h)elp\r\n");

    while (1)
    {
        // periodically check sensors for safety
        oi_update(sensor_data);
        safety_check(sensor_data);

        uart_sendStr("\r\n> ");
        char choice = uart_receive();
        uart_sendChar(choice);
        uart_sendStr("\r\n");

        if (choice == 'i')
        {
            oi_setWheels(0, 0); // stop before scanning
            ir_scan_front(sensor_data);
            // Filter the IR data before object detection
            filter_sensor_data();
            compute_ir_diff();
            detect_objects();
        }
        else if (choice == 'p')
        {
            oi_setWheels(0, 0); // stop before scanning
            ping_scan_front(sensor_data);
            detect_objects_ping();
        }
        else if (choice == 'm')
        {
            int turn_angle;
            int distance_mm;

            uart_sendStr("\r\nEnter turn angle (+ right, - left): ");
            turn_angle = get_numeric_input();

            sprintf(buffer, "\r\nTurn angle set to: %d degrees\r\n", turn_angle);
            uart_sendStr(buffer);

            uart_sendStr("\r\nEnter distance (cm): ");
            distance_mm = get_numeric_input() * 10; // convert cm to mm

            sprintf(buffer, "\r\nDistance set to: %d mm\r\n", distance_mm);
            uart_sendStr(buffer);

            // execute turn based on direction
            if (turn_angle > 0) {
                sprintf(buffer, "\r\nTurning right %d degrees...\r\n", turn_angle);
                uart_sendStr(buffer);
                turn_right(sensor_data, turn_angle);
            } else if (turn_angle < 0) {
                sprintf(buffer, "\r\nTurning left %d degrees...\r\n", -turn_angle);
                uart_sendStr(buffer);
                turn_left(sensor_data, -turn_angle);
            }

            // only move if distance is greater than zero
            if (distance_mm > 0) {
                sprintf(buffer, "\r\nMoving forward %d mm...\r\n", distance_mm);
                uart_sendStr(buffer);
                move_forward(sensor_data, distance_mm);
            }

            uart_sendStr("\r\nMovement complete.\r\n");
        }
        else if (choice == 'b')
        {
            uart_sendStr("\r\nChecking for blue sample...\r\n");
            blue_sample_check(sensor_data);
        }
        else if (choice == 'r')
        {
            // quick right turn 45 degrees
            uart_sendStr("\r\nQuick turn right 45 degrees\r\n");
            turn_right(sensor_data, 10);
        }
        else if (choice == 'l')
        {
            // quick left turn 45 degrees
            uart_sendStr("\r\nQuick turn left 45 degrees\r\n");
            turn_left(sensor_data, 10);
        }
        else if (choice == 'f')
        {
            // quick forward movement 10cm
            uart_sendStr("\r\nQuick move forward 10cm\r\n");
            move_forward(sensor_data, 100);
        }
        else if (choice == 'v')
        {
            // quick backward movement 10cm
            uart_sendStr("\r\nQuick move backward 10cm\r\n");
            move_backward(sensor_data, 100);
        }
        else if (choice == 's')
        {
            // sample collection simulation
            uart_sendStr("\r\nSimulating sample collection (spinning 360)...\r\n");
            turn_left(sensor_data, 360);
            uart_sendStr("Sample collection complete.\r\n");
        }
        else if (choice == 'h')
        {
            // help menu
            uart_sendStr("\r\n--- Mars Rover Command Help ---\r\n");
            uart_sendStr("i - IR scan surroundings\r\n");
            uart_sendStr("p - PING scan surroundings\r\n");
            uart_sendStr("m - Move with turn angle and distance\r\n");
            uart_sendStr("b - Check for blue sample\r\n");
            uart_sendStr("r - Quick turn right 45 degrees\r\n");
            uart_sendStr("l - Quick turn left 45 degrees\r\n");
            uart_sendStr("f - Quick move forward 10cm\r\n");
            uart_sendStr("v - Quick move backward 10cm\r\n");
            uart_sendStr("s - Simulate sample collection (spin 360)\r\n");
            uart_sendStr("h - Show this help menu\r\n");
        }
    }
}

// get user input as a number
int get_numeric_input()
{
    char input[16];  // buffer for storing input
    int index = 0;   // current position in buffer
    char c;          // current character
    int value = 0;   // final value to return
    int negative = 0; // flag for negative numbers
    int i;           // loop counter

    // clear the input buffer
    for (i = 0; i < 16; i++) {
        input[i] = 0;
    }

    // wait a bit before starting to read helps with putty
    timer_waitMillis(50);

    // clear any pending input
    while ((UART1_FR_R & 0x10) == 0) {
        // read and discard any character in buffer
        UART1_DR_R;
    }

    while (1) {
        // wait for a character
        c = uart_receive();

        // give a small delay after each keypress to avoid duplicates
        timer_waitMillis(50);

        // check for enter key
        if (c == '\r' || c == '\n') {
            uart_sendStr("\r\n");
            break;
        }

        // handle backspace
        if (c == 8 || c == 127) {
            if (index > 0) {
                index--;
                uart_sendStr("\b \b"); // erase character on terminal
            }
            continue;
        }

        // handle minus sign only at beginning
        if (c == '-' && index == 0) {
            input[index++] = c;
            uart_sendChar(c);
            negative = 1;
            continue;
        }

        // only accept digits with a reasonable max length
        if (c >= '0' && c <= '9' && index < 15) {
            input[index++] = c;
            uart_sendChar(c); // echo character once
        }
    }

    // end the string
    input[index] = '\0';

    // print what we are parsing
    uart_sendStr("Input: ");
    uart_sendStr(input);
    uart_sendStr("\r\n");

    // convert string to integer
    i = negative ? 1 : 0; // start after minus sign if negative

    // manual conversion digit by digit
    while (i < index) {
        if (input[i] >= '0' && input[i] <= '9') {
            value = value * 10 + (input[i] - '0');
        }
        i++;
    }

    // apply negative sign if needed
    if (negative) {
        value = -value;
    }

    // prevent extreme values
    if (value > 1000) {
        uart_sendStr("Value too large, limiting to 1000\r\n");
        value = 1000;
    } else if (value < -1000) {
        uart_sendStr("Value too small, limiting to -1000\r\n");
        value = -1000;
    }

    return value;
}

// scan the environment using ir sensor
void ir_scan_front(oi_t *sensor_data)
{
    int i;
    int angle = 0;
    int raw = 0;
    float distance = 0.0f;

    uart_sendStr("\r\nBeginning IR environment scan...\r\n");
    uart_sendStr("Angle\tDistance(cm)\tIR Raw\r\n-----------------------------------\r\n");

    for (i = 0; i < FRONT_SCAN_POINTS; i++)
    {
        angle = i * STEP_DEGREE;
        servo_move(angle);
        // with hardware averaging set by adc0_sac_r one read is enough
        raw = adc_read();
        distance = calculate_distance(raw);

        ir_readings[i]   = raw;
        distances_cm[i]  = distance;

        sprintf(buffer, "%3d\t%7.2f\t%d\r\n", angle, distance, raw);
        uart_sendStr(buffer);
    }
    // return servo to center position
    servo_move(90);
    uart_sendStr("\r\nIR scan complete.\r\n");
}

// scan the environment using ping sensor
void ping_scan_front(oi_t *sensor_data)
{
    int i;
    int angle = 0;
    float ping_distance = 0.0f;

    uart_sendStr("\r\nBeginning PING environment scan...\r\n");
    uart_sendStr("Angle\tDistance(cm)\r\n-----------------------------------\r\n");

    for (i = 0; i < FRONT_SCAN_POINTS; i++)
    {
        angle = i * STEP_DEGREE;
        servo_move(angle);

        // use ping for distance measurement
        ping_trigger();
        ping_distance = ping_getDistance();

        // store the ping distance
        ping_distances_cm[i] = ping_distance;

        sprintf(buffer, "%3d\t%7.2f\r\n", angle, ping_distance);
        uart_sendStr(buffer);
    }

    // return servo to center position
    servo_move(90);
    uart_sendStr("\r\nPING scan complete.\r\n");
}

// Apply median filters to IR data to reduce noise
void filter_sensor_data()
{
    uart_sendStr("\r\nFiltering sensor data...");

    // Copy boundaries directly (no filtering for first and last points)
    ir_filtered[0] = ir_readings[0];
    ir_filtered[FRONT_SCAN_POINTS - 1] = ir_readings[FRONT_SCAN_POINTS - 1];

    // Apply median filter of 3 to interior points
    int i;
    for (i = 1; i < FRONT_SCAN_POINTS - 1; i++) {
        // IR filter (integer values)
        ir_filtered[i] = median_of_3_int(
            ir_readings[i - 1],
            ir_readings[i],
            ir_readings[i + 1]
        );
    }

    uart_sendStr(" done.\r\n");
}

// Compute differences between consecutive IR readings for edge detection
void compute_ir_diff()
{
    ir_diff[0] = 0;  // First element has no difference
    int i;
    for (i = 1; i < FRONT_SCAN_POINTS; i++) {
        ir_diff[i] = ir_filtered[i] - ir_filtered[i - 1];
    }
}

// Median filter for float values
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

// Median filter for integer values
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

// Calculate linear width using trigonometry: 2 * distance * sin(angle/2)
float calculate_linear_width(float radialWidth, float distance)
{
    // Convert from degrees to radians
    float radialWidth_rad = radialWidth * (M_PI / 180.0f);

    // Calculate width using 2 * distance * sin(angle/2)
    return 2.0f * distance * sin(radialWidth_rad / 2.0f);
}

// detect objects from ir scan data
void detect_objects()
{
    uart_sendStr("\r\nProcessing IR scan data...\r\n");

    // Reset object count
    object_count = 0;

    // Track object detection state
    int on_object = 0;
    int start_index = 0;
    int i, j;

    // Find objects using absolute IR values (without verbose output)
    for (i = 1; i < FRONT_SCAN_POINTS - 1; i++) {
        // Start of object: IR value exceeds threshold
        if (!on_object && ir_filtered[i] > IR_THRESHOLD) {
            start_index = i;
            on_object = 1;
        }
        // End of object: IR value drops below threshold
        else if (on_object && ir_filtered[i] < IR_THRESHOLD) {
            int end_index = i - 1;
            on_object = 0;

            // Calculate object properties
            float start_angle = start_index * STEP_DEGREE;
            float end_angle = end_index * STEP_DEGREE;
            float radial_width = end_angle - start_angle;
            float center_angle = (start_angle + end_angle) / 2.0f;

            // Skip objects that are too narrow (likely noise)
            if (radial_width < MIN_OBJECT_WIDTH_DEGREE) {
                continue;
            }

            // Find minimum distance within object boundaries
            float min_dist = 999.0f;
            for (j = start_index; j <= end_index; j++) {
                if (distances_cm[j] < min_dist && distances_cm[j] > 0) {
                    min_dist = distances_cm[j];
                }
            }

            // Calculate linear width using trigonometry
            float linear_width = calculate_linear_width(radial_width, min_dist);

            // Store the object if we have space
            if (object_count < MAX_OBJECTS) {
                objects[object_count].startAngle = start_angle;
                objects[object_count].endAngle = end_angle;
                objects[object_count].centerAngle = center_angle;
                objects[object_count].radialWidth = radial_width;
                objects[object_count].distance = min_dist;
                objects[object_count].linearWidth = linear_width;
                object_count++;
            }
        }
    }

    // If still on an object at end of scan, properly end it
    if (on_object) {
        int end_index = FRONT_SCAN_POINTS - 1;
        float start_angle = start_index * STEP_DEGREE;
        float end_angle = end_index * STEP_DEGREE;
        float radial_width = end_angle - start_angle;

        // Skip objects that are too narrow
        if (radial_width >= MIN_OBJECT_WIDTH_DEGREE) {
            float center_angle = (start_angle + end_angle) / 2.0f;

            // Find minimum distance within object
            float min_dist = 999.0f;
            for (j = start_index; j <= end_index; j++) {
                if (distances_cm[j] < min_dist && distances_cm[j] > 0) {
                    min_dist = distances_cm[j];
                }
            }

            // Calculate linear width
            float linear_width = calculate_linear_width(radial_width, min_dist);

            // Store the object if we have space
            if (object_count < MAX_OBJECTS) {
                objects[object_count].startAngle = start_angle;
                objects[object_count].endAngle = end_angle;
                objects[object_count].centerAngle = center_angle;
                objects[object_count].radialWidth = radial_width;
                objects[object_count].distance = min_dist;
                objects[object_count].linearWidth = linear_width;
                object_count++;
            }
        }
    }

    // Only display the final results
    uart_sendStr("\r\nIR Object Detection Results:\r\n");
    uart_sendStr("Obj | Center | Distance | Width\r\n-----------------------------------\r\n");

    for (i = 0; i < object_count; i++) {
        sprintf(buffer, "%3d | %6.1f | %7.2f | %7.2f\r\n",
                i + 1, objects[i].centerAngle, objects[i].distance, objects[i].linearWidth);
        uart_sendStr(buffer);
    }

    if (object_count == 0) {
        uart_sendStr("No objects detected with IR.\r\n");
    } else {
        sprintf(buffer, "\r\nTotal objects detected with IR: %d\r\n", object_count);
        uart_sendStr(buffer);
    }
}

// Detect objects using ping sensor data
void detect_objects_ping()
{
    uart_sendStr("\r\nProcessing PING scan data...\r\n");

    // Reset object count
    object_count = 0;

    // Track object detection state
    int on_object = 0;
    int start_index = 0;
    int i, j;
    float threshold_distance = 100.0f; // Adjust based on your environment

    // Find objects by looking for regions with distance < threshold (without verbose output)
    for (i = 1; i < FRONT_SCAN_POINTS - 1; i++) {
        // Start of object: Distance is below threshold
        if (!on_object && ping_distances_cm[i] < threshold_distance && ping_distances_cm[i] > 0) {
            start_index = i;
            on_object = 1;
        }
        // End of object: Distance goes above threshold
        else if (on_object && (ping_distances_cm[i] >= threshold_distance || ping_distances_cm[i] <= 0)) {
            int end_index = i - 1;
            on_object = 0;

            // Calculate object properties
            float start_angle = start_index * STEP_DEGREE;
            float end_angle = end_index * STEP_DEGREE;
            float radial_width = end_angle - start_angle;
            float center_angle = (start_angle + end_angle) / 2.0f;

            // Skip objects that are too narrow (likely noise)
            if (radial_width < MIN_OBJECT_WIDTH_DEGREE) {
                continue;
            }

            // Find minimum distance within object boundaries
            float min_dist = 999.0f;
            for (j = start_index; j <= end_index; j++) {
                if (ping_distances_cm[j] < min_dist && ping_distances_cm[j] > 0) {
                    min_dist = ping_distances_cm[j];
                }
            }

            // Calculate linear width using trigonometry
            float linear_width = calculate_linear_width(radial_width, min_dist);

            // Store the object if we have space
            if (object_count < MAX_OBJECTS) {
                objects[object_count].startAngle = start_angle;
                objects[object_count].endAngle = end_angle;
                objects[object_count].centerAngle = center_angle;
                objects[object_count].radialWidth = radial_width;
                objects[object_count].distance = min_dist;
                objects[object_count].linearWidth = linear_width;
                object_count++;
            }
        }
    }

    // If still on an object at end of scan, properly end it
    if (on_object) {
        int end_index = FRONT_SCAN_POINTS - 1;
        float start_angle = start_index * STEP_DEGREE;
        float end_angle = end_index * STEP_DEGREE;
        float radial_width = end_angle - start_angle;

        // Skip objects that are too narrow
        if (radial_width >= MIN_OBJECT_WIDTH_DEGREE) {
            float center_angle = (start_angle + end_angle) / 2.0f;

            // Find minimum distance within object
            float min_dist = 999.0f;
            for (j = start_index; j <= end_index; j++) {
                if (ping_distances_cm[j] < min_dist && ping_distances_cm[j] > 0) {
                    min_dist = ping_distances_cm[j];
                }
            }

            // Calculate linear width
            float linear_width = calculate_linear_width(radial_width, min_dist);

            // Store the object if we have space
            if (object_count < MAX_OBJECTS) {
                objects[object_count].startAngle = start_angle;
                objects[object_count].endAngle = end_angle;
                objects[object_count].centerAngle = center_angle;
                objects[object_count].radialWidth = radial_width;
                objects[object_count].distance = min_dist;
                objects[object_count].linearWidth = linear_width;
                object_count++;
            }
        }
    }

    // Only display the final results
    uart_sendStr("\r\nPING Object Detection Results:\r\n");
    uart_sendStr("Obj | Center | Distance | Width\r\n-----------------------------------\r\n");

    for (i = 0; i < object_count; i++) {
        sprintf(buffer, "%3d | %6.1f | %7.2f | %7.2f\r\n",
                i + 1, objects[i].centerAngle, objects[i].distance, objects[i].linearWidth);
        uart_sendStr(buffer);
    }

    if (object_count == 0) {
        uart_sendStr("No objects detected with PING.\r\n");
    } else {
        sprintf(buffer, "\r\nTotal objects detected with PING: %d\r\n", object_count);
        uart_sendStr(buffer);
    }
}

// check safety sensors
void safety_check(oi_t *sensor_data)
{
    // check for bumpers
    if (sensor_data->bumpLeft || sensor_data->bumpRight)
    {
        uart_sendStr("\r\n*** warning: obstacle collision detected! ***\r\n");
        move_backward(sensor_data, 50); // backup 5 cm
    }

    // check for white tape boundaries
    if ((sensor_data->cliffLeftSignal > WHITE_THRESHOLD &&
         sensor_data->cliffLeftSignal < BLUE_THRESHOLD) ||
        (sensor_data->cliffRightSignal > WHITE_THRESHOLD &&
         sensor_data->cliffRightSignal < BLUE_THRESHOLD) ||
        (sensor_data->cliffFrontLeftSignal > WHITE_THRESHOLD &&
         sensor_data->cliffFrontLeftSignal < BLUE_THRESHOLD) ||
        (sensor_data->cliffFrontRightSignal > WHITE_THRESHOLD &&
         sensor_data->cliffFrontRightSignal < BLUE_THRESHOLD))
    {
        // get which sensors detected the boundary
        sprintf(buffer, "\r\n*** white tape boundary detected ***\r\n");
        uart_sendStr(buffer);

        // just back away from boundary no turning
        uart_sendStr("backing away from boundary...\r\n");
        move_backward(sensor_data, 50); // backup 5 cm
    }

    // check for black craters or holes
    if (sensor_data->cliffLeftSignal < BLACK_THRESHOLD ||
        sensor_data->cliffRightSignal < BLACK_THRESHOLD ||
        sensor_data->cliffFrontLeftSignal < BLACK_THRESHOLD ||
        sensor_data->cliffFrontRightSignal < BLACK_THRESHOLD)
    {
        // get which sensors detected the crater
        sprintf(buffer, "\r\n*** black crater detected ***\r\n");
        uart_sendStr(buffer);

        // just back away from crater no turning
        uart_sendStr("backing away from crater...\r\n");
        move_backward(sensor_data, 50); // backup 5 cm
    }

    // check for blue samples values above the blue threshold
    if (sensor_data->cliffLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffRightSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontRightSignal >= BLUE_THRESHOLD)
    {
        // display which sensors detected blue sample
        sprintf(buffer, "\r\n*** blue sample detected ***\r\n");
        uart_sendStr(buffer);

        // stop the robot
        oi_setWheels(0, 0);

        // collect sample data by spinning
        uart_sendStr("collecting sample data spinning 360 degrees...\r\n");
        turn_left(sensor_data, 360);

        uart_sendStr("sample collection complete.\r\n");
    }
}

// updated blue sample check function
void blue_sample_check(oi_t *sensor_data)
{
    oi_update(sensor_data);

    // display all cliff sensor values for debugging
    sprintf(buffer, "cliff values: l=%d, fl=%d, fr=%d, r=%d\r\n",
            sensor_data->cliffLeftSignal,
            sensor_data->cliffFrontLeftSignal,
            sensor_data->cliffFrontRightSignal,
            sensor_data->cliffRightSignal);
    uart_sendStr(buffer);

    // check if any cliff sensor detects blue paper
    if (sensor_data->cliffLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffRightSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontRightSignal >= BLUE_THRESHOLD)
    {
        uart_sendStr("*** blue sample detected! ***\r\n");
        uart_sendStr("collecting sample data spinning...\r\n");

        // stop first
        oi_setWheels(0, 0);
        timer_waitMillis(50);

        // spin in place for sample collection simulation
        turn_left(sensor_data, 360);

        uart_sendStr("sample collection complete.\r\n");
    }
    else
    {
        uart_sendStr("no blue sample detected.\r\n");
    }
}

// calculate distance from ir sensor value
float calculate_distance(int irValue) {
    if (irValue <= 0) {  // prevent division by zero or negative values
        return 0;
    }
    // directly compute the distance from the adc reading y using the derived equation
    float distance = pow(12453.9382f / (float)irValue, 1.0f / 0.7358f);
    return distance;
}
