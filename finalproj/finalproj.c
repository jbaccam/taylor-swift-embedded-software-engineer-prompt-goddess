/*
 * finalproj.c - Mars Rover "Ping Patrol" - 180 Scan
 *
 * Fully C90 compatible, clear and simple
 */

#include "Timer.h"
#include "lcd.h"
#include "uart.h"
#include "open_interface.h"
#include "servo.h"
#include "adc.h"
#include "movement.h"

#include <math.h>
#include <stdio.h>

/* --- Constants --- */
#define STEP_DEGREE 2
#define IR_THRESHOLD 950
#define MIN_OBJECT_WIDTH_DEGREE 6
#define FRONT_SCAN_POINTS ((180 / STEP_DEGREE) + 1)
#define BLUE_THRESHOLD 2850  /* For blue paper (water samples) - returns 2850-2900 */
#define WHITE_THRESHOLD 2750  /* For white tape (boundaries) - returns 2750-2800 */
#define FLOOR_THRESHOLD 2200  /* Regular floor - baseline values */
#define BLACK_THRESHOLD 200   /* Black boxes/craters - values below this are craters */

/* --- Globals --- */
static int ir_readings[FRONT_SCAN_POINTS];
static float distances_cm[FRONT_SCAN_POINTS];

/* --- Function Prototypes --- */
float calculate_distance(int ir_value);
void scan_front(oi_t *sensor_data);
void detect_objects();
void safety_check(oi_t *sensor_data);
int get_numeric_input();
void blue_sample_check(oi_t *sensor_data);

/* --- Main Program --- */
int main(void)
{
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    servo_init();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    servo_move(90); /* Center servo at start */

    char buffer[64];

    uart_sendStr("\r\n*** Mars Rover 'Ping Patrol' Control Interface ***\r\n");
    uart_sendStr("Commands: (s)can, (m)ove, (b)lue sample check, (h)elp\r\n");

    while (1)
    {
        /* Periodically check sensors for safety */
        oi_update(sensor_data);
        safety_check(sensor_data);

        uart_sendStr("\r\n> ");
        char choice = uart_receive();
        uart_sendChar(choice);
        uart_sendStr("\r\n");

        if (choice == 's')
        {
            oi_setWheels(0, 0); /* Stop before scanning */
            scan_front(sensor_data);
            detect_objects();
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
            distance_mm = get_numeric_input() * 10; /* Convert cm to mm */

            sprintf(buffer, "\r\nDistance set to: %d mm\r\n", distance_mm);
            uart_sendStr(buffer);

            /* Execute turn based on direction */
            if (turn_angle > 0) {
                sprintf(buffer, "\r\nTurning right %d degrees...\r\n", turn_angle);
                uart_sendStr(buffer);
                turn_right(sensor_data, turn_angle);
            } else if (turn_angle < 0) {
                sprintf(buffer, "\r\nTurning left %d degrees...\r\n", -turn_angle);
                uart_sendStr(buffer);
                turn_left(sensor_data, -turn_angle);
            }

            /* Only move if distance is greater than zero */
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
            /* Quick right turn (45 degrees) */
            uart_sendStr("\r\nQuick turn right 45 degrees\r\n");
            turn_right(sensor_data, 45);
        }
        else if (choice == 'l')
        {
            /* Quick left turn (45 degrees) */
            uart_sendStr("\r\nQuick turn left 45 degrees\r\n");
            turn_left(sensor_data, 45);
        }
        else if (choice == 'f')
        {
            /* Quick forward movement (10cm) */
            uart_sendStr("\r\nQuick move forward 10cm\r\n");
            move_forward(sensor_data, 100);
        }
        else if (choice == 'v')
        {
            /* Quick backward movement (10cm) */
            uart_sendStr("\r\nQuick move backward 10cm\r\n");
            move_backward(sensor_data, 100);
        }
        else if (choice == 'p')
        {
            /* Sample collection simulation */
            uart_sendStr("\r\nSimulating sample collection (spinning 360)...\r\n");
            turn_left(sensor_data, 360);
            uart_sendStr("Sample collection complete.\r\n");
        }
        else if (choice == 'h')
        {
            /* Help menu */
            uart_sendStr("\r\n--- Mars Rover Command Help ---\r\n");
            uart_sendStr("s - Scan surroundings\r\n");
            uart_sendStr("m - Move with turn angle and distance\r\n");
            uart_sendStr("b - Check for blue sample\r\n");
            uart_sendStr("r - Quick turn right 45 degrees\r\n");
            uart_sendStr("l - Quick turn left 45 degrees\r\n");
            uart_sendStr("f - Quick move forward 10cm\r\n");
            uart_sendStr("v - Quick move backward 10cm\r\n");
            uart_sendStr("p - Simulate sample collection (spin 360)\r\n");
            uart_sendStr("h - Show this help menu\r\n");
        }
    }
}

/* --- Get user input as a number --- */
int get_numeric_input()
{
    char input[16];  /* Buffer for storing input */
    int index = 0;   /* Current position in buffer */
    char c;          /* Current character */
    int value = 0;   /* Final value to return */
    int negative = 0; /* Flag for negative numbers */
    int i;           /* Loop counter */

    /* Clear the input buffer */
    for (i = 0; i < 16; i++) {
        input[i] = 0;
    }

    /* Wait a bit before starting to read (helps with PuTTY) */
    timer_waitMillis(50);

    /* Clear any pending input */
    while ((UART1_FR_R & 0x10) == 0) {
        /* Read and discard any character in buffer */
        UART1_DR_R;
    }

    while (1) {
        /* Wait for a character */
        c = uart_receive();

        /* Give a small delay after each keypress to avoid duplicates */
        timer_waitMillis(50);

        /* Check for enter key */
        if (c == '\r' || c == '\n') {
            uart_sendStr("\r\n");
            break;
        }

        /* Handle backspace */
        if (c == 8 || c == 127) {
            if (index > 0) {
                index--;
                uart_sendStr("\b \b"); /* Erase character on terminal */
            }
            continue;
        }

        /* Handle minus sign (only at beginning) */
        if (c == '-' && index == 0) {
            input[index++] = c;
            uart_sendChar(c);
            negative = 1;
            continue;
        }

        /* Only accept digits, with a reasonable max length */
        if (c >= '0' && c <= '9' && index < 15) {
            input[index++] = c;
            uart_sendChar(c); /* Echo character once */
        }
    }

    /* End the string */
    input[index] = '\0';

    /* Print what we're parsing */
    uart_sendStr("Input: ");
    uart_sendStr(input);
    uart_sendStr("\r\n");

    /* Convert string to integer */
    i = negative ? 1 : 0; /* Start after minus sign if negative */

    /* Manual conversion, digit by digit */
    while (i < index) {
        if (input[i] >= '0' && input[i] <= '9') {
            value = value * 10 + (input[i] - '0');
        }
        i++;
    }

    /* Apply negative sign if needed */
    if (negative) {
        value = -value;
    }

    /* Prevent extreme values */
    if (value > 1000) {
        uart_sendStr("Value too large, limiting to 1000\r\n");
        value = 1000;
    } else if (value < -1000) {
        uart_sendStr("Value too small, limiting to -1000\r\n");
        value = -1000;
    }

    return value;
}

/* --- Scan the environment --- */
void scan_front(oi_t *sensor_data)
{
    char buffer[64];
    int i;
    int angle = 0;
    int raw = 0;
    float distance = 0.0f;

    uart_sendStr("\r\nBeginning environment scan...\r\n");
    uart_sendStr("Angle\tDistance(cm)\tIR Raw\r\n-----------------------------------\r\n");

    for (i = 0; i < FRONT_SCAN_POINTS; i++)
    {
        angle = i * STEP_DEGREE;
        servo_move(angle);
        timer_waitMillis(100); /* Give servo time to position */

        raw = adc_read();
        distance = calculate_distance(raw);

        ir_readings[i] = raw;
        distances_cm[i] = distance;

        sprintf(buffer, "%3d\t%7.2f\t%d\r\n", angle, distance, raw);
        uart_sendStr(buffer);
    }

    /* Return servo to center position */
    servo_move(90);
    uart_sendStr("\r\nScan complete.\r\n");
}

/* --- Detect objects from scan data --- */
void detect_objects()
{
    char buffer[64];
    int i;
    int in_object = 0;
    int start_index = 0;
    int object_count = 0;

    uart_sendStr("\r\nObject Detection Results:\r\n");
    uart_sendStr("Obj | Center | Distance | Width\r\n-----------------------------------\r\n");

    for (i = 0; i < FRONT_SCAN_POINTS; i++)
    {
        if (!in_object && ir_readings[i] > IR_THRESHOLD)
        {
            in_object = 1;
            start_index = i;
        }
        else if (in_object && (ir_readings[i] < IR_THRESHOLD || i == FRONT_SCAN_POINTS - 1))
        {
            int end_index = (i == FRONT_SCAN_POINTS - 1 && ir_readings[i] > IR_THRESHOLD) ? i : i - 1;
            int object_width_degree = (end_index - start_index + 1) * STEP_DEGREE;

            if (object_width_degree >= MIN_OBJECT_WIDTH_DEGREE)
            {
                int j;
                float min_distance = 999.0f;
                int min_distance_angle = 0;

                for (j = start_index; j <= end_index; j++)
                {
                    if (distances_cm[j] < min_distance && distances_cm[j] > 0)
                    {
                        min_distance = distances_cm[j];
                        min_distance_angle = j * STEP_DEGREE;
                    }
                }

                float center_angle = ((start_index * STEP_DEGREE) + (end_index * STEP_DEGREE)) / 2.0f;
                float width_cm = 2.0f * min_distance * sinf((float)object_width_degree * (float)M_PI / 360.0f);

                object_count++;
                sprintf(buffer, "%3d | %6.1f | %7.2f | %7.2f\r\n",
                        object_count, center_angle, min_distance, width_cm);
                uart_sendStr(buffer);
            }
            in_object = 0;
        }
    }

    if (object_count == 0) {
        uart_sendStr("No objects detected.\r\n");
    } else {
        sprintf(buffer, "\r\nTotal objects detected: %d\r\n", object_count);
        uart_sendStr(buffer);
    }
}

/* --- Check safety sensors --- */
void safety_check(oi_t *sensor_data)
{
    char buffer[64];

    /* Check for bumpers */
    if (sensor_data->bumpLeft || sensor_data->bumpRight)
    {
        uart_sendStr("\r\n*** WARNING: Obstacle collision detected! ***\r\n");
        move_backward(sensor_data, 50); /* Backup 5 cm */
    }

    /* Check for white tape boundaries */
    if ((sensor_data->cliffLeftSignal > WHITE_THRESHOLD &&
         sensor_data->cliffLeftSignal < BLUE_THRESHOLD) ||
        (sensor_data->cliffRightSignal > WHITE_THRESHOLD &&
         sensor_data->cliffRightSignal < BLUE_THRESHOLD) ||
        (sensor_data->cliffFrontLeftSignal > WHITE_THRESHOLD &&
         sensor_data->cliffFrontLeftSignal < BLUE_THRESHOLD) ||
        (sensor_data->cliffFrontRightSignal > WHITE_THRESHOLD &&
         sensor_data->cliffFrontRightSignal < BLUE_THRESHOLD))
    {
        /* Get which sensors detected the boundary */
        sprintf(buffer, "\r\n*** WHITE TAPE BOUNDARY DETECTED ***\r\n");
        uart_sendStr(buffer);

        /* Just back away from boundary (no turning) */
        uart_sendStr("Backing away from boundary...\r\n");
        move_backward(sensor_data, 50); /* Backup 5 cm */
    }

    /* Check for black craters/holes */
    if (sensor_data->cliffLeftSignal < BLACK_THRESHOLD ||
        sensor_data->cliffRightSignal < BLACK_THRESHOLD ||
        sensor_data->cliffFrontLeftSignal < BLACK_THRESHOLD ||
        sensor_data->cliffFrontRightSignal < BLACK_THRESHOLD)
    {
        /* Get which sensors detected the crater */
        sprintf(buffer, "\r\n*** BLACK CRATER DETECTED ***\r\n");
        uart_sendStr(buffer);

        /* Just back away from crater (no turning) */
        uart_sendStr("Backing away from crater...\r\n");
        move_backward(sensor_data, 50); /* Backup 5 cm */
    }

    /* Check for blue samples - values ABOVE the blue threshold */
    if (sensor_data->cliffLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffRightSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontRightSignal >= BLUE_THRESHOLD)
    {
        /* Display which sensors detected blue sample */
        sprintf(buffer, "\r\n*** BLUE SAMPLE DETECTED ***\r\n");
        uart_sendStr(buffer);

        /* Stop the robot */
        oi_setWheels(0, 0);

        /* Collect sample data by spinning */
        uart_sendStr("Collecting sample data (spinning 360°)...\r\n");
        turn_left(sensor_data, 360);

        uart_sendStr("Sample collection complete.\r\n");
    }
}

/* --- Updated blue sample check function --- */
void blue_sample_check(oi_t *sensor_data)
{
    char buffer[64];

    oi_update(sensor_data);

    /* Display all cliff sensor values for debugging */
    sprintf(buffer, "Cliff values: L=%d, FL=%d, FR=%d, R=%d\r\n",
            sensor_data->cliffLeftSignal,
            sensor_data->cliffFrontLeftSignal,
            sensor_data->cliffFrontRightSignal,
            sensor_data->cliffRightSignal);
    uart_sendStr(buffer);

    /* Check if any cliff sensor detects blue paper */
    if (sensor_data->cliffLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffRightSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontLeftSignal >= BLUE_THRESHOLD ||
        sensor_data->cliffFrontRightSignal >= BLUE_THRESHOLD)
    {
        uart_sendStr("*** BLUE SAMPLE DETECTED! ***\r\n");
        uart_sendStr("Collecting sample data (spinning)...\r\n");

        /* Stop first */
        oi_setWheels(0, 0);
        timer_waitMillis(500);

        /* Spin in place for sample collection simulation */
        turn_left(sensor_data, 360);

        uart_sendStr("Sample collection complete.\r\n");
    }
    else
    {
        uart_sendStr("No blue sample detected.\r\n");
    }
}

/* --- Calculate distance from IR sensor value --- */
float calculate_distance(int irValue)
{
    if (irValue <= 50) {  /* Prevent division by zero or very small values */
        return 150.0f;    /* Return a large value indicating "too far" */
    }

    /* Simple distance calculation */
    float distance = 100000.0f / (float)irValue;

    return distance;
}
