/**
 * main.c
 */
#include "Timer.h"
#include "lcd.h"
#include <string.h>
#include "cyBot_uart.h"
#include "cyBot_Scan.h"
#include "open_interface.h"
#include "movement.h"  // includes turn_left() and turn_right()

// function defs
void send_string_putty(const char *str);
void send_string_to_putty();
void clear_putty_terminal();
void detect_objects(oi_t *sensor_data);
void scan_all_angles();
void get_angle_array();
void moving_median_filter_3(const float distance[], float distance_filtered[], int n);
void face_object(oi_t *sensor_data, float degrees, float smallestWidthDistance);

float median_of_3(float a, float b, float c);
void compute_diff(const float dist_filt[], float diff[], int n);

// cybot distance enums
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define STEP 2
#define THRESHOLD 40.0f

// cybot 7 calibration values
#define LEFT_CAL 1261750
#define RIGHT_CAL 232750

// calc num points
#define NUM_POINTS (((MAX_ANGLE - MIN_ANGLE) / STEP) + 1)

// declare arrays for storing data
float angles[NUM_POINTS];
float distance[NUM_POINTS];
float distance_filtered[NUM_POINTS];
float diff_array[NUM_POINTS];

// define a minimum width to skip super small objects (e.g., from noise)
#define MIN_OBJECT_WIDTH 6.0f

int main(void)
{

    // Allocate sensor_data BEFORE calling oi_init()
    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data);           // <-- MUST call this so the wheels can move

    timer_init(); // init timer
    lcd_init();   // init lcd (clears display)
    cyBot_uart_init();

    right_calibration_value = LEFT_CAL;
    left_calibration_value  = RIGHT_CAL;

    cyBOT_Scan_t scan;
    cyBOT_init_Scan(0b0111);
    cyBOT_Scan(0, &scan);
    timer_waitMillis(2000);

    while (1)
    {
        char m;
        m = cyBot_getByte();
        if (m == 'q') {
            // break out if user wants to quit
            break;
        }
        else if (m == '\n' || m == 'm')
        {
            clear_putty_terminal();
            // fill angles array
            get_angle_array();

            // perform full scan and store raw distances
            scan_all_angles();

            // filter data using a 3-point median filter (low-pass)
            moving_median_filter_3(distance, distance_filtered, NUM_POINTS);

            // compute difference array (high-pass)
            compute_diff(distance_filtered, diff_array, NUM_POINTS);

            // detect objects using edge detection
            detect_objects(sensor_data);
        }
    }
    oi_free(sensor_data); // do this once at end of main()
    return 0;
}

// A simple function to populate the angles[] array from 0 degrees to 180 degrees in 2 degrees increments:
void get_angle_array(void)
{
    float angleVal = MIN_ANGLE;
    int i;

    for (i = 0; i < NUM_POINTS; i++)
    {
        angles[i] = angleVal;
        angleVal += STEP;
    }
}

// Use cyBOT_Scan to get the PING distance at each angle and store it:
void scan_all_angles(void)
{
    right_calibration_value = LEFT_CAL;
    left_calibration_value  = RIGHT_CAL;
    cyBOT_Scan_t scan;
    int i;
    char buffer[100];

    sprintf(buffer, "Degrees   PING Distance (cm)\r\n");
    send_string_putty(buffer);

    for (i = 0; i < NUM_POINTS; i++)
    {
        int currAngle = (int) angles[i];  // e.g., 0, 2, 4, ..., 180
        cyBOT_Scan(currAngle, &scan);
        distance[i] = scan.sound_dist; // Raw distance from PING
        sprintf(buffer, "%3.0f       %6.2f\r\n", angles[i], distance[i]);
        send_string_putty(buffer);
    }
}

// apply a 3-point median filter
void moving_median_filter_3(const float distance_in[],
                             float distance_out[], int n)
{
    // distance_in[] = unfiltered
    // distance_out[] = filtered
    // n = NUM_POINTS

    // copy the boundaries from unfiltered to filtered
    distance_out[0]     = distance_in[0];
    distance_out[n - 1] = distance_in[n - 1];

    char buffer[50];
    // apply the filter for the interior points (not including boundaries)
    int i;
    for (i = 1; i < n - 1; i++)
    {
        float m = median_of_3(distance_in[i - 1],
                              distance_in[i],
                              distance_in[i + 1]);
        distance_out[i] = m;
    }

    sprintf(buffer, "\r\nFiltered Data (Median of 3):\r\n");
    send_string_putty(buffer);
    for (i = 0; i < n; i++)
    {
        sprintf(buffer, "%3.0f\t%6.2f\r\n", angles[i], distance_out[i]);
        send_string_putty(buffer);
    }
}

void detect_objects(oi_t *sensor_data)
{
    // EDGE APPROACH

    // We'll use the diff_array (global) computed by compute_diff() to find edges.
    // Large negative diff => leading edge, large positive diff => trailing edge.

    float EDGE_THRESHOLD = THRESHOLD;

    // Variables for tracking an object
    int onObject = 0;       // bool for onObject
    int startIndex = 0;
    int i;
    float smallestWidth = 99999.0f;
    float smallestWidthAngle = 90.0f; // default to forward if none found
    float smallestWidthDistance = 0.0f;

    // We'll iterate through angles from i=1 to i < NUM_POINTS
    // because diff_array[i] compares distance_filtered[i] - distance_filtered[i-1]
    for (i = 1; i < NUM_POINTS; i++)
    {
        // Leading edge (sudden drop in distance)
        if (!onObject && (diff_array[i] < -EDGE_THRESHOLD))
        {
            startIndex = i;
            onObject   = 1;
        }
        // Trailing edge (sudden rise in distance)
        else if (onObject && (diff_array[i] > EDGE_THRESHOLD))
        {
            int endIndex = i;
            onObject = 0; // we've found the far edge

            // object stats
            float startAngle  = angles[startIndex];
            float endAngle    = angles[endIndex];
            float width       = endAngle - startAngle;
            float centerAngle = (startAngle + endAngle) / 2.0f;

            // skip super small objects
            if (width < MIN_OBJECT_WIDTH)
            {
                // skip this object entirely
                continue;
            }

            // distance at midpoint (or find min in [startIndex..endIndex])
            float minDist = 99999.0f;
            int k;
            for (k = startIndex; k <= endIndex; k++)
            {
                if (distance_filtered[k] < minDist)
                {
                    minDist = distance_filtered[k];
                }
            }
            float objectDistance = minDist;

            //print stats to Putty
            char outbuf[100];
            sprintf(outbuf, "\nObject found (edge-based):\r\n"
                            " - Start angle:  %.2f\r\n"
                            " - End angle:    %.2f\r\n"
                            " - Center angle: %.2f\r\n"
                            " - Radial width: %.2f deg\r\n"
                            " - Distance:     %.2f cm\r\n\r\n",
                    startAngle, endAngle, centerAngle, width, objectDistance);
            send_string_putty(outbuf);

            // update smallest object if needed
            if (width < smallestWidth) {
                smallestWidth = width;
                smallestWidthAngle = centerAngle;
                smallestWidthDistance = objectDistance - 10.0;

            }
        }
    }

    // if we never got a trailing edge, but are still on an object at the end
    if (onObject)
    {
        int endIndex = NUM_POINTS - 1;
        onObject = 0;

        float startAngle  = angles[startIndex];
        float endAngle    = angles[endIndex];
        float width       = endAngle - startAngle;
        float centerAngle = (startAngle + endAngle) / 2.0f;

        // skip super small objects
        if (width >= MIN_OBJECT_WIDTH)
        {
            float minDist = 99999.0f;
            int k;
            for (k = startIndex; k <= endIndex; k++)
            {
                if (distance_filtered[k] < minDist)
                {
                    minDist = distance_filtered[k];
                }
            }
            float objectDistance = minDist;

            char outbuf[100];
            sprintf(outbuf, "\nObject found (end-of-scan):\r\n"
                            " - Start angle:  %.2f\r\n"
                            " - End angle:    %.2f\r\n"
                            " - Center angle: %.2f\r\n"
                            " - Radial width: %.2f deg\r\n"
                            " - Distance:     %.2f cm\r\n\r\n",
                    startAngle, endAngle, centerAngle, width, objectDistance);
            send_string_putty(outbuf);

            if (width < smallestWidth) {
                smallestWidth = width;
                smallestWidthAngle = centerAngle;
                smallestWidthDistance = objectDistance - 10.0;
            }
        }
    }

    // face the smallest object only if we found one
    if (smallestWidth < 99999.0f)
    {
        face_object(sensor_data, smallestWidthAngle, smallestWidthDistance);
    }
    else
    {
        send_string_putty("\nNo objects detected.\r\n");
    }
}

void get_ping_data()
{
    right_calibration_value = LEFT_CAL;
    left_calibration_value  = RIGHT_CAL;
    int angle;
    cyBOT_Scan_t scan;

    char buffer[50];
    int i;

    while (1)
    {
        char m;
        m = cyBot_getByte();
        clear_putty_terminal();
        if (m == 'm')
        {
            sprintf(buffer, "Degrees   PING Distance (cm)\r\n");
            int stringLen = strlen(buffer);
            for (i = 0; i < stringLen; i++)
            {
                cyBot_sendByte(buffer[i]);
            }

            for (angle = 0; angle <= 180; angle += 2)
            {
                cyBOT_Scan(angle, &scan);
                float distance_cm = scan.sound_dist;
                // Format: angle in a 3-digit field, distance in a 6-character field with 2 decimals
                sprintf(buffer, "%3d       %6.2f\r\n", angle, distance_cm);
                for (i = 0; buffer[i] != '\0'; i++)
                {
                    cyBot_sendByte(buffer[i]);
                }
            }
        }
    }
}

// save some code space
void send_string_putty(const char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        cyBot_sendByte(str[i]);
        i++;
    }
}

// Send messages from putty to cybot back to putty
void send_string_to_putty()
{
    while (1)
    {
        char message[25];
        int index = 0;
        char k;
        char test[25];
        message[0] = '\0';
        while (1)
        {
            k = cyBot_getByte();

            if (index < (sizeof(message) - 1))
            {
                message[index++] = k;
            }

            if (k == '\x0d') // enter key
            {
                break;
            }
        }
        message[index] = '\0';

        // putty confirmation message
        sprintf(test, "Got an %s\n", message);
        int i;
        int stringLen = strlen(test);
        for (i = 0; i < stringLen; i++)
        {
            cyBot_sendByte(test[i]);
        }
    }
}

// Helper function to clear the putty terminal using ANSI escape codes
void clear_putty_terminal()
{
    char clearSeq[] = "\033[2J\033[H";  // clear screen and move cursor home
    int i;
    for (i = 0; clearSeq[i] != '\0'; i++)
    {
        cyBot_sendByte(clearSeq[i]);
    }
}

float median_of_3(float a, float b, float c)
{
    // You can implement this in many ways. One quick approach:
    if (a > b)
    {
        if (b > c) return b;       // a > b > c
        else if (a > c) return c;  // a > c >= b
        else return a;             // c >= a > b
    }
    else
    {
        // b >= a
        if (a > c) return a;       // b >= a > c
        else if (b > c) return c;  // b > c >= a
        else return b;             // c >= b >= a
    }
}

// compute the difference array (high-pass)
void compute_diff(const float dist_filt[], float diff[], int n)
{
    diff[0] = 0.0f; // or some sentinel
    int i;
    for (i = 1; i < n; i++)
    {
        diff[i] = dist_filt[i] - dist_filt[i - 1];
    }
}

void face_object(oi_t *sensor_data, float centerAngle, float smallestWidthDistance)
{
    // We assume 0° is right, 180° is left, 90° is straight ahead.

    // If the object is to the right of center, we turn right.
    if (centerAngle < 90)
    {
        double degreesToTurn = 90 - centerAngle;
        turn_right(sensor_data, degreesToTurn);
    }
    // If the object is to the left of center, we turn left.
    else if (centerAngle > 90)
    {
        double degreesToTurn = centerAngle - 90;
        turn_left(sensor_data, degreesToTurn);
    }

    // If centerAngle == 90, do nothing because we are already facing it.

    // now we move forward to it and make sure its within 10cm
    smallestWidthDistance *= 10;

    // calibrating
    smallestWidthDistance *= 0.95;
    move_forward(sensor_data, smallestWidthDistance);

}



