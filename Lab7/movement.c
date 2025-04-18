/*
 * movement.c
 *
 * Implementation of movement control functions for the CyBot
 *
 * @author Jeremiah Baccam, Luke Patterson
 */

#include "open_interface.h"
#include "movement.h"
#include "Timer.h"
#include "uart.h"

/**
 * Move the robot forward by the specified distance in millimeters
 */
void move_forward(oi_t *sensor_data, double distance_mm)
{
    double sum = 0; // distance member in oi_t struct is type double, this tracks the total distance moved
    distance_mm = distance_mm * 0.95; // makes up going too far
    oi_setWheels(100, 100); // move forward at full speed

    // we loop until sum accumulates enough negative values to reach -distance
    while (sum <= distance_mm)
    {
        oi_update(sensor_data); // update sensor data
        sum += sensor_data->distance; // use -> notation since pointer, accumulates negative distance values
    }

    oi_setWheels(0, 0); //stop
    timer_waitMillis(500);
}

/**
 * Move the robot backward by the specified distance in millimeters
 */
void move_backward(oi_t *sensor_data, double distance_mm)
{
    double sum = 0; // distance member in oi_t struct is type double, this tracks the total distance moved
    distance_mm = distance_mm; // makes up going too far
    oi_setWheels(-100, -100); // move forward at full speed

    // since robot is moving backwards, sensor_data->distance will be negative
    // we loop until sum accumulates enough negative values to reach -distance
    while (sum > -distance_mm)
    {
        oi_update(sensor_data); // update sensor data
        sum += sensor_data->distance; // use -> notation since pointer, accumulates negative distance values
    }

    oi_setWheels(0, 0); //stop
    timer_waitMillis(500);
}

/**
 * Turn the robot right by the specified angle in degrees
 */
void turn_right(oi_t *sensor_data, double degrees)
{
    double sum = 0; // distance member in oi_t struct is type double
    degrees = degrees * -1 + 17; // Calibration offset
    oi_setWheels(-100, 100); // turn speed

    while (sum >= degrees)
    {
        oi_update(sensor_data);
        sum += sensor_data->angle; // use -> notation since pointer
    }

    oi_setWheels(0, 0); //stop
    timer_waitMillis(500);
}

/**
 * Turn the robot left by the specified angle in degrees
 */
void turn_left(oi_t *sensor_data, double degrees)
{
    degrees -= 17; // Calibration offset
    double sum = 0; // distance member in oi_t struct is type double
    oi_setWheels(100, -100); //move forward at full speed

    while (sum <= degrees)
    {
        oi_update(sensor_data);
        sum += sensor_data->angle; // use -> notation since pointer
    }

    oi_setWheels(0, 0); //stop
    timer_waitMillis(500);
}

/**
 * Move the robot in a square pattern
 */
void move_square(oi_t *sensor_data)
{
    move_forward(sensor_data, 500);
    turn_right(sensor_data, 90);

    move_forward(sensor_data, 500);
    turn_right(sensor_data, 90);

    move_forward(sensor_data, 500);
    turn_right(sensor_data, 90);

    move_forward(sensor_data, 500);
    turn_right(sensor_data, 90);
}

/**
 * Move forward with obstacle avoidance
 */
void move_forward_smart(oi_t *sensor_data, double distance_mm)
{
    double distance_moved = 0;
    int right_bump_status = 0;
    int left_bump_status = 0;
    int backup_distance = 150; //mm
    int bump_moveaway_distance = 250; //mm
    oi_setWheels(100, 100);

    while (distance_moved < distance_mm)
    {
        oi_update(sensor_data); //update sensor data
        distance_moved += sensor_data->distance; //update distance value
        right_bump_status = sensor_data->bumpRight; //update the value of the right bump sensor
        left_bump_status = sensor_data->bumpLeft; //update the value of the left bump sensor

        if (left_bump_status != 0) // left bumper triggered
        {
            oi_setWheels(0, 0);
            move_backward(sensor_data, backup_distance); // backup after collision
            distance_moved -= backup_distance; // add distance backed up to total distance covered

            turn_right(sensor_data, 90); // turn right, away from the obstacle
            move_forward(sensor_data, bump_moveaway_distance); // move forward the 250mm

            turn_left(sensor_data, 90); // turn back
            oi_setWheels(100, 100); // go forward the remaining distance
        }

        if (right_bump_status != 0) // right bumper triggered
        {
            oi_setWheels(0, 0);
            move_backward(sensor_data, backup_distance); // backup after collision
            distance_moved -= backup_distance; // add distance backed up to total distance covered

            turn_left(sensor_data, 90); // turn back
            move_forward(sensor_data, bump_moveaway_distance); // move forward the 250mm

            turn_right(sensor_data, 90); // turn back
            oi_setWheels(100, 100);  // go forward the remaining distance
        }
    }

    oi_setWheels(0, 0); //stop
}

/**
 * Helper function: Move forward at a faster speed
 */
void faster_move_forward(oi_t *sensor_data, double distance_mm)
{
    double sum = 0;
    distance_mm *= 0.95; // Adjustment factor
    oi_setWheels(200, 200); // Faster speed for go-around

    while (sum <= distance_mm)
    {
        oi_update(sensor_data);
        sum += sensor_data->distance;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300); // Shorter wait time
}

/**
 * Helper function: Turn right at a faster speed
 */
void faster_turn_right(oi_t *sensor_data, double degrees)
{
    double sum = 0;
    degrees = degrees * -1 + 17; // Calibration offset
    oi_setWheels(-200, 200); // Faster turning

    while (sum >= degrees)
    {
        oi_update(sensor_data);
        sum += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}

/**
 * Helper function: Turn left at a faster speed
 */
void faster_turn_left(oi_t *sensor_data, double degrees)
{
    degrees -= 17; // Calibration offset
    double sum = 0;
    oi_setWheels(200, -200); // Faster turning

    while (sum <= degrees)
    {
        oi_update(sensor_data);
        sum += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(300);
}

/**
 * - go_to_position function for correct scanning direction
 * - After obstacle navigation, turns properly to face south
 * - Maintains faster wheel speeds during obstacle avoidance
 */
int go_to_position(oi_t *sensor_data, float angle, float distance_cm)
{
    char buffer[100];

    // DEBUGGING: Print the actual values coming in
    sprintf(buffer, "DEBUG - Received: angle=%.1f, distance=%.1f\r\n", angle,
            distance_cm);
    uart_sendStr(buffer);

    // 1. Turn to face the target
    float degreesToTurn = 0;
    if (angle < 90)
    {
        // Object is to the right
        degreesToTurn = 90 - angle;
        sprintf(buffer, "Turning right %.1f degrees\r\n", degreesToTurn);
        uart_sendStr(buffer);

        // Make sure the turn angle is sufficiently large to matter
        if (degreesToTurn >= 2.0)
        {
            turn_right(sensor_data, degreesToTurn);
            uart_sendStr("Turn completed\r\n");
        }
        else
        {
            uart_sendStr("Turn angle too small, skipping\r\n");
        }
    }
    else if (angle > 90)
    {
        // Object is to the left
        degreesToTurn = angle - 90;
        sprintf(buffer, "Turning left %.1f degrees\r\n", degreesToTurn);
        uart_sendStr(buffer);

        // Make sure the turn angle is sufficiently large to matter
        if (degreesToTurn >= 2.0)
        {
            turn_left(sensor_data, degreesToTurn);
            uart_sendStr("Turn completed\r\n");
        }
        else
        {
            uart_sendStr("Turn angle too small, skipping\r\n");
        }
    }

    // 2. Calculate how far to move
    float move_distance = distance_cm - 10.0f; // Stop 10cm from object

    // Check if already close enough
    if (move_distance <= 0)
    {
        uart_sendStr("Already close to object\r\n");
        return 0;
    }

    // Cap to safe distance
    if (move_distance > 100.0f)
    {
        move_distance = 100.0f;
    }

    // Convert to mm
    float move_distance_mm = move_distance * 10.0f;

    // Print the intended movement
    sprintf(buffer, "Moving forward %.1f cm (%.0f mm)\r\n", move_distance,
            move_distance_mm);
    uart_sendStr(buffer);

    // 3. Move forward, watching for bumps
    double distance_moved = 0;

    // Start moving at normal speed for approaching objects
    oi_setWheels(100, 100);
    int right_bump_status = 0;
    int left_bump_status = 0;
    while (distance_moved < move_distance_mm)
    {
        oi_update(sensor_data); //update sensor data
        distance_moved += sensor_data->distance; //update distance value
        right_bump_status = sensor_data->bumpRight; //update the value of the right bump sensor
        left_bump_status = sensor_data->bumpLeft; //update the value of the left bump sensor

        if (left_bump_status || right_bump_status)
        {
            if (left_bump_status != 0) // left bumper triggered
            {
                go_around(sensor_data, 1);
            }

            if (right_bump_status != 0) // right bumper triggered
            {
                go_around(sensor_data, 0);
            }

            uart_sendStr("Navigation complete. Performing rescan...\r\n");
            return 1; // Signal caller to rescan
        }
    }

// Stop motion
    oi_setWheels(0, 0);
    uart_sendStr("Reached target!\r\n");

    return 0; // No bumps occurred
}


void go_around(oi_t *sensor_data, int left_or_right)
{
    if (left_or_right)
    {
        oi_setWheels(0, 0);
        move_backward(sensor_data, 150); // backup after collision

        // Hit left bumper - go around to the right
        uart_sendStr("Going around to the right\r\n");
        faster_turn_right(sensor_data, 90);
        uart_sendStr("Moving to the side (700mm)\r\n");
        faster_move_forward(sensor_data, 700); // 700mm to the side

        faster_turn_left(sensor_data, 90); // Face forward again (north)
        uart_sendStr("Moving forward past obstacle (1000mm)\r\n");
        faster_move_forward(sensor_data, 1200); // 1 meter forward --------------------------------------------------------TESTING-----------------------------------

        faster_turn_left(sensor_data, 90); // Turn toward original path (facing west)
        uart_sendStr("Moving back toward original path (700mm)\r\n");
        faster_move_forward(sensor_data, 700); // 700mm back toward path

        // PROPERLY FIXED: Now we're facing west, need to turn right 90� to face south
        uart_sendStr("Turning right 90 degrees to face south\r\n");
        faster_turn_left(sensor_data, 90);   // Turn right to face south
    }
    else
    {
        oi_setWheels(0, 0);
        move_backward(sensor_data, 150); // backup after collision

        // Hit right bumper - go around to the left
        uart_sendStr("Going around to the left\r\n");

        faster_turn_left(sensor_data, 90);
        uart_sendStr("Moving to the side (700mm)\r\n");
        faster_move_forward(sensor_data, 700); // 700mm to the side

        faster_turn_right(sensor_data, 90); // Face forward again (north)
        uart_sendStr("Moving forward past obstacle (1000mm)\r\n");
        faster_move_forward(sensor_data, 1200); // 1 meter forward --------------------------------------------------------TESTING-----------------------------------

        faster_turn_right(sensor_data, 90); // Turn toward original path (facing east)
        uart_sendStr("Moving back toward original path (700mm)\r\n");
        faster_move_forward(sensor_data, 700); // 700mm back toward path

        // PROPERLY FIXED: Now we're facing east, need to turn left 90� to face south
        uart_sendStr("Turning left 90 degrees to face south\r\n");
        faster_turn_right(sensor_data, 90);   // Turn left to face south
    }
}
