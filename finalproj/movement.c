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
#include <stdio.h>

/**
 * Move the robot forward by the specified distance in millimeters
 */
void move_forward(oi_t *sensor_data, double distance_mm)
{
    double sum = 0;
    distance_mm = distance_mm * 0.95; /* Adjustment factor - prevents going too far */

    /* Set wheels to move forward at moderate speed */
    oi_setWheels(100, 100);

    /* Loop until we've covered the distance */
    while (sum < distance_mm)
    {
        /* Update sensor data */
        oi_update(sensor_data);

        /* Check for obstacles */
        if (sensor_data->bumpLeft || sensor_data->bumpRight) {
            /* Stop if we hit something */
            char buffer[64];
            sprintf(buffer, "\r\nObstacle detected after %.1f mm\r\n", sum);
            uart_sendStr(buffer);
            break;
        }

        /* Check for holes/cliffs */
        if (sensor_data->cliffLeft || sensor_data->cliffRight ||
            sensor_data->cliffFrontLeft || sensor_data->cliffFrontRight) {
            /* Stop if we detect a cliff */
            char buffer[64];
            sprintf(buffer, "\r\nCliff detected after %.1f mm\r\n", sum);
            uart_sendStr(buffer);
            break;
        }

        /* Accumulate distance moved */
        sum += sensor_data->distance;
    }

    /* Stop wheels */
    oi_setWheels(0, 0);

    /* Wait for stability */
    timer_waitMillis(300);
}

/**
 * Move the robot backward by the specified distance in millimeters
 */
void move_backward(oi_t *sensor_data, double distance_mm)
{
    double sum = 0;

    /* Set wheels to move backward at moderate speed */
    oi_setWheels(-100, -100);

    /* Loop until we've covered the distance */
    while (sum > -distance_mm)
    {
        /* Update sensor data */
        oi_update(sensor_data);

        /* When moving backward, distance values are negative */
        sum += sensor_data->distance;
    }

    /* Stop wheels */
    oi_setWheels(0, 0);

    /* Wait for stability */
    timer_waitMillis(300);
}

/**
 * Fixed turn_right function - Use EXACTLY as shown in your test program
 */
void turn_right(oi_t *sensor_data, double degrees)
{
    char buffer[64];

    /* Simple check for valid input */
    if (degrees <= 0) {
        uart_sendStr("Invalid angle (zero or negative), not turning\r\n");
        return;
    }

    /* Prevent extremely large turns */
    if (degrees > 360) {
        degrees = 360;
        uart_sendStr("Limited to 360 degrees\r\n");
    }

    double sum = 0;
    int loop_count = 0;

    /* Set proper wheel speeds for right turn */
    oi_setWheels(-75, 75);

    uart_sendStr("Starting right turn loop...\r\n");

    /* Loop until turned enough or timeout */
    while (sum < degrees && loop_count < 300)
    {
        oi_update(sensor_data);

        /* Debug output every 20 iterations */
        if (loop_count % 20 == 0) {
            sprintf(buffer, "Angle: %.2f, Sum: %.2f\r\n", sensor_data->angle, sum);
            uart_sendStr(buffer);
        }

        /* IMPORTANT: For right turns with the Cybot, angle values are NEGATIVE */
        sum -= sensor_data->angle;

        loop_count++;

        /* Short delay to allow sensor updates */
        timer_waitMillis(10);
    }

    /* Stop wheels */
    oi_setWheels(0, 0);

    sprintf(buffer, "Turn complete after %d iterations. Final angle: %.1f\r\n",
            loop_count, sum);
    uart_sendStr(buffer);

    if (loop_count >= 300) {
        uart_sendStr("WARNING: Turn timed out!\r\n");
    }

}

/**
 * Fixed turn_left function - Use EXACTLY as shown in your test program
 */
void turn_left(oi_t *sensor_data, double degrees)
{
    char buffer[64];

    /* Simple check for valid input */
    if (degrees <= 0) {
        uart_sendStr("Invalid angle (zero or negative), not turning\r\n");
        return;
    }

    /* Prevent extremely large turns */
    if (degrees > 360) {
        degrees = 360;
        uart_sendStr("Limited to 360 degrees\r\n");
    }

    double sum = 0;
    int loop_count = 0;

    /* Set proper wheel speeds for left turn */
    oi_setWheels(75, -75);

    uart_sendStr("Starting left turn loop...\r\n");

    /* Loop until turned enough or timeout */
    while (sum < degrees && loop_count < 300)
    {
        oi_update(sensor_data);

        /* Debug output every 20 iterations */
        if (loop_count % 20 == 0) {
            sprintf(buffer, "Angle: %.2f, Sum: %.2f\r\n", sensor_data->angle, sum);
            uart_sendStr(buffer);
        }

        /* IMPORTANT: For left turns with the Cybot, angle values are POSITIVE */
        sum += sensor_data->angle;

        loop_count++;

        /* Short delay to allow sensor updates */
        timer_waitMillis(10);
    }

    /* Stop wheels */
    oi_setWheels(0, 0);

    sprintf(buffer, "Turn complete after %d iterations. Final angle: %.1f\r\n",
            loop_count, sum);
    uart_sendStr(buffer);

    if (loop_count >= 300) {
        uart_sendStr("WARNING: Turn timed out!\r\n");
    }

}
/**
 * Helper function: Move forward with obstacle detection and avoidance
 */
void move_forward_smart(oi_t *sensor_data, double distance_mm)
{
    double distance_moved = 0;
    int backup_distance = 150; /* mm */
    int bump_moveaway_distance = 250; /* mm */

    /* Start moving forward */
    oi_setWheels(100, 100);

    while (distance_moved < distance_mm)
    {
        /* Update sensor data */
        oi_update(sensor_data);

        /* Update distance moved */
        distance_moved += sensor_data->distance;

        /* Check for left bump */
        if (sensor_data->bumpLeft) {
            /* Stop when we hit something */
            oi_setWheels(0, 0);

            /* Back up */
            move_backward(sensor_data, backup_distance);
            distance_moved -= backup_distance;

            /* Turn right to avoid obstacle */
            turn_right(sensor_data, 90);

            /* Move forward to go around */
            move_forward(sensor_data, bump_moveaway_distance);

            /* Turn left to return to original direction */
            turn_left(sensor_data, 90);

            /* Resume forward movement */
            oi_setWheels(100, 100);
        }

        /* Check for right bump */
        if (sensor_data->bumpRight) {
            /* Stop when we hit something */
            oi_setWheels(0, 0);

            /* Back up */
            move_backward(sensor_data, backup_distance);
            distance_moved -= backup_distance;

            /* Turn left to avoid obstacle */
            turn_left(sensor_data, 90);

            /* Move forward to go around */
            move_forward(sensor_data, bump_moveaway_distance);

            /* Turn right to return to original direction */
            turn_right(sensor_data, 90);

            /* Resume forward movement */
            oi_setWheels(100, 100);
        }
    }

    /* Stop wheels when done */
    oi_setWheels(0, 0);
}
