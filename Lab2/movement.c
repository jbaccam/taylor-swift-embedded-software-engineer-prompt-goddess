/*
 * movement.c
 *
 *  Created on: Feb 7, 2025
 *      Author: jjbaccam
 */

#include "open_interface.h"
void turn_right(oi_t *sensor_data, double degrees);
void turn_left(oi_t *sensor_data, double degrees);
void move_forward(oi_t *sensor_data, double distance_mm);
void move_backward(oi_t *sensor_data, double distance_mm);
void move_square(oi_t *sensor_data);
void move_forward_smart(oi_t *sensor_data, double distance_mm);

void main()
{

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()
    move_forward_smart(sensor_data, 2000);
    oi_free(sensor_data); // do this once at end of main()
}

void move_forward(oi_t *sensor_data, double distance_mm)
{
    double sum = 0; // distance member in oi_t struct is type double, this tracks the total distance moved
    distance_mm = distance_mm * 0.95; // makes up going too far
    oi_setWheels(250, 250); // move forward at full speed

    // we loop until sum accumulates enough negative values to reach -distance
    while (sum <= distance_mm)
    {
        oi_update(sensor_data); // update sensor data
        sum += sensor_data->distance; // use -> notation since pointer, accumulates negative distance values
    }

    oi_setWheels(0, 0); //stop
    timer_waitMillis(500);

}

void move_backward(oi_t *sensor_data, double distance_mm)
{
    double sum = 0; // distance member in oi_t struct is type double, this tracks the total distance moved
    distance_mm = distance_mm; // makes up going too far
    oi_setWheels(-250, -250); // move forward at full speed

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

void turn_right(oi_t *sensor_data, double degrees)
{
    double sum = 0;
    degrees = degrees * -1 + 17;  // Changed from +12 to +17
    oi_setWheels(-200, 200);

    while (sum >= degrees)
    {
        oi_update(sensor_data);
        sum += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(500);
}

void turn_left(oi_t *sensor_data, double degrees)
{
    degrees -= 17;  // Changed from -12 to -17
    double sum = 0;
    oi_setWheels(200, -200);

    while (sum <= degrees)
    {
        oi_update(sensor_data);
        sum += sensor_data->angle;
    }

    oi_setWheels(0, 0);
    timer_waitMillis(500);
}

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

void move_forward_smart(oi_t *sensor_data, double distance_mm)
{
    double distance_moved = 0;
    int right_bump_status = 0;
    int left_bump_status = 0;
    int backup_distance = 150; //mm
    int bump_moveaway_distance = 250; //mm
    oi_setWheels(250, 250);

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
            oi_setWheels(250, 250); // go forward the remaining distance
        }

        if (right_bump_status != 0) // right bumper triggered
        {
            oi_setWheels(0, 0);
            move_backward(sensor_data, backup_distance);  // backup after collision
            distance_moved -= backup_distance; // add distance backed up to total distance covered

            turn_left(sensor_data, 90); // turn back
            move_forward(sensor_data, bump_moveaway_distance); // move forward the 250mm

            turn_right(sensor_data, 90); // turn back
            oi_setWheels(250, 250);  // go forward the remaining distance
        }

    }
    oi_setWheels(0, 0); //stop
}

