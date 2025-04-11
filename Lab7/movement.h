/**
 * movement.h
 *
 * Movement control functions for the CyBot
 *
 * @author Your Name
 * @date March 24, 2025
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

// Movement parameters
#define MOVE_SPEED 250
#define TURN_SPEED 200
#define BACKUP_DISTANCE 150
#define OBSTACLE_AVOID_DISTANCE 250
#define STOP_DISTANCE 10.0  // Stop 10cm from the target object

// Basic movement functions
void move_forward(oi_t *sensor_data, double distance_mm);
void move_backward(oi_t *sensor_data, double distance_mm);
void turn_right(oi_t *sensor_data, double degrees);
void turn_left(oi_t *sensor_data, double degrees);
int go_to_position(oi_t *sensor_data, float angle, float distance_cm);
void go_around(oi_t *sensor_data, int left_or_right);


// Smart movement with obstacle avoidance
void move_forward_smart(oi_t *sensor_data, double distance_mm);

#endif /* MOVEMENT_H_ */
