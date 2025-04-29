/*
 * movement.h
 *
 * Header file for Cybot movement functions
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

/**
 * Moves the Cybot forward by the specified distance
 * @param sensor_data Pointer to oi_t sensor struct
 * @param distance_mm Distance to move in millimeters
 */
void move_forward(oi_t *sensor_data, double distance_mm);

/**
 * Moves the Cybot backward by the specified distance
 * @param sensor_data Pointer to oi_t sensor struct
 * @param distance_mm Distance to move in millimeters
 */
void move_backward(oi_t *sensor_data, double distance_mm);

/**
 * Turns the Cybot right (clockwise) by the specified angle
 * @param sensor_data Pointer to oi_t sensor struct
 * @param degrees Angle to turn in degrees
 */
void turn_right(oi_t *sensor_data, double degrees);

/**
 * Turns the Cybot left (counter-clockwise) by the specified angle
 * @param sensor_data Pointer to oi_t sensor struct
 * @param degrees Angle to turn in degrees
 */
void turn_left(oi_t *sensor_data, double degrees);

/**
 * Moves the Cybot forward with obstacle avoidance
 * @param sensor_data Pointer to oi_t sensor struct
 * @param distance_mm Distance to move in millimeters
 */
void move_forward_smart(oi_t *sensor_data, double distance_mm);

#endif /* MOVEMENT_H_ */
