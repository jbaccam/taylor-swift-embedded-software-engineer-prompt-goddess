/*
 * movement.h
 *
 *  Created on: Feb 7, 2025
 *      Author: jjbaccam
 */


#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"

// Prototypes of movement functions:
void turn_right(oi_t *sensor_data, double degrees);
void turn_left(oi_t *sensor_data, double degrees);
void move_forward(oi_t *sensor_data, double distance_mm);
void move_backward(oi_t *sensor_data, double distance_mm);
void move_square(oi_t *sensor_data);
void move_forward_smart(oi_t *sensor_data, double distance_mm);

#endif /* MOVEMENT_H_ */
