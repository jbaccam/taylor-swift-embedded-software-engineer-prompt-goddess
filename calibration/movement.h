/*
 * movement.h
 *
 *  Created on: Sep 9, 2021
 *      Author: tdbolton
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"
void turn_clockwise(oi_t *sensor, int degrees);
void turn_counterclockwise(oi_t *sensor, int degrees);
void move_backwards(oi_t *sensor, int centimeters);
void move_forward(oi_t *sensor, int centimeters);
void go_to_object(oi_t *sensor, int centimeters);
void autoturn_clockwise(oi_t *sensor, int degrees);
void autoturn_counterclockwise(oi_t *sensor, int degrees);

#endif /* MOVEMENT_H_ */
