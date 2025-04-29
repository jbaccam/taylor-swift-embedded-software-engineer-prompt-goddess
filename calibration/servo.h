/*
 * servo.h
 *
 *  Created on: Nov 4, 2021
 *      Author: tdbolton
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"
#include "Timer.h"

void servo_init(void);
void servo_move(float degrees);
void calibrate_servo(void);



#endif /* SERVO_H_ */
