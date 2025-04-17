/*
 * servo.h
 *
 *  Created on: April 17, 2025
 *      Author: jjbaccam
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "Timer.h"

void servo_init(void);
void servo_move(float degrees);
void calibrate_servo(void);



#endif /* SERVO_H_ */
