/*
 * servo.h
 *
 *  Created on: Apr 17, 2025
 *      Author: jjbaccam
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/interrupt.h"
#include "Timer.h"

// Calibration values for the servo
// These will be set during calibration
extern int right_calibration_value;  // Match value for 0 degrees (right)
extern int left_calibration_value;   // Match value for 180 degrees (left)

/**
 * Initialize the servo motor using PWM on Timer 1B (PB5)
 */
void servo_init(void);

/**
 * Move the servo to the specified angle (0-180 degrees)
 * @param degrees Angle to move to (0-180)
 */
void servo_move(float degrees);

/**
 * Control the servo using pushbuttons
 * - Button 1: Move 1 degree in current direction
 * - Button 2: Move 5 degrees in current direction
 * - Button 3: Toggle direction (CW/CCW)
 * - Button 4: Move to extremes (5° in CW mode, 175° in CCW mode)
 */
void servo_button_control(void);

/**
 * Calibrate the servo for accurate positioning using the following process:
 * 1. Use Button 1 and Button 2 to move servo left/right until it's at 0 degrees
 * 2. Press Button 4 to confirm and store the calibration value for 0 degrees
 * 3. Use Button 1 and Button 2 to move servo left/right until it's at 180 degrees
 * 4. Press Button 4 to confirm and store the calibration value for 180 degrees
 */
void servo_calibrate(void);

#endif /* SERVO_H_ */
