/*
 * lab10.c
 *
 *  Created on: Apr 17, 2025
 *      Author: jjbaccam
 */
#include "Timer.h"
#include "lcd.h"
#include "button.h"
#include "servo.h"

int main(void) {
    // Initialize hardware components
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    button_init();
    servo_init();

    // Display welcome message
    lcd_printf("Lab 10: Servo\nControl with PWM\n\nInitializing...");
    timer_waitMillis(1000);

    // Menu for selecting the part to run
    lcd_printf("Select part:\nB1: Basic Movement\nB2: Button Control\nB3: Calibration");

    // Wait for user selection
    while (1) {
        uint8_t button = button_getButton();

        if (button) {
            // Add debounce delay
            timer_waitMillis(200);

            if (button == 0x01) {
                // Part 1: Basic Movement Test
                lcd_printf("Part 1: Testing\nServo Movement");
                timer_waitMillis(1000);

                // Move to center position (90 degrees)
                servo_move(90);
                lcd_printf("Position: 90 deg\n(center)");
                timer_waitMillis(2000);

                // Move to 30 degrees
                servo_move(30);
                lcd_printf("Position: 30 deg");
                timer_waitMillis(2000);

                // Move to 150 degrees
                servo_move(150);
                lcd_printf("Position: 150 deg");
                timer_waitMillis(2000);

                // Return to center
                servo_move(90);
                lcd_printf("Position: 90 deg\n(center)");
                timer_waitMillis(2000);

                // Return to menu
                lcd_printf("Select part:\nB1: Basic Movement\nB2: Button Control\nB3: Calibration");
            }
            else if (button == 0x02) {
                // Part 2: Button Control
                lcd_printf("Part 2: Button\nControl\n\nStarting...");
                timer_waitMillis(1000);

                // Launch button control mode
                servo_button_control();

                // This won't execute due to infinite loop in servo_button_control
                lcd_printf("Select part:\nB1: Basic Movement\nB2: Button Control\nB3: Calibration");
            }
            else if (button == 0x04) {
                // Part 3: Calibration
                lcd_printf("Part 3: Servo\nCalibration\n\nStarting...");
                timer_waitMillis(1000);

                // Launch calibration mode
                servo_calibrate();

                // This won't execute due to infinite loop in servo_calibrate
                lcd_printf("Select part:\nB1: Basic Movement\nB2: Button Control\nB3: Calibration");
            }
        }
    }

    return 0;
}
