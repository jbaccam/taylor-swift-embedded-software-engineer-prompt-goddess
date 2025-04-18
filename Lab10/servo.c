/*
 * servo.c
 *
 *  Created on: Apr 17, 2025
 *      Author: jjbaccam
 */
#include "servo.h"
#include "button.h"
#include "lcd.h"

// Calibration values for the servo
// Default values - these will be overwritten by calibration
int right_calibration_value = 8000;
int left_calibration_value = 34250;

void servo_init(void)
{
    // Enable clock to Port B
    SYSCTL_RCGCGPIO_R |= 0b000010;

    // Configure PB5 for alternate function (Timer1B)
    GPIO_PORTB_DIR_R |= 0b00100000;    // Set PB5 as output
    GPIO_PORTB_DEN_R |= 0b00100000;    // Digital enable PB5
    GPIO_PORTB_AFSEL_R |= 0b00100000;  // Enable alternate function

    // Configure Port Control Register for Timer1B (T1CCP1 = 7)
    GPIO_PORTB_PCTL_R &= 0xFF0FFFFF;   // Clear bits for PB5
    GPIO_PORTB_PCTL_R |= 0x00700000;   // Set bits for Timer1B

    // Enable clock to Timer 1
    SYSCTL_RCGCTIMER_R |= 0b000010;

    // Disable Timer 1B during setup
    TIMER1_CTL_R &= 0b111111011111111;

    // Configure Timer 1B in 16-bit mode with prescaler
    TIMER1_CFG_R |= 0b100;

    // Configure Timer 1B for PWM mode (periodic, PWM, count down)
    TIMER1_TBMR_R |= 0b000000001010;
    TIMER1_TBMR_R &= 0b111111101010;

    // Enable Timer 1B features
    TIMER1_CTL_R |= ~0b011111111111111;

    // Set load value for 20ms period (50Hz)
    TIMER1_TBILR_R = 0x0000E200;
    TIMER1_TBPR_R = 0x00000004;

    // Initial position at center (90 degrees)
    servo_move(90);

    // Enable Timer 1B
    TIMER1_CTL_R |= 0b0000000100000000;
}

void servo_move(float degrees)
{
    // Add delay to allow servo to reach position
    timer_waitMillis(300);

    // Calculate match value based on degrees using calibration values
    int matchValue = right_calibration_value +
                     (int)((left_calibration_value - right_calibration_value) * degrees / 180.0);

    // Set the match value
    TIMER1_TBMATCHR_R = matchValue;
}

void servo_button_control(void)
{
    // Button configuration
    button_init();

    // Variables to track servo position and direction
    int positionValue;
    int button_flag = 1;  // 0 = clockwise, 1 = counterclockwise (starting direction is CCW)
    uint8_t button = 0;
    uint8_t last_direction_button = 0;  // Track Button 3 presses separately

    // Calculate slope for degree calculations
    int slopeAngle = (left_calibration_value - right_calibration_value)/180;

    // Move to center position (90 degrees)
    servo_move(90);

    while (1) {
        // Get current position value
        positionValue = TIMER1_TBMATCHR_R;

        // Get button press
        button = button_getButton();

        // Special handling for direction toggle (Button 3)
        if (button == 0x04) {
            if (last_direction_button == 0) {
                // Toggle direction
                button_flag = !button_flag;
                last_direction_button = 0x04;
                timer_waitMillis(200);  // Debounce for direction change
            }
        } else {
            last_direction_button = 0;  // Reset when button is released
        }

        // Handle clockwise direction (decreasing angle)
        if (button_flag == 0) {
            // Display current position and direction
            lcd_printf("Position: %d\nCW\n\nAngle: %d",
                      positionValue,
                      (positionValue - right_calibration_value)/slopeAngle);

            // Button 1: Move 1 degree clockwise
            if (button == 0x01 && positionValue > right_calibration_value) {
                TIMER1_TBMATCHR_R -= slopeAngle;
                timer_waitMillis(100);  // Shorter delay for continuous movement
            }

            // Button 2: Move 5 degrees clockwise
            if (button == 0x02 && positionValue > right_calibration_value + slopeAngle*4) {
                TIMER1_TBMATCHR_R -= slopeAngle*5;
                timer_waitMillis(100);  // Shorter delay for continuous movement
            }

            // Button 2: Move to 0 degrees if less than 5 degrees away
            if (button == 0x02 && positionValue < right_calibration_value + slopeAngle*4 &&
                positionValue > right_calibration_value) {
                TIMER1_TBMATCHR_R = right_calibration_value;
                timer_waitMillis(200);
            }

            // Button 4: Move to 5 degrees
            if (button == 0x08) {
                TIMER1_TBMATCHR_R = right_calibration_value + slopeAngle*5;
                timer_waitMillis(200);
            }
        }

        // Handle counterclockwise direction (increasing angle)
        if (button_flag == 1) {
            // Display current position and direction
            lcd_printf("Position: %d\nCCW\n\nAngle: %d",
                      positionValue,
                      (positionValue - right_calibration_value)/slopeAngle);

            // Button 1: Move 1 degree counterclockwise
            if (button == 0x01 && positionValue < left_calibration_value) {
                TIMER1_TBMATCHR_R += slopeAngle;
                timer_waitMillis(100);  // Shorter delay for continuous movement
            }

            // Button 2: Move 5 degrees counterclockwise
            if (button == 0x02 && positionValue < left_calibration_value - slopeAngle*4) {
                TIMER1_TBMATCHR_R += slopeAngle*5;
                timer_waitMillis(100);  // Shorter delay for continuous movement
            }

            // Button 2: Move to 180 degrees if less than 5 degrees away
            if (button == 0x02 && positionValue > left_calibration_value - slopeAngle*4 &&
                positionValue < left_calibration_value) {
                TIMER1_TBMATCHR_R = left_calibration_value;
                timer_waitMillis(200);
            }

            // Button 4: Move to 175 degrees
            if (button == 0x08) {
                TIMER1_TBMATCHR_R = left_calibration_value - slopeAngle*5;
                timer_waitMillis(200);
            }
        }
    }
}

void servo_calibrate(void)
{
    // Initialize variables for calibration
    uint8_t button;
    uint8_t last_button = 0;

    // Initialize buttons
    button_init();

    // Start with center position
    TIMER1_TBMATCHR_R = 22000; // Approximate center value

    // First calibrate 0 degrees (right)
    lcd_printf("Calibrating 0 deg\nB1: Move left\nB2: Move right\nB4: Confirm");

    while (1) {
        button = button_getButton();

        if (button == 0x01) {
            // Move left (increase match value)
            TIMER1_TBMATCHR_R += 250;
            timer_waitMillis(50);  // Short delay for continuous movement

            // Update display less frequently during continuous movement
            if (last_button != button) {
                lcd_printf("Calibrating 0 deg\nValue: %d\nB1: Left B2: Right\nB4: Confirm", TIMER1_TBMATCHR_R);
                last_button = button;
            }
        }
        else if (button == 0x02) {
            // Move right (decrease match value)
            TIMER1_TBMATCHR_R -= 250;
            timer_waitMillis(50);  // Short delay for continuous movement

            // Update display less frequently during continuous movement
            if (last_button != button) {
                lcd_printf("Calibrating 0 deg\nValue: %d\nB1: Left B2: Right\nB4: Confirm", TIMER1_TBMATCHR_R);
                last_button = button;
            }
        }
        else if (button == 0x08) {
            // Confirm 0 degree position
            right_calibration_value = TIMER1_TBMATCHR_R;
            timer_waitMillis(200); // Debounce
            break;
        }
        else if (button == 0) {
            // If no button is pressed but we were pressing one before,
            // update the display once more
            if (last_button != 0) {
                lcd_printf("Calibrating 0 deg\nValue: %d\nB1: Left B2: Right\nB4: Confirm", TIMER1_TBMATCHR_R);
                last_button = 0;
            }
        }
    }

    // Show the 0 degree calibration value
    lcd_printf("0 deg calibration\nvalue: %d\n\nPress any button", right_calibration_value);

    // Wait for button press and release
    while (button_getButton() != 0) {}  // Wait for release
    while (button_getButton() == 0) {}  // Wait for press
    while (button_getButton() != 0) {}  // Wait for release again
    timer_waitMillis(200); // Extra debounce

    // Now calibrate 180 degrees (left)
    last_button = 0;
    TIMER1_TBMATCHR_R = 22000; // Reset to approximate center
    lcd_printf("Calibrating 180 deg\nB1: Move left\nB2: Move right\nB4: Confirm");

    while (1) {
        button = button_getButton();

        if (button == 0x01) {
            // Move left (increase match value)
            TIMER1_TBMATCHR_R += 250;
            timer_waitMillis(50);  // Short delay for continuous movement

            // Update display less frequently during continuous movement
            if (last_button != button) {
                lcd_printf("Calibrating 180 deg\nValue: %d\nB1: Left B2: Right\nB4: Confirm", TIMER1_TBMATCHR_R);
                last_button = button;
            }
        }
        else if (button == 0x02) {
            // Move right (decrease match value)
            TIMER1_TBMATCHR_R -= 250;
            timer_waitMillis(50);  // Short delay for continuous movement

            // Update display less frequently during continuous movement
            if (last_button != button) {
                lcd_printf("Calibrating 180 deg\nValue: %d\nB1: Left B2: Right\nB4: Confirm", TIMER1_TBMATCHR_R);
                last_button = button;
            }
        }
        else if (button == 0x08) {
            // Confirm 180 degree position
            left_calibration_value = TIMER1_TBMATCHR_R;
            timer_waitMillis(200); // Debounce
            break;
        }
        else if (button == 0) {
            // If no button is pressed but we were pressing one before,
            // update the display once more
            if (last_button != 0) {
                lcd_printf("Calibrating 180 deg\nValue: %d\nB1: Left B2: Right\nB4: Confirm", TIMER1_TBMATCHR_R);
                last_button = 0;
            }
        }
    }

    // Show both calibration values
    lcd_printf("Calibration values\n0 deg: %d\n180 deg: %d",
              right_calibration_value, left_calibration_value);

    // Wait for button press and release
    while (button_getButton() != 0) {}  // Wait for release
    while (button_getButton() == 0) {}  // Wait for press
    timer_waitMillis(200); // Debounce

    // Return to center position with calibration applied
    servo_move(90);

    // Show completion message
    lcd_printf("Calibration done\nReturning to\nbutton control");
    timer_waitMillis(2000);

    // Enter button control mode with calibrated values
    servo_button_control();
}
