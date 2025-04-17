/*
 * servo.c
 *
 *  Created on: Apr 17, 2025
 *      Author: jjbaccam
 */
#include "servo.h"
#include "button.h"
#include "lcd.h"

extern volatile int button_num;
int zeroAngle = 9087;
int maxAngle = 36946;

void servo_init(void)
{
    SYSCTL_RCGCGPIO_R |= 0b000010;
    GPIO_PORTB_DIR_R |= 0b00100000;
    GPIO_PORTB_DEN_R |= 0b00100000;
    GPIO_PORTB_AFSEL_R |= 0b00100000;
    GPIO_PORTB_PCTL_R &= 0xFF0FFFFF;
    GPIO_PORTB_PCTL_R |= 0x00700000;
    SYSCTL_RCGCTIMER_R |= 0b000010;

    TIMER1_CTL_R &= 0b111111011111111;
    TIMER1_CFG_R |= 0b100;
    TIMER1_TBMR_R |= 0b000000001010;
    TIMER1_TBMR_R &= 0b111111101010;
    TIMER1_CTL_R |= ~0b011111111111111;
    TIMER1_TBILR_R = 0x0000E200;
    TIMER1_TBPR_R = 0x00000004;
    TIMER1_TBMATCHR_R = zeroAngle;  //Degrees*155+9087
    TIMER1_TBPMR_R = 0x00000000;
    TIMER1_CTL_R |= 0b0000000100000000;
}

void servo_move(float degrees)
{
    timer_waitMillis(300);
    TIMER1_TBMATCHR_R = degrees*155+9087;
}

void calibrate_servo(void)
{
    int positionValue;
    int button_flag = 0;

    int slopeAngle = (maxAngle - zeroAngle)/180;
    int halfAngle = slopeAngle*90 + zeroAngle;

    TIMER1_TBMATCHR_R = halfAngle;
    while (1) {

        positionValue = TIMER1_TBMATCHR_R;
        if (button_flag == 0) {
            lcd_printf("Position: %d\nCW\n\nAngle: %d", positionValue, (positionValue - zeroAngle)/slopeAngle);
            if (button_num == 1 && positionValue > zeroAngle) {
                TIMER1_TBMATCHR_R -= slopeAngle;
                timer_waitMillis(250);
            }
            if (button_num == 2 && positionValue > zeroAngle + slopeAngle*4) {
                TIMER1_TBMATCHR_R -= slopeAngle*5;
                timer_waitMillis(250);
            }
            if (button_num == 2 && positionValue < zeroAngle + slopeAngle*4 && positionValue > zeroAngle) {
                TIMER1_TBMATCHR_R = zeroAngle;
                timer_waitMillis(250);
            }
            if (button_num == 3) {
                button_flag = 1;
                timer_waitMillis(250);
            }
            if (button_num == 4) {
                TIMER1_TBMATCHR_R = zeroAngle;
            }
        }
        if (button_flag == 1) {
            lcd_printf("Position: %d\nCCW\n\nAngle: %d", positionValue, (positionValue - zeroAngle)/slopeAngle);
            if (button_num == 1 && positionValue < maxAngle) {
                TIMER1_TBMATCHR_R += slopeAngle;
                timer_waitMillis(250);
            }
            if (button_num == 2 && positionValue < maxAngle - slopeAngle*4) {
                TIMER1_TBMATCHR_R += slopeAngle*5;
                timer_waitMillis(250);
            }
            if (button_num == 2 && positionValue > maxAngle - slopeAngle*4 && positionValue < maxAngle) {
                TIMER1_TBMATCHR_R = maxAngle;
                timer_waitMillis(250);
            }
            if (button_num == 3) {
                button_flag = 0;
                timer_waitMillis(250);
            }
            if (button_num == 4) {
                TIMER1_TBMATCHR_R = maxAngle;
            }
        }
    }
}






