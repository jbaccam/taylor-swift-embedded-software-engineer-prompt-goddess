/*
 * finalproj.c
 *
 *  Created on: Apr 25, 2025
 *      Author: jjbaccam
 */
#include "adc.h"
#include "timer.h"
#include "lcd.h"
#include "uart.h"
#include <open_interface.h>
#include <movement.h>
#include "ping.h"
#include "servo.h"
#include "button.h"

void main(void)
{
    timer_init();
    lcd_init();
    adc_init();
    ping_init();
    servo_init();
    button_init();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);
    uart_interrupt_init();

    char input;
    char inputPutty[50];
    int IRmeasurement;
    float pingDistance;

    while(1) {
        inputCharacter = uart_receive();
        if (inputCharacter == '\r') {
            break;
        }
    }

    while(1) {

    }

}



