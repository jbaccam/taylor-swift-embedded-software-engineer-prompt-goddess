/*
 * lab10.c
 *
 *  Created on: Apr 17, 2025
 *      Author: jjbaccam
 */
#include "Timer.h"
#include "lcd.h"
#include "ping.h"
#include "servo.h"
#include "open_interface.h"  // For Open Interface functions


int main(void) {
    // Initialize hardware components
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    ping_init();
    servo_init();

    // Initialize Open Interface
    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);

    servo_move(180);

    oi_free(sensor_data);
    return 0;
}




