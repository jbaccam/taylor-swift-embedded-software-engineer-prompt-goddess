#include "cyBot_Scan.h"
#include "cyBot_uart.h"
#include "Timer.h"
#include "lcd.h"
#define LEFT_CAL 1167250
#define RIGHT_CAL 2432250

int main(void)
{
    timer_init();
    lcd_init();
    cyBOT_init_Scan(0b0111);
    cyBOT_SERVO_cal();
    return 0;
//
//    timer_init();
//    lcd_init();
//    cyBOT_init_Scan(0b0111);
//    right_calibration_value = RIGHT_CAL;
//    left_calibration_value = LEFT_CAL;
//
//    int i;
//    cyBOT_Scan_t scan;
//
//    for (i = 0; i<=180; i+=2){
//        cyBOT_Scan(i, &scan);
//    }
//    return 0;
}
