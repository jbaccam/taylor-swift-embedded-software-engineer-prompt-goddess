///**
// * lab6_template.c
// *
// * Template file for CprE 288 Lab 6
// *
// * @author Diane Rover, 2/15/2020
// *
// */
//
//#include "Timer.h"
//#include "lcd.h"
//#include "cyBot_Scan.h"  // For scan sensors
//#include "cyBot_uart.h"  // For scan sensors
//
//#include "uart.h"
//
//// Uncomment or add any include directives that are needed
//// #include "open_interface.h"
//// #include "movement.h"
//// #include "button.h"
//#define LEFT_CAL 1335250
//#define RIGHT_CAL 343000
//
//#define REPLACEME 0
//
//int main(void)
//{
//
//    timer_init(); // Must be called before lcd_init(),      which uses timer functions
//    lcd_init();
////	uart_init();
////    cyBot_uart_init();
//    cyBOT_Scan_t scan;
//    cyBOT_init_Scan(0b0111);
//    cyBOT_Scan(0, &scan);
//
//    // YOUR CODE HERE
//
//    while (1)
//    {
//        char m = uart_receive();
//
//        if (m == 'g')
//        {
//            right_calibration_value = RIGHT_CAL;
//            left_calibration_value = LEFT_CAL;
//            cyBOT_Scan_t scan;
//            int i;
//
//            for (i = 0; i < 180; i += 2)
//            {
//                char character_receieved = uart_receive_nonblocking();
//                if (character_receieved == 's') {
//                    break;
//                } else {
//                    cyBOT_Scan(i, &scan);
//                }
//            }
//        }
//
//    }
//    return 0;
//}
//
