///*
// * cliff_sensor_test.c
// *
// * This program continuously reads and displays cliff sensor values
// * to help determine IR values for different colors
// */
//
//#include "Timer.h"
//#include "lcd.h"
//#include "uart.h"
//#include "open_interface.h"
//
//int main(void)
//{
//    // Initialize peripherals
//    timer_init();
//    lcd_init();
//    uart_init();
//
//    // Initialize robot
//    oi_t *sensor_data = oi_alloc();
//    oi_init(sensor_data);
//
//    char buffer[64];
//
//    uart_sendStr("\r\n*** Cliff Sensor Color Detection Test ***\r\n");
//    uart_sendStr("Place the robot over different surfaces to test IR values\r\n");
//    uart_sendStr("Press any key to stop\r\n\n");
//
//    // Clear the UART receive buffer
//    while ((UART1_FR_R & 0x10) == 0) {
//        UART1_DR_R;
//    }
//
//    // Main loop to continuously read sensors
//    while (1)
//    {
//        // Update sensor data
//        oi_update(sensor_data);
//
//        // Display cliff sensor values on LCD
//        lcd_printf("L:%d FL:%d\nFR:%d R:%d",
//                 sensor_data->cliffLeftSignal,
//                 sensor_data->cliffFrontLeftSignal,
//                 sensor_data->cliffFrontRightSignal,
//                 sensor_data->cliffRightSignal);
//
//        // Display cliff sensor values on UART
//        sprintf(buffer, "Left: %4d | Front Left: %4d | Front Right: %4d | Right: %4d\r",
//                sensor_data->cliffLeftSignal,
//                sensor_data->cliffFrontLeftSignal,
//                sensor_data->cliffFrontRightSignal,
//                sensor_data->cliffRightSignal);
//        uart_sendStr(buffer);
//
//        // Brief delay to make values readable
//        timer_waitMillis(200);
//
//        // Check if any key was pressed to exit
//        if ((UART1_FR_R & 0x10) == 0) {
//            break; // Exit if key pressed
//        }
//    }
//
//    // Clean up
//    oi_free(sensor_data);
//
//    uart_sendStr("\r\n\nTest complete. Cliff sensor reading ended.\r\n");
//
//    return 0;
//}
