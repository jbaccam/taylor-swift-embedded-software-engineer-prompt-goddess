///**
// * turn_test.c
// *
// * A simple program to test turning accuracy of the CyBot
// * Uses existing movement.h functions for consistency
// *
// * Tests:
// * 1. Four 90-degree right turns (should make a complete 360)
// * 2. Four 90-degree left turns (should make a complete 360)
// * 3. Forward movement
// *
// * @author Your Name
// * @date March 24, 2025
// */
//
//#include "Timer.h"
//#include "lcd.h"
//#include "open_interface.h"
//#include "uart.h"      // Use your custom UART
//#include "movement.h"  // Using your existing movement functions
//
//// Servo calibration values - REPLACE WITH YOUR ACTUAL VALUES
//#define RIGHT_CAL 232750
//#define LEFT_CAL 1261750
//
//// Function to print messages to PuTTY
//void print_message(const char *message);
//
//int main(void) {
//    // Initialize hardware
//    timer_init();
//    lcd_init();
//    uart_interrupt_init();
//
//    // Initialize the CyBot
//    oi_t *sensor_data = oi_alloc();
//    oi_init(sensor_data);
//
//    // Set servo calibration values
//    int right_calibration_value = RIGHT_CAL;
//    int left_calibration_value = LEFT_CAL;
//
//    // Display welcome message
//    print_message("\033[2J\033[H");  // Clear screen
//    print_message("==== CyBot Turn Test Program ====\r\n\r\n");
//    print_message("Press any key to start the test sequence\r\n");
//
//    // Wait for user input
//    uart_receive();
//
//    // Test 1: Four 90-degree right turns
//    print_message("\r\nTest 1: Four 90-degree right turns\r\n");
//    print_message("Should complete a full 360-degree rotation\r\n");
//    int i;
//    for (i = 1; i <= 4; i++) {
//        char buffer[50];
//        sprintf(buffer, "Right turn #%d - turning 90 degrees right...\r\n", i);
//        print_message(buffer);
//
//        turn_right(sensor_data, 90);
//        timer_waitMillis(1000);  // Pause for 1 second
//    }
//
//    print_message("Right turn test complete. Press any key to continue...\r\n");
//    uart_receive();
//
//    // Test 2: Four 90-degree left turns
//    print_message("\r\nTest 2: Four 90-degree left turns\r\n");
//    print_message("Should complete a full 360-degree rotation\r\n");
//
//    for (i = 1; i <= 4; i++) {
//        char buffer[50];
//        sprintf(buffer, "Left turn #%d - turning 90 degrees left...\r\n", i);
//        print_message(buffer);
//
//        turn_left(sensor_data, 90);
//        timer_waitMillis(1000);  // Pause for 1 second
//    }
//
//    print_message("Left turn test complete. Press any key to continue...\r\n");
//    uart_receive();
//
//    // Test 3: Move forward
//    print_message("\r\nTest 3: Forward movement\r\n");
//    print_message("Moving forward 100mm...\r\n");
//
//    move_forward(sensor_data, 100);
//
//    print_message("Forward movement complete. Press any key to continue...\r\n");
//    uart_receive();
//
//    // Additional precision test: 45-degree turns
//    print_message("\r\nTest 4: Precision turns\r\n");
//    print_message("Turning right 45 degrees...\r\n");
//    turn_right(sensor_data, 45);
//    timer_waitMillis(1000);
//
//    print_message("Turning left 45 degrees (should return to starting orientation)...\r\n");
//    turn_left(sensor_data, 45);
//    timer_waitMillis(1000);
//
//    print_message("Turning left 180 degrees...\r\n");
//    turn_left(sensor_data, 180);
//    timer_waitMillis(1000);
//
//    print_message("Turning right 180 degrees (should return to starting orientation)...\r\n");
//    turn_right(sensor_data, 180);
//    timer_waitMillis(1000);
//
//    // Test complete
//    print_message("\r\nTurn test sequence complete!\r\n");
//    oi_free(sensor_data);
//
//    return 0;
//}
//
///**
// * Helper function to send a string to PuTTY
// */
//void print_message(const char *message) {
//    uart_sendStr(message);
//}
