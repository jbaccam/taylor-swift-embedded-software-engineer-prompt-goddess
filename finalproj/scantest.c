///*
// * simple_ir_scan.c - Basic IR scanning functionality for Mars Rover
// *
// * Performs a 180-degree scan in front of the robot, using only IR for simplicity
// * Avoids using ping_getDistance() since it's causing the program to hang
// *
// * Created: April 28, 2025
// * Authors: Jeremiah (Jay), Luke, Gavin, Judson, Benjamin
// */
//
//#include "Timer.h"
//#include "lcd.h"
//#include "uart.h"
//#include "servo.h"
//#include "adc.h"
//#include <math.h>
//#include <inc/tm4c123gh6pm.h>
//
//// Constants for scanning
//#define SCAN_STEP 5      // Scan every 5 degrees
//#define IR_THRESHOLD 400 // Threshold for IR object detection
//
//// Function prototypes
//void perform_scan(void);
//float calculate_ir_distance(int ir_value);
//
//int main(void) {
//    // Initialize hardware
//    timer_init();
//    lcd_init();
//    uart_init();
//    servo_init();
//    adc_init();
//
//    // Display welcome message
//    lcd_printf("Mars Rover\nIR Scanner");
//
//    char toPutty[100];
//    sprintf(toPutty, "=== Mars Rover IR Scanner ===\r\n");
//    uart_sendStr(toPutty);
//    sprintf(toPutty, "Press 'm' to start a 180-degree scan\r\n");
//    uart_sendStr(toPutty);
//
//    // Make sure servo is initialized by moving to center first
//    servo_move(90);
//    timer_waitMillis(1000);
//
//    sprintf(toPutty, "Servo initialized at center position.\r\n");
//    uart_sendStr(toPutty);
//
//    // Main loop
//    while (1) {
//        // Wait for command
//        char cmd = uart_receive();
//
//        if (cmd == 'm') {
//            // Start scan
//            perform_scan();
//
//            // Prompt for next scan
//            sprintf(toPutty, "\r\nPress 'm' to scan again\r\n");
//            uart_sendStr(toPutty);
//        }
//
//        // Small delay to prevent CPU hogging
//        timer_waitMillis(10);
//    }
//
//    return 0;
//}
//
///*
// * Calculate distance from IR sensor value
// * This is a simple conversion function - you may need to calibrate it
// */
//float calculate_ir_distance(int ir_value) {
//    if (ir_value <= 0) {  // Prevent division by zero
//        return 0;
//    }
//
//    // Simple conversion formula (you should calibrate this for your rover)
//    // This is a starting point based on typical IR sensor behavior
//    float distance = 10000.0 / (float)ir_value;
//
//    return distance;
//}
//
///*
// * Perform a 180-degree scan using only IR sensor
// */
//void perform_scan(void) {
//    int i;
//    int ir_value;
//    float ir_distance;
//    char toPutty[100];
//
//    // Announce scan start
//    sprintf(toPutty, "\r\nBeginning 180 Degree IR Scan...\r\n");
//    uart_sendStr(toPutty);
//
//    // Print table header
//    sprintf(toPutty, "Angle\tIR Distance (cm)\tIR Value\r\n");
//    uart_sendStr(toPutty);
//    sprintf(toPutty, "-----------------------------------\r\n");
//    uart_sendStr(toPutty);
//
//    // Start at center position
//    lcd_printf("Reset to Center");
//    servo_move(90);
//    timer_waitMillis(500);
//
//    // First move to 0 degrees as a separate step
//    lcd_printf("Moving to 0");
//    servo_move(0);
//    timer_waitMillis(1000); // Longer wait for first movement
//
//    // Scan from 0 to 180 degrees
//    for (i = 0; i <= 180; i += SCAN_STEP) {
//        // Update LCD with current angle
//        lcd_printf("Scanning\nAngle: %d", i);
//
//        // Move servo to position (if not already there)
//        if (i > 0) {
//            servo_move(i);
//            timer_waitMillis(200); // Wait for servo to reach position
//        }
//
//        // Take multiple IR readings and average them for stability
//        int sum = 0;
//        int readings = 5;
//        int j;
//
//        for (j = 0; j < readings; j++) {
//            sum += adc_read();
//            timer_waitMillis(10);
//        }
//
//        ir_value = sum / readings;
//
//        // Calculate approximate distance based on IR value
//        ir_distance = calculate_ir_distance(ir_value);
//
//        // Send data to PuTTY
//        sprintf(toPutty, "%d\t%.1f\t\t%d\r\n", i, ir_distance, ir_value);
//        uart_sendStr(toPutty);
//    }
//
//    // Return servo to center position
//    sprintf(toPutty, "\r\nScan complete, returning to center position...\r\n");
//    uart_sendStr(toPutty);
//
//    servo_move(90);
//    timer_waitMillis(1000);
//
//    // Scan complete message
//    sprintf(toPutty, "IR Scan complete!\r\n");
//    uart_sendStr(toPutty);
//
//    // Update LCD
//    lcd_printf("Scan Complete\nPress 'm' again");
//}
