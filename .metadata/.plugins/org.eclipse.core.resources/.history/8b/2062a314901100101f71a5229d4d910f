///*
// * lab8_uart_trigger_csv.c
// *
// *  Created on: Apr 3, 2025
// *      Author: jjbaccam
// */
//
//#include "Timer.h"
//#include "lcd.h"
//#include "uart.h"
//#include "cyBot_Scan.h"
//#include "open_interface.h"
//#include "movement.h"
//#include "adc.h"
//#include "button.h"
//#include <stdio.h>
//#include <string.h>
//#include <math.h>
//
//#define LEFT_CAL 1177750
//#define RIGHT_CAL 264250
//
//// Define the size of the CSV buffer (adjust as needed)
//#define CSV_BUFFER_SIZE 2048
//
//// Helper function to send a string via UART.
//void send_uart_string(const char *str) {
//    uart_sendStr(str);
//}
//
//
//float calculate_distance(int irValue) {
//    if (irValue <= 0) {  // Prevent division by zero or negative values
//        return 0;
//    }
//    // Directly compute the distance from the ADC reading (y) using the derived equation:
//    float distance = pow(12453.9382f / (float)irValue, 1.0f / 0.7358f);
//    return distance;
//}
//
//
//int main(void) {
//    // Initialize hardware components.
//
//    timer_init();
//    lcd_init();
//    cyBOT_init_Scan(0b0111);
//    oi_t *sensor_data = oi_alloc();
//    oi_init(sensor_data);
//    adc_init();
//    uart_interrupt_init();
//
//    // Display initial instructions.
//    lcd_clear();
//    send_uart_string("IR Data Logging Mode\r\n");
//
//    // CSV buffer for storing samples.
//    cyBOT_Scan_t scan;
//    cyBOT_Scan(90, &scan);
//
//    // Main loop: wait for UART commands.
//    while (1) {
//        int numSamples = 16;  // Number of samples to average.
//        int sum = 0;
//        // Take 16 ADC samples
//        int i;
//        for (i = 0; i < numSamples; i++) {
//            sum += adc_read();
//        }
//        // Compute the averaged ADC value.
//        int avgAdc = sum / numSamples;
//        // Calculate the estimated distance in cm using our regression.
//        float distance = calculate_distance(avgAdc);
//
//        // Clear the LCD and display both values.
//        lcd_clear();
//        lcd_printf("ADC: %d\nDist: %.1f cm", avgAdc, distance);
//
//        // Delay before the next sample cycle.
//        timer_waitMillis(500);
//    }
//
//    return 0;
//}
