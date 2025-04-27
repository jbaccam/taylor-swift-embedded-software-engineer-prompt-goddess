/*
 * mars_rover.c - Team SE2 "Ping Patrol" Mars Rover Implementation
 * 
 * Manual navigation control with:
 * - Obstacle detection and avoidance
 * - Water sample detection with spin reaction
 * - Boundary detection and adherence
 * - Object scanning and reporting
 * - Current orientation tracking
 * 
 * Created: April 27, 2025
 * Authors: Jeremiah (Jay), Luke, Gavin, Judson, Benjamin
 */

 #include "Timer.h"
 #include "lcd.h"
 #include "uart.h"
 #include "open_interface.h"
 #include "ping.h"
 #include "servo.h"
 #include "adc.h"
 #include "movement.h"
 #include <math.h>
 #include <stdbool.h>
 #include <string.h>
 
 // constants for detection
 #define BLUE_THRESHOLD 2800    // threshold for blue paper detection (needs calibration)
 #define IR_THRESHOLD 400       // threshold for IR object detection
 #define SCAN_STEP 2            // scan every 2 degrees
 
 // function prototypes
 void initialize_hardware(void);
 void manual_mode(oi_t *sensor_data);
 bool check_for_water_sample(oi_t *sensor_data);
 void spin_for_water_sample(oi_t *sensor_data);
 bool check_for_cliff(oi_t *sensor_data);
 void update_orientation_display(float current_angle);
 
 // global variables
 float current_orientation = 90.0; // initial orientation (90 = forward)
 
 int main(void) {
     // initialize hardware components
     initialize_hardware();
     
     // initialize the cybot
     oi_t *sensor_data = oi_alloc();
     oi_init(sensor_data);
     
     // display welcome message
     lcd_printf("Mars Rover\nInitialized");
     
     char toPutty[100];
     sprintf(toPutty, "=== Mars Rover - Team Ping Patrol ===\r\n");
     uart_sendStr(toPutty);
     sprintf(toPutty, "Manual control mode active\r\n");
     uart_sendStr(toPutty);
     sprintf(toPutty, "Current orientation: %.1f degrees\r\n", current_orientation);
     uart_sendStr(toPutty);
     sprintf(toPutty, "Controls:\r\n");
     uart_sendStr(toPutty);
     sprintf(toPutty, "  w - forward 5cm\r\n  s - backward 5cm\r\n  a - turn CCW 10°\r\n  d - turn CW 10°\r\n");
     uart_sendStr(toPutty);
     sprintf(toPutty, "  m - 180° scan\r\n  n - 360° scan\r\n");
     uart_sendStr(toPutty);
     
     // start in manual mode
     manual_mode(sensor_data);
     
     // clean up
     oi_free(sensor_data);
     return 0;
 }
 
 // initialize all hardware components
 void initialize_hardware(void) {
     timer_init();
     lcd_init();
     uart_init();
     adc_init();
     ping_init();
     servo_init();
 }
 
 // display current orientation in putty
 void update_orientation_display(float current_angle) {
     char toPutty[100];
     sprintf(toPutty, "Current orientation: %.1f degrees\r\n", current_angle);
     uart_sendStr(toPutty);
 }
 
 // manual control mode with base station commands
 void manual_mode(oi_t *sensor_data) {
     int flag = 1;  // simple integer flag, 1 = true
     int i, j, k;
     float pingDistance;
     int IRmeasurement;
     char inputCharacter;
     char toPutty[100];
     
     lcd_printf("Manual Mode");
     
     while (flag == 1) {
         i = 0;
         j = 0;
         k = 0;
         
         // check for emergency conditions
         oi_update(sensor_data);
         
         // Check for bumps - halt immediately if detected
         if (sensor_data->bumpLeft) {
             // Emergency stop
             oi_setWheels(0, 0);
             sprintf(toPutty, "\r\n*** LEFT BUMPER TRIGGERED! EMERGENCY STOP ***\r\n");
             uart_sendStr(toPutty);
             
             // Back up slightly for safety
             move_backwards(sensor_data, 5);
             
             // Update status
             sprintf(toPutty, "Backed up 5cm for safety. Please choose a new direction.\r\n");
             uart_sendStr(toPutty);
             update_orientation_display(current_orientation);
             continue;
         }
         
         if (sensor_data->bumpRight) {
             // Emergency stop
             oi_setWheels(0, 0);
             sprintf(toPutty, "\r\n*** RIGHT BUMPER TRIGGERED! EMERGENCY STOP ***\r\n");
             uart_sendStr(toPutty);
             
             // Back up slightly for safety
             move_backwards(sensor_data, 5);
             
             // Update status
             sprintf(toPutty, "Backed up 5cm for safety. Please choose a new direction.\r\n");
             uart_sendStr(toPutty);
             update_orientation_display(current_orientation);
             continue;
         }
         
         // Check for cliffs
         if (check_for_cliff(sensor_data)) {
             // Emergency stop
             oi_setWheels(0, 0);
             sprintf(toPutty, "\r\n*** CLIFF DETECTED! EMERGENCY STOP ***\r\n");
             uart_sendStr(toPutty);
             move_backwards(sensor_data, 10); // back up 10cm for safety
             
             // Update status
             sprintf(toPutty, "Backed up 10cm for safety. Please choose a new direction.\r\n");
             uart_sendStr(toPutty);
             update_orientation_display(current_orientation);
             continue;
         }
         
         // Check for water samples (blue paper)
         if (check_for_water_sample(sensor_data)) {
             // Stop and collect sample
             oi_setWheels(0, 0);
             sprintf(toPutty, "\r\n*** WATER SAMPLE DETECTED! Collecting... ***\r\n");
             uart_sendStr(toPutty);
             spin_for_water_sample(sensor_data);
             update_orientation_display(current_orientation);
         }
 
         // Check for manual commands using non-blocking UART
         inputCharacter = uart_receive_nonblocking();
         if (inputCharacter != 255) { // 255 means no character received
             uart_sendChar(inputCharacter);
             
             if (inputCharacter == 'w') {
                 // Move forward 5cm
                 sprintf(toPutty, "\r\nMoving Forward 5cm.\r\n");
                 uart_sendStr(toPutty);
                 move_forward(sensor_data, 5);
                 sprintf(toPutty, "Done Moving Forward.\r\n");
                 uart_sendStr(toPutty);
                 update_orientation_display(current_orientation);
             }
             else if (inputCharacter == 's') {
                 // Move backward 5cm
                 sprintf(toPutty, "\r\nMoving Backwards 5cm.\r\n");
                 uart_sendStr(toPutty);
                 move_backwards(sensor_data, 5);
                 sprintf(toPutty, "Done Moving Backwards.\r\n");
                 uart_sendStr(toPutty);
                 update_orientation_display(current_orientation);
             }
             else if (inputCharacter == 'd') {
                 // Turn clockwise 10 degrees
                 sprintf(toPutty, "\r\nTurning Clockwise 10 Degrees.\r\n");
                 uart_sendStr(toPutty);
                 turn_clockwise(sensor_data, 10);
                 
                 // Update current orientation
                 current_orientation -= 10;
                 if (current_orientation < 0) {
                     current_orientation += 360;
                 }
                 
                 sprintf(toPutty, "Done Turning Clockwise.\r\n");
                 uart_sendStr(toPutty);
                 update_orientation_display(current_orientation);
             }
             else if (inputCharacter == 'a') {
                 // Turn counterclockwise 10 degrees
                 sprintf(toPutty, "\r\nTurning Counterclockwise 10 Degrees.\r\n");
                 uart_sendStr(toPutty);
                 turn_counterclockwise(sensor_data, 10);
                 
                 // Update current orientation
                 current_orientation += 10;
                 if (current_orientation >= 360) {
                     current_orientation -= 360;
                 }
                 
                 sprintf(toPutty, "Done Turning Counterclockwise.\r\n");
                 uart_sendStr(toPutty);
                 update_orientation_display(current_orientation);
             }
             else if (inputCharacter == 'm') {
                 // Perform 180-degree scan
                 oi_setWheels(0, 0);
                 
                 sprintf(toPutty, "\r\nBeginning 180 Degree Scan.\r\n");
                 uart_sendStr(toPutty);
                 
                 int sensorAngle[80] = {0};        // angles where objects are detected
                 float sensorDistance[80] = {0};   // distances to objects
                 
                 sprintf(toPutty, "Degrees\tDistance (cm)\n\r");
                 uart_sendStr(toPutty);
                 
                 // Perform the scan
                 for (i = 0; i <= 180; i = i + SCAN_STEP) {
                     servo_move(i);
                     pingDistance = ping_read();
                     IRmeasurement = adc_read();
                     
                     if (IRmeasurement > IR_THRESHOLD) {
                         sprintf(toPutty, "%d\t%f\n\r", i, pingDistance);
                         uart_sendStr(toPutty);
                         
                         sensorAngle[k] = i;
                         sensorDistance[k] = pingDistance;
                         k++;
                     }
                     j = 0;
                 }
                 
                 // Analyze detected objects (same as your paste code)
                 int avgAngle[10] = {0};     // average angle of objects
                 float avgDist[10] = {0};    // average distance to objects
                 int width[10] = {0};        // angular width of objects
                 int l = 0;                  // counter for objects
                 float StartDist[10] = {0};  // distance at start angle
                 float EndDist[10] = {0};    // distance at end angle
                 
                 // Process object data - this is from your existing code
                 // It finds distinct objects and calculates their properties
                 for (i = 1; i <= k; i++) {
                     if (StartDist[l] == 0) {
                         StartDist[l] = sensorDistance[i];
                     }
                     if (sensorAngle[i] - sensorAngle[i-1] <= 4 && i != k) {
                         if (sensorAngle[i] - sensorAngle[i-1] == 4) {
                             j = j + 4;
                             avgAngle[l] = avgAngle[l] + 4*sensorAngle[i];
                             avgDist[l] = avgDist[l] + 4*sensorDistance[i];
                             EndDist[l] = sensorDistance[i];
                         }
                         else {
                             j = j + 2;
                             avgAngle[l] = avgAngle[l] + 2*sensorAngle[i];
                             avgDist[l] = avgDist[l] + 2*sensorDistance[i];
                             EndDist[l] = sensorDistance[i];
                         }
                     }
                     else {
                         if (j <= 2) {
                             j = 0;
                             avgAngle[l] = 0;
                             avgDist[l] = 0;
                             StartDist[l] = 0;
                         }
                         else {
                             width[l] = j;
                             avgAngle[l] = avgAngle[l] / j;
                             avgDist[l] = avgDist[l] / j;
                             j = 0;
                             l++;
                         }
                     }
                 }
                 
                 // Calculate linear width using Law of Cosines
                 float LinearWidth[10] = {0};
                 for (i = 0; i < l; i++) {
                     LinearWidth[i] = sqrt(pow(StartDist[i], 2) + pow(EndDist[i], 2) - 
                                       2*StartDist[i]*EndDist[i]*cos((width[i])*(M_PI / 180)));
                 }
                 
                 // Display results
                 sprintf(toPutty, "\n\r*** OBJECT DETECTION RESULTS ***\r\n");
                 uart_sendStr(toPutty);
                 sprintf(toPutty, "Objects found: %d\r\n", l);
                 uart_sendStr(toPutty);
                 sprintf(toPutty, "Object\tRel Angle\tAbs Angle\tDistance\tWidth\n\r");
                 uart_sendStr(toPutty);
                 
                 for (i = 0; i < l; i++) {
                     // Calculate absolute angle in the environment
                     float absoluteAngle = fmod(current_orientation - 90 + avgAngle[i], 360);
                     if (absoluteAngle < 0) absoluteAngle += 360;
                     
                     sprintf(toPutty, "%d\t%d°\t\t%.1f°\t\t%.1f cm\t%.1f cm\n\r", 
                             i + 1, avgAngle[i], absoluteAngle, avgDist[i], LinearWidth[i]);
                     uart_sendStr(toPutty);
                 }
                 
                 // Return servo to center position
                 servo_move(90);
                 
                 // Final status update
                 sprintf(toPutty, "\r\nScan complete. Navigate based on objects shown above.\r\n");
                 uart_sendStr(toPutty);
                 update_orientation_display(current_orientation);
             }
             else if (inputCharacter == 'n') {
                 // This is same as your original n command from your paste
                 oi_setWheels(0, 0);
                 
                 sprintf(toPutty, "\r\nBeginning 360 Degree Scan.\r\n");
                 uart_sendStr(toPutty);
                 
                 int sensorAngle[80] = {0};
                 float sensorDistance[80] = {0};
                 
                 sprintf(toPutty, "Degrees\tDistance (cm)\n\r");
                 uart_sendStr(toPutty);
                 
                 // Scan 0-180 degrees
                 for (i = 0; i <= 180; i = i + SCAN_STEP) {
                     servo_move(i);
                     pingDistance = ping_read();
                     IRmeasurement = adc_read();
                     
                     if (IRmeasurement > IR_THRESHOLD) {
                         sprintf(toPutty, "%d\t%f\n\r", i, pingDistance);
                         uart_sendStr(toPutty);
                         
                         sensorAngle[k] = i;
                         sensorDistance[k] = pingDistance;
                         k++;
                     }
                     j = 0;
                 }
                 
                 // Turn robot 180 degrees
                 autoturn_clockwise(sensor_data, 180);
                 
                 // Update orientation after turning
                 current_orientation -= 180;
                 if (current_orientation < 0) {
                     current_orientation += 360;
                 }
                 
                 // Scan second 180 degrees
                 for (i = 2; i <= 180; i = i + SCAN_STEP) {
                     servo_move(i);
                     pingDistance = ping_read();
                     IRmeasurement = adc_read();
                     
                     if (IRmeasurement > IR_THRESHOLD) {
                         sprintf(toPutty, "%d\t%f\n\r", i + 180, pingDistance);
                         uart_sendStr(toPutty);
                         
                         sensorAngle[k] = i + 180;
                         sensorDistance[k] = pingDistance;
                         k++;
                     }
                     j = 0;
                 }
                 
                 // Object analysis - identical to your paste code
                 int avgAngle[10] = {0};
                 float avgDist[10] = {0};
                 int width[10] = {0};
                 int l = 0;
                 float StartDist[10] = {0};
                 float EndDist[10] = {0};
                 
                 for (i = 1; i <= k; i++) {
                     if (StartDist[l] == 0) {
                         StartDist[l] = sensorDistance[i];
                     }
                     if (sensorAngle[i] - sensorAngle[i-1] <= 4 && i != k) {
                         if (sensorAngle[i] - sensorAngle[i-1] == 4) {
                             j = j + 4;
                             avgAngle[l] = avgAngle[l] + 4*sensorAngle[i];
                             avgDist[l] = avgDist[l] + 4*sensorDistance[i];
                             EndDist[l] = sensorDistance[i];
                         }
                         else {
                             j = j + 2;
                             avgAngle[l] = avgAngle[l] + 2*sensorAngle[i];
                             avgDist[l] = avgDist[l] + 2*sensorDistance[i];
                             EndDist[l] = sensorDistance[i];
                         }
                     }
                     else {
                         if (j <= 2) {
                             j = 0;
                             avgAngle[l] = 0;
                             avgDist[l] = 0;
                             StartDist[l] = 0;
                         }
                         else {
                             width[l] = j;
                             avgAngle[l] = avgAngle[l] / j;
                             avgDist[l] = avgDist[l] / j;
                             j = 0;
                             l++;
                         }
                     }
                 }
                 
                 float LinearWidth[10] = {0};
                 for (i = 0; i < l; i++) {
                     LinearWidth[i] = sqrt(pow(StartDist[i], 2) + pow(EndDist[i], 2) - 
                                       2*StartDist[i]*EndDist[i]*cos((width[i])*(M_PI / 180)));
                 }
                 
                 // Display results with absolute angles based on current orientation
                 sprintf(toPutty, "\n\r*** OBJECT DETECTION RESULTS (360° SCAN) ***\r\n");
                 uart_sendStr(toPutty);
                 sprintf(toPutty, "Objects found: %d\r\n", l);
                 uart_sendStr(toPutty);
                 sprintf(toPutty, "Object\tRel Angle\tAbs Angle\tDistance\tWidth\n\r");
                 uart_sendStr(toPutty);
                 
                 for (i = 0; i < l; i++) {
                     // Calculate absolute angle in the environment
                     float absoluteAngle = fmod(current_orientation - 90 + avgAngle[i], 360);
                     if (absoluteAngle < 0) absoluteAngle += 360;
                     
                     sprintf(toPutty, "%d\t%d°\t\t%.1f°\t\t%.1f cm\t%.1f cm\n\r", 
                             i + 1, avgAngle[i], absoluteAngle, avgDist[i], LinearWidth[i]);
                     uart_sendStr(toPutty);
                 }
                 
                 // Return robot to original orientation
                 autoturn_clockwise(sensor_data, 180);
                 
                 // Update orientation after turning back
                 current_orientation -= 180;
                 if (current_orientation < 0) {
                     current_orientation += 360;
                 }
                 
                 // Center servo
                 servo_move(90);
                 
                 // Final status update
                 sprintf(toPutty, "\r\n360° Scan complete. Navigate based on objects shown above.\r\n");
                 uart_sendStr(toPutty);
                 update_orientation_display(current_orientation);
             }
         }
         
         // Small delay to prevent excessive CPU usage
         timer_waitMillis(20);
     }
 }
 
 // check for water samples (blue paper) using cliff sensors
 bool check_for_water_sample(oi_t *sensor_data) {
     // cliff sensors return high values for reflective surfaces
     // blue paper should have a specific reflection signature
     
     // read cliff sensor signals (these are analog values, not binary cliff detection)
     int left_cliff = sensor_data->cliffLeftSignal;
     int front_left_cliff = sensor_data->cliffFrontLeftSignal;
     int front_right_cliff = sensor_data->cliffFrontRightSignal;
     int right_cliff = sensor_data->cliffRightSignal;
     
     // check if any sensor detects blue paper
     // blue paper reflects IR differently than black surface or white tape
     if (front_left_cliff > BLUE_THRESHOLD || front_right_cliff > BLUE_THRESHOLD) {
         return true;
     }
     
     return false;
 }
 
 // check for cliff/hole detection to prevent falling
 bool check_for_cliff(oi_t *sensor_data) {
     // cliff sensors return binary indication of cliff detection
     if (sensor_data->cliffLeft || sensor_data->cliffFrontLeft || 
         sensor_data->cliffFrontRight || sensor_data->cliffRight) {
         return true;
     }
     return false;
 }
 
 // spin in place to simulate water sample collection
 void spin_for_water_sample(oi_t *sensor_data) {
     char toPutty[100];
     
     lcd_printf("Collecting\nWater Sample");
     
     // spin 360 degrees to simulate collection
     sprintf(toPutty, "Spinning to collect water sample...\r\n");
     uart_sendStr(toPutty);
     
     // turn clockwise 360 degrees
     turn_clockwise(sensor_data, 360);
     
     // completion message
     sprintf(toPutty, "Water sample collection complete!\r\n");
     uart_sendStr(toPutty);
     lcd_printf("Manual Mode");
 }