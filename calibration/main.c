
#include "adc.h"
#include "timer.h"
#include "lcd.h"
#include "uart.h"
#include <open_interface.h>
#include <movement.h>
#include "ping.h"
#include "servo.h"
#include "button.h"

volatile  char uart_data;  // Your UART interrupt code can place read data here
volatile  char flag = 0;       // Your UART interrupt can update this flag

void main(void)
{
    timer_init();
    lcd_init();
    uart_init(115200);
    adc_init();
    ping_init();
    servo_init();
    button_init();
    init_button_interrupts();

//    calibrate_servo();

    oi_t *sensor_data = oi_alloc();
    oi_init(sensor_data);           //Initializes sensor
    uart_interrupt_init();

//    while(1) {
//        oi_free(sensor_data);
//    }

    int i = 0;              //Constants and counters for for loops
    int j = 0;
    int k = 0;
    char inputCharacter;    //Input into PuTTy
    char toPutty[100];      //Used to make the PuTTy output a message
    char *toPutty_ptr = toPutty;
    int IRmeasurement;
    float pingDistance;

//    while (1) {
//        sprintf(toPutty, "Cliff Signal Value: %d\n\r", sensor_data->cliffFrontLeftSignal);
//        uart_sendStr(toPutty_ptr);
//        timer_waitMillis(10);
//    }

    while(1) {                  //Cybot will wait until 'enter' is pressed
        inputCharacter = uart_receive();
        if (inputCharacter == '\r') {
            break;
        }
    }
    while (1) {
//Automatic Mode

        while (flag == 0) {
            i = 0;
            j = 0;
            k = 0;

            int sensorAngle[90] = {0};        //sensorAngle will be set to angles where there is an object
            float sensorDistance[90] = {0};   //sensorDistance will be set to the distance to the object

            sprintf(toPutty, "Degrees\tDistance (cm)\n\r");     //Sets up the degree and distance display before the scanner reads degrees and distances
            uart_sendStr(toPutty_ptr);

            for (i = 0; i <= 180; i = i + 2) {                              //Has the cybot scan every 2 degrees from 0 to 180 degrees
                servo_move(i);
                pingDistance = ping_read();
                IRmeasurement = adc_read();
                if (IRmeasurement > 400) {                                //The cybot will detect an object when the IR output is greater than 900
                    sprintf(toPutty, "%d\t%f\n\r", i, pingDistance);     //Sends the object angle and distance at that angle to the PuTTy
                    while (toPutty[j] != '\0') {
                        uart_sendChar(toPutty[j]);
                        j++;
                    }
                    sensorAngle[k] = i;                                     //Angle and distance at that angle are stored in their variables at element k, and k will increase after storing an angle and distance
                    sensorDistance[k] = pingDistance;
                    k++;


                }
                j = 0;
            }

            autoturn_clockwise(sensor_data, 180);                                   //Turns the cybot 180 degrees so it can scan the remaining 180 degrees
            for (i = 2; i <= 180; i = i + 2) {
                servo_move(i);
                pingDistance = ping_read();
                IRmeasurement = adc_read();
                if (IRmeasurement > 400) {                                    //Detects an object when the IR output is greater than 900
                    sprintf(toPutty, "%d\t%f\n\r", i + 180, pingDistance);   //Sends object angle and distance to the PuTTy
                    while (toPutty[j] != '\0') {
                        uart_sendChar(toPutty[j]);
                        j++;
                    }
                    sensorAngle[k] = i + 180;
                    sensorDistance[k] = pingDistance;                        //Angle and distance stored in the variables at element k
                    k++;
                }
                j = 0;
            }

            int avgAngle[10] = {0};     //Will be set to the average angle of the object
            float avgDist[10] = {0};    //Will be set to the average distance to the object
            int width[10] = {0};        //Will be set to the angular width of the object
            int l = 0;                  //Counter that increases for each object
            float StartDist[10] = {0};  //Distance at the object's starting angle
            float EndDist[10] = {0};    //Distance at the object's ending angle

            for (i = 1; i <= k; i++) {
                if (StartDist[l] == 0) {
                    StartDist[l] = sensorDistance[i];                       //StartDist will have the distance at the start angle
                }
                if (sensorAngle[i] - sensorAngle[i-1] <= 4  && i != k) {    //Compares current angle to the last angle to tell if it is the same object
                    if (sensorAngle[i] - sensorAngle[i-1] == 4) {           //The sensor sometimes skips an angle, and this will prevent it from saying it is two separate objects
                        j = j + 4;                                          //The j's are used for width and finding average
                        avgAngle[l] = avgAngle[l] + 4*sensorAngle[i];       //For setting up averages; j increases by 4, so the angle/distance should be multiplied by 4
                        avgDist[l] = avgDist[l] + 4*sensorDistance[i];
                        EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle

                    }
                    else {
                        j = j + 2;                                          //The j's are used for width and finding average
                        avgAngle[l] = avgAngle[l] + 2*sensorAngle[i];       //For setting up averages; j increases by 2, so the angle/distance should be multiplied by 2
                        avgDist[l] = avgDist[l] + 2*sensorDistance[i];
                        EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle
                    }
                }
                else {
                    if (j <= 2) {                           //The scanner sometimes reads an object when there isn't one, so this will prevent it from affecting the data
                        j = 0;                              //It sets j, the averages, and the start distance back to 0 since there isn't an object
                        avgAngle[l] = 0;
                        avgDist[l] = 0;
                        StartDist[l] = 0;
                    }
                    else {                                  //At the end of an object
                        width[l] = j;                       //Sets angular width to j
                        avgAngle[l] = avgAngle[l] / j;      //Finds the averages
                        avgDist[l] = avgDist[l] / j;
                        j = 0;                              //Resets j for the next object
                        l++;                                //l increases for counting number of objects and storing object data in the arrays
                    }
                }
            }


            int SmallestWidth = 1000;
            int SmallestWidthAngle = 0;
            int SmallestWidthDistance = 0;


            float LinearWidth[10] = {0};    //Stores object linear width
            for (i = 0; i < l; i++) {
                LinearWidth[i] = sqrt(pow(StartDist[i], 2) + pow(EndDist[i], 2) - 2*StartDist[i]*EndDist[i]*cos((width[i])*(M_PI / 180)));  //Uses Law of Cosines to find the linear width of each object
            }

            j = 0;
            sprintf(toPutty, "\n\rObject\tAngle\tDistance\tWidth\n\r");     //Sets up the PuTTy display before reading off the object angle, distance, and width
            while (toPutty[j] != '\0') {
                uart_sendChar(toPutty[j]);
                j++;
            }

            for (i = 0; i < l; i++) {
                sprintf(toPutty, "%d\t%d\t%f\t%f\n\r", i + 1, avgAngle[i], avgDist[i], LinearWidth[i]);     //Prints each object's average angle, average distance, and linear width
                j = 0;
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }
            }

            for (i = 0; i < l; i++) {
                if (LinearWidth[i] < SmallestWidth) {   //Finds the object with the smallest linear width and finds their angle and distance
                    SmallestWidth = LinearWidth[i];
                    SmallestWidthAngle = avgAngle[i];
                    SmallestWidthDistance = avgDist[i];
                }
            }

//            if (SmallestWidthDistance <= 5) {
//                while (flag == 0) {
//
//                }
//                break;
//            }

            if (SmallestWidthAngle <= 90) {                                     //Has the cybot move to the smallest width object if the object is at 0 to 90 degrees
                autoturn_clockwise(sensor_data, 90 + SmallestWidthAngle);
                go_to_object(sensor_data, SmallestWidthDistance);
            }
            else if (SmallestWidthAngle <= 270) {
                autoturn_counterclockwise(sensor_data, 270 - SmallestWidthAngle);   //Has the cybot move to the smallest width object if the object is at 90 to 270 degrees
                go_to_object(sensor_data, SmallestWidthDistance);
            }
            else {                                                              //Has the cybot move to the smallest width object if the object is at 270 to 360 degrees
                turn_clockwise(sensor_data, SmallestWidthAngle - 270);
                go_to_object(sensor_data, SmallestWidthDistance);
            }
            while (flag == 0) {
                inputCharacter = uart_receive();
                if (inputCharacter == '\r') {
                    break;
                }
            }

        }

        sprintf(toPutty, "\rSwitched to Manual Mode.\n\r");
        uart_sendStr(toPutty_ptr);

//Manual Mode

        while (flag == 1) {
            i = 0;
            j = 0;
            k = 0;

            inputCharacter = uart_receive();
            uart_sendChar(inputCharacter);
            if (inputCharacter == 'w'){         //Moves the bot forward when 'w' is pressed
                sprintf(toPutty, "\rMoving Forward 5cm.\n\r");
                uart_sendStr(toPutty_ptr);
                move_forward(sensor_data, 5);
                sprintf(toPutty, "\rDone Moving Forward.\n\r");
                uart_sendStr(toPutty_ptr);
            }
            if (inputCharacter == 's'){         //Moves the bot backward when 's' is pressed
                j = 0;
                sprintf(toPutty, "\rMoving Backwards 5cm.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }
                move_backwards(sensor_data, 5);
                j = 0;
                sprintf(toPutty, "\rDone Moving Backwards.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }
            }
            if (inputCharacter == 'd'){         //Turns the bot counterclockwise when 'a' is pressed
                j = 0;
                sprintf(toPutty, "\rTurning Clockwise 10 Degrees.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }
                turn_counterclockwise(sensor_data, 10);
                j = 0;
                sprintf(toPutty, "\rDone Turning Clockwise.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }
            }
            if (inputCharacter == 'a'){         //Turns the bot clockwise when 'd' is pressed
                j = 0;
                sprintf(toPutty, "\rTurning Counterclockwise 10 Degrees.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }
                turn_clockwise(sensor_data, 10);
                j = 0;
                sprintf(toPutty, "\rDone Turning Counterclockwise.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }
            }
            if (inputCharacter == 'm'){
                oi_setWheels(0, 0);

                j = 0;
                sprintf(toPutty, "\rBeginning 180 Degree Scan.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }

                int sensorAngle[80] = {0};        //sensorAngle will be set to angles where there is an object
                float sensorDistance[80] = {0};   //sensorDistance will be set to the distance to the object

                sprintf(toPutty, "Degrees\tDistance (cm)\n\r");     //Sets up the degree and distance display before the scanner reads degrees and distances
                while (toPutty[i] != '\0') {
                    uart_sendChar(toPutty[i]);
                    i++;
                }

                for (i = 0; i <= 180; i = i + 2) {                              //Has the cybot scan every 2 degrees from 0 to 180 degrees
                    servo_move(i);
                    pingDistance = ping_read();
                    IRmeasurement = adc_read();
                    if (IRmeasurement > 400) {                                //The cybot will detect an object when the IR output is greater than 900
                        sprintf(toPutty, "%d\t%f\n\r", i, pingDistance);     //Sends the object angle and distance at that angle to the PuTTy
                        while (toPutty[j] != '\0') {
                            uart_sendChar(toPutty[j]);
                            j++;
                        }
                        sensorAngle[k] = i;                                     //Angle and distance at that angle are stored in their variables at element k, and k will increase after storing an angle and distance
                        sensorDistance[k] = pingDistance;
                        k++;
                    }
                    j = 0;
                }

                int avgAngle[10] = {0};     //Will be set to the average angle of the object
                float avgDist[10] = {0};    //Will be set to the average distance to the object
                int width[10] = {0};              //Will be set to the angular width of the object
                int l = 0;                  //Counter that increases for each object
                float StartDist[10] = {0};  //Distance at the object's starting angle
                float EndDist[10] = {0};    //Distance at the object's ending angle

                for (i = 1; i <= k; i++) {
                    if (StartDist[l] == 0) {
                        StartDist[l] = sensorDistance[i];                       //StartDist will have the distance at the start angle
                    }
                    if (sensorAngle[i] - sensorAngle[i-1] <= 4  && i != k) {    //Compares current angle to the last angle to tell if it is the same object
                        if (sensorAngle[i] - sensorAngle[i-1] == 4) {           //The sensor sometimes skips an angle, and this will prevent it from saying it is two separate objects
                            j = j + 4;                                          //The j's are used for width and finding average
                            avgAngle[l] = avgAngle[l] + 4*sensorAngle[i];       //For setting up averages; j increases by 4, so the angle/distance should be multiplied by 4
                            avgDist[l] = avgDist[l] + 4*sensorDistance[i];
                            EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle

                        }
                        else {
                            j = j + 2;                                          //The j's are used for width and finding average
                            avgAngle[l] = avgAngle[l] + 2*sensorAngle[i];       //For setting up averages; j increases by 2, so the angle/distance should be multiplied by 2
                            avgDist[l] = avgDist[l] + 2*sensorDistance[i];
                            EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle
                        }
                    }
                    else {
                        if (j <= 2) {                           //The scanner sometimes reads an object when there isn't one, so this will prevent it from affecting the data
                            j = 0;                              //It sets j, the averages, and the start distance back to 0 since there isn't an object
                            avgAngle[l] = 0;
                            avgDist[l] = 0;
                            StartDist[l] = 0;
                        }
                        else {                                  //At the end of an object
                            width[l] = j;                       //Sets angular width to j
                            avgAngle[l] = avgAngle[l] / j;      //Finds the averages
                            avgDist[l] = avgDist[l] / j;
                            j = 0;                              //Resets j for the next object
                            l++;                                //l increases for counting number of objects and storing object data in the arrays
                        }
                    }
                }

                float LinearWidth[10] = {0};    //Stores object linear width
                for (i = 0; i < l; i++) {
                    LinearWidth[i] = sqrt(pow(StartDist[i], 2) + pow(EndDist[i], 2) - 2*StartDist[i]*EndDist[i]*cos((width[i])*(M_PI / 180)));  //Uses Law of Cosines to find the linear width of each object
                }

                j = 0;
                sprintf(toPutty, "\n\rObject\tAngle\tDistance\tWidth\n\r");     //Sets up the PuTTy display before reading off the object angle, distance, and width
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }

                for (i = 0; i < l; i++) {
                    sprintf(toPutty, "%d\t%d\t%f\t%f\n\r", i + 1, avgAngle[i], avgDist[i], LinearWidth[i]);     //Prints each object's average angle, average distance, and linear width
                    j = 0;
                    while (toPutty[j] != '\0') {
                        uart_sendChar(toPutty[j]);
                        j++;
                    }
                }


            }
            if (inputCharacter == 'n') {
                oi_setWheels(0, 0);

                j = 0;
                sprintf(toPutty, "\rBeginning 360 Degree Scan.\n\r");
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }

                int sensorAngle[80] = {0};        //sensorAngle will be set to angles where there is an object
                float sensorDistance[80] = {0};   //sensorDistance will be set to the distance to the object

                sprintf(toPutty, "Degrees\tDistance (cm)\n\r");     //Sets up the degree and distance display before the scanner reads degrees and distances
                while (toPutty[i] != '\0') {
                    uart_sendChar(toPutty[i]);
                    i++;
                }

                for (i = 0; i <= 180; i = i + 2) {                              //Has the cybot scan every 2 degrees from 0 to 180 degrees
                    servo_move(i);
                    pingDistance = ping_read();
                    IRmeasurement = adc_read();
                    if (IRmeasurement > 400) {                                //The cybot will detect an object when the IR output is greater than 900
                        sprintf(toPutty, "%d\t%f\n\r", i, pingDistance);     //Sends the object angle and distance at that angle to the PuTTy
                        while (toPutty[j] != '\0') {
                            uart_sendChar(toPutty[j]);
                            j++;
                        }
                        sensorAngle[k] = i;                                     //Angle and distance at that angle are stored in their variables at element k, and k will increase after storing an angle and distance
                        sensorDistance[k] = pingDistance;
                        k++;
                    }
                    j = 0;
                }

                autoturn_clockwise(sensor_data, 180);                                   //Turns the cybot 180 degrees so it can scan the remaining 180 degrees
                for (i = 2; i <= 180; i = i + 2) {
                    servo_move(i);
                    pingDistance = ping_read();
                    IRmeasurement = adc_read();
                    if (IRmeasurement > 400) {                                    //Detects an object when the IR output is greater than 900
                        sprintf(toPutty, "%d\t%f\n\r", i + 180, pingDistance);   //Sends object angle and distance to the PuTTy
                        while (toPutty[j] != '\0') {
                            uart_sendChar(toPutty[j]);
                            j++;
                        }
                        sensorAngle[k] = i + 180;
                        sensorDistance[k] = pingDistance;                        //Angle and distance stored in the variables at element k
                        k++;
                    }
                    j = 0;
                }

                int avgAngle[10] = {0};     //Will be set to the average angle of the object
                float avgDist[10] = {0};    //Will be set to the average distance to the object
                int width[10] = {0};              //Will be set to the angular width of the object
                int l = 0;                  //Counter that increases for each object
                float StartDist[10] = {0};  //Distance at the object's starting angle
                float EndDist[10] = {0};    //Distance at the object's ending angle

                for (i = 1; i <= k; i++) {
                    if (StartDist[l] == 0) {
                        StartDist[l] = sensorDistance[i];                       //StartDist will have the distance at the start angle
                    }
                    if (sensorAngle[i] - sensorAngle[i-1] <= 4  && i != k) {    //Compares current angle to the last angle to tell if it is the same object
                        if (sensorAngle[i] - sensorAngle[i-1] == 4) {           //The sensor sometimes skips an angle, and this will prevent it from saying it is two separate objects
                            j = j + 4;                                          //The j's are used for width and finding average
                            avgAngle[l] = avgAngle[l] + 4*sensorAngle[i];       //For setting up averages; j increases by 4, so the angle/distance should be multiplied by 4
                            avgDist[l] = avgDist[l] + 4*sensorDistance[i];
                            EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle

                        }
                        else {
                            j = j + 2;                                          //The j's are used for width and finding average
                            avgAngle[l] = avgAngle[l] + 2*sensorAngle[i];       //For setting up averages; j increases by 2, so the angle/distance should be multiplied by 2
                            avgDist[l] = avgDist[l] + 2*sensorDistance[i];
                            EndDist[l] = sensorDistance[i];                     //EndDist will be the distance at the object's last angle
                        }
                    }
                    else {
                        if (j <= 2) {                           //The scanner sometimes reads an object when there isn't one, so this will prevent it from affecting the data
                            j = 0;                              //It sets j, the averages, and the start distance back to 0 since there isn't an object
                            avgAngle[l] = 0;
                            avgDist[l] = 0;
                            StartDist[l] = 0;
                        }
                        else {                                  //At the end of an object
                            width[l] = j;                       //Sets angular width to j
                            avgAngle[l] = avgAngle[l] / j;      //Finds the averages
                            avgDist[l] = avgDist[l] / j;
                            j = 0;                              //Resets j for the next object
                            l++;                                //l increases for counting number of objects and storing object data in the arrays
                        }
                    }
                }

                float LinearWidth[10] = {0};    //Stores object linear width
                for (i = 0; i < l; i++) {
                    LinearWidth[i] = sqrt(pow(StartDist[i], 2) + pow(EndDist[i], 2) - 2*StartDist[i]*EndDist[i]*cos((width[i])*(M_PI / 180)));  //Uses Law of Cosines to find the linear width of each object
                }

                j = 0;
                sprintf(toPutty, "\n\rObject\tAngle\tDistance\tWidth\n\r");     //Sets up the PuTTy display before reading off the object angle, distance, and width
                while (toPutty[j] != '\0') {
                    uart_sendChar(toPutty[j]);
                    j++;
                }

                for (i = 0; i < l; i++) {
                    sprintf(toPutty, "%d\t%d\t%f\t%f\n\r", i + 1, avgAngle[i], avgDist[i], LinearWidth[i]);     //Prints each object's average angle, average distance, and linear width
                    j = 0;
                    while (toPutty[j] != '\0') {
                        uart_sendChar(toPutty[j]);
                        j++;
                    }
                }
                autoturn_clockwise(sensor_data, 180);
            }
        }

        j = 0;
        sprintf(toPutty, "\rSwitched to Automatic Mode.\n\r");
        while (toPutty[j] != '\0') {
            uart_sendChar(toPutty[j]);
            j++;
        }
    }



}
