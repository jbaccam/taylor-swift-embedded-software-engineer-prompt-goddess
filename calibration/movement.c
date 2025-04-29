
#include "movement.h"
#include "uart.h"

void move_backwards(oi_t *sensor, int centimeters) {        //moves cybot backwards
    double sum = 0;
    oi_setWheels(-50, -50);                               // sets speed of each wheel to -100 mm/s
    while (sum < centimeters * 10) {                        // sets desired distance
        oi_update(sensor);
        sum -= sensor->distance;
    }
    oi_setWheels(0, 0);                                     // stops cybot
}

void move_forward(oi_t *sensor, int centimeters) { //moves the cybot forward
    double sum = 0;
    char toPutty[60];
    char *toPutty_ptr = toPutty;

    while (sum < centimeters * 10) { // sets desired distance
        oi_setWheels(80, 80);      // sets speed of each wheel to 200 mm/s
        oi_update(sensor);
        sum += sensor->distance;       // sums distance traveled
        if (sensor->bumpLeft) {         //executes  if bumped left
//            move_backwards(sensor, 15);         //moves cybot backwards 15 cm
//            turn_counterclockwise(sensor, 90);  // turns cybot counterclockwise 90 degrees
//            oi_setWheels(200, 200);             // sets speed of each wheel to 200 mm/s after turning
//            int sumLeft = 0;
//            while (sumLeft < 250) {            //causes the bot to move 25 cm away from obstacle
//                oi_update(sensor);
//                sumLeft += sensor->distance;
//            }
//            turn_clockwise(sensor, 90);         // turns cybot clockwise 90 degrees
//            sum = sum - 150;                    // substracks 15 cm from distance traveled because of the cybot moving backwards
            sprintf(toPutty, "Bumped Left after moving %f cm.\r\n", sum/10.0);
            uart_sendStr(toPutty_ptr);
            break;
        }
        if (sensor->bumpRight) {                    //executes  if bumped right
//            move_backwards(sensor, 15);             //moves cybot backwards 15 cm
//            turn_clockwise(sensor, 90);             // turns cybot clockwise 90 degrees
//            oi_setWheels(80, 80);                 // sets speed of each wheel to 200 mm/s after turning
//            int sumLeft = 0;
//            while (sumLeft < 250) {                 //causes the bot to move 25 cm away from obstacle
//                oi_update(sensor);
//                sumLeft += sensor->distance;
//            }
//            turn_counterclockwise(sensor, 90);      // turns cybot counterclockwise 90 degrees
//            sum = sum - 150;                         // substracks 15 cm from distance traveled because of the cybot moving backwards
            sprintf(toPutty, "Bumped Right after moving %f cm.\n\r", sum/10.0);
            uart_sendStr(toPutty_ptr);
            break;
        }
        if (sensor->cliffFrontLeft) {
            sprintf(toPutty, "Cliff Front Left after moving %f cm.\n\r", sum/10.0);
            uart_sendStr(toPutty_ptr);
            break;
        }
        if (sensor->cliffFrontRight) {
            sprintf(toPutty, "Cliff Front Right after moving %f cm.\n\r", sum/10.0);
            uart_sendStr(toPutty_ptr);
            break;
        }
    }
    oi_setWheels(0, 0);                              // stops cybot
}

void go_to_object(oi_t *sensor, int centimeters) {
    double sum = 0;
    signed int numBumps = 0;

    while (sum < ((centimeters * 10) + 200)) { // sets desired distance
        oi_setWheels(80, 80);      // sets speed of each wheel to 100 mm/s
        oi_update(sensor);
        sum += sensor->distance;       // sums distance traveled
        if (sensor->bumpLeft) {         //executes  if bumped left
            numBumps++;
            move_backwards(sensor, 15);         //moves cybot backwards 15 cm
            autoturn_counterclockwise(sensor, 90);  // turns cybot counterclockwise 90 degrees
            oi_setWheels(80, 80);             // sets speed of each wheel to 200 mm/s after turning
            int sumLeft = 0;
            while (sumLeft < 250) {            //causes the bot to move 25 cm away from obstacle
                oi_update(sensor);
                sumLeft += sensor->distance;
            }
            autoturn_clockwise(sensor, 90);         // turns cybot clockwise 90 degrees
            sum = sum - 150;                    // subtracts 15 cm from distance traveled because of the cybot moving backwards
        }
        if (sensor->bumpRight) {                    //executes  if bumped right
            numBumps--;
            move_backwards(sensor, 15);             //moves cybot backwards 15 cm
            autoturn_clockwise(sensor, 90);             // turns cybot clockwise 90 degrees
            oi_setWheels(80, 80);                 // sets speed of each wheel to 200 mm/s after turning
            int sumLeft = 0;
            while (sumLeft < 250) {                 //causes the bot to move 25 cm away from obstacle
                oi_update(sensor);
                sumLeft += sensor->distance;
            }
            autoturn_counterclockwise(sensor, 90);      // turns cybot counterclockwise 90 degrees
            sum = sum - 150;                         // subtracts 15 cm from distance traveled because of the cybot moving backwards
        }
        if(numBumps == 0 && sum > ((centimeters * 10) - 115)) {
            break;
        }
    }
    oi_setWheels(0, 0);                              // stops cybot
    int sumLeftRight = 0;
    if (numBumps < 0){
        autoturn_counterclockwise(sensor, 90);
    }
    if (numBumps > 0){
        autoturn_clockwise(sensor, 90);
    }
    while (sumLeftRight < abs(250*numBumps) - 45 - 180) {
        oi_update(sensor);
        oi_setWheels(80, 80);                 // sets speed of each wheel to 200 mm/s after turning
        sumLeftRight += sensor->distance;
    }
//    go_to_object(sensor, (abs(25*numBumps) - 4));
    oi_setWheels(0, 0);
}

void autoturn_clockwise(oi_t *sensor, int degrees) {        //turns cybot clockwise
    double sum = 0;
    oi_setWheels(60, -60);                              //sets the left wheel to 50 mm/s and the right wheel to -50 mm/s
    while (sum < degrees) {                             // sets the desired angle
        oi_update(sensor);
        sum += sensor->angle;
    }
    oi_setWheels(0, 0);                                 // stops cybot
}

void autoturn_counterclockwise(oi_t *sensor, int degrees){      //turns cybot counterclockwise
    double sum = 0;
    oi_setWheels(-60, 60);                                  //sets the left wheel to -50 mm/s and the right wheel to 50 mm/s
    while (sum < degrees){                                  //sets the desired angle
        oi_update(sensor);
        sum -= sensor->angle;
    }
    oi_setWheels(0, 0);                                // stops cybot
}

void turn_clockwise(oi_t *sensor, int degrees) {        //turns cybot clockwise
    double sum = 0;
    oi_setWheels(38, -38);                              //sets the left wheel to 50 mm/s and the right wheel to -50 mm/s
    while (sum < degrees) {                             // sets the desired angle
        oi_update(sensor);
        sum += sensor->angle;
    }
    oi_setWheels(0, 0);                                 // stops cybot
}

void turn_counterclockwise(oi_t *sensor, int degrees){      //turns cybot counterclockwise
    double sum = 0;
    oi_setWheels(-38, 38);                                  //sets the left wheel to -50 mm/s and the right wheel to 50 mm/s
    while (sum < degrees){                                  //sets the desired angle
        oi_update(sensor);
        sum -= sensor->angle;
    }
    oi_setWheels(0, 0);                                // stops cybot
}


