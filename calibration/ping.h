/*
 * ping.h
 *
 *  Created on: Oct 28, 2021
 *      Author: tdbolton
 */

#ifndef PING_H_
#define PING_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
#include "driverlib/interrupt.h"
#include "Timer.h"

void ping_init(void);
void send_pulse(void);
float ping_read(void);
void TIMER3B_HANDLER(void);



#endif /* PING_H_ */
