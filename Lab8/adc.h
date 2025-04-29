/*
 * adc.h
 *
 *  Created on: Apr 3, 2025
 *      Author: jjbaccam
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>

void adc_init(void);
int adc_read(void);




#endif /* ADC_H_ */
