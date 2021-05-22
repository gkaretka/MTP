/*
 * myapp.h
 *
 *  Created on: Apr 29, 2021
 *      Author: gkaretka
 */

#ifndef INC_MYAPP_H_
#define INC_MYAPP_H_

#include "main.h"
#include "controllers.h"

typedef union
{
	uint64_t uldata;
	struct {
		uint16_t adc_m0;
		uint16_t adc_m1;
		uint16_t internal_ref;
	} samples;
} adcdata_t;

void hw_init(void);
void sw_turnoff(void);
void loop(void);
void sw_turnon(void);

#endif /* INC_MYAPP_H_ */
