/*
 * adc_tim_dual.h
 *
 *  Created on: Apr 8, 2021
 *      Author: gkaretka
 */

#ifndef INC_ADC_TIM_DUAL_H_
#define INC_ADC_TIM_DUAL_H_

#include "main.h"

void hw_init(void);
void main_loop(void);
void main_isr(void);

typedef union
{
	uint32_t	uwData[2];
	uint16_t	usData[4];

	struct {
		uint16_t ch6_I1;	// adc1 sample 1
		uint16_t ch1_V1;	// adc2 sample 1
		uint16_t ch8_Vdc2;	// adc1 sample 2
		uint16_t ch11_Vdc1;	// adc2 sample2
	} samples;
} ADC12Data_t;


#endif /* INC_ADC_TIM_DUAL_H_ */
