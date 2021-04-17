/*
 * pireg_app.c
 *
 *  Created on: Apr 15, 2021
 *      Author: gkaretka
 */

#include "pireg_app.h"
#include "controllers.h"
#include "dataAcq.h"

filter_lp_inst_t filter_1;
integrator_inst_t capacitor_1;
regPI_inst_t pireg_current_1;

int cnt;

float signal1, signal2, signal3, signalw;

void hw_init(void)
{
	// hardware
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);

	LL_TIM_ClearFlag_CC4(TIM1);
	LL_TIM_EnableIT_CC4(TIM1);

	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM1);

	// software
	pireg_current_1.Ts = capacitor_1.Ts = filter_1.Ts = ((float)TIM1_ARR + 1.f)/(float)SystemCoreClock;

	filter_1.tau = 0.5;				// seconds
	filter_lp_init(&filter_1);

	capacitor_1.gain = 1 / 1.0f; // 1 / C
	integrator_gain_init(&capacitor_1);

	pireg_current_1.Kp = 0.0001f;
	pireg_current_1.Ti = 10.0f;
	pireg_current_1.h_lim = 1.0f;
	pireg_current_1.l_lim = -1.0f;

	regPI_p_init(&pireg_current_1);

	ClearBuffer();
}

void main_loop(void)
{
	filter_lp_init(&filter_1);
	regPI_p_init(&pireg_current_1);
}

void periodic_task(void)
{
	filter_lp(signal1, &signal2, &filter_1);
	integrator_gain(signal1, &signal3, &capacitor_1);
	regPI_p(signalw - signal3, &signal1, &pireg_current_1);

	DumpTrace();
}
