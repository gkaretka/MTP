/*
 * prj.c
 *
 *  Created on: Mar 18, 2021
 *      Author: gkaretka
 */

#include "prj.h"

extern uint8_t cnt_dir;

void state_machine_executor(void)
{
	switch(machine_state)
	{
	case BLINKY_4HZ:
		// turn off PWM
		LL_TIM_DisableCounter(TIM1);
		LL_TIM_DisableAllOutputs(TIM1);

		// turn on LED
		TIM2->CCR1 = 124;	// LED PWM 50%
		LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
		LL_TIM_EnableAllOutputs(TIM2);
		LL_TIM_EnableCounter(TIM2);
		machine_state++;
		break;
	case BLINKY_W8:
		// idle
		break;

	case CNT_UP_DOWN:
		TIM2->CCR1 = 249; // LED PWM 100%
		cnt_dir = 1; // up
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
		LL_TIM_EnableAllOutputs(TIM1);
		LL_TIM_EnableCounter(TIM1);
		LL_TIM_EnableIT_UPDATE(TIM1);

		machine_state++;
		break;
	case CNT_UP_W8:
		// idle
		break;

	case SQUARE_WAVE:
		cnt_dir = 1; // up
		// everything is enabled from previous step
		machine_state++;
		break;
	case SQUARE_WAVE_W8:
		// idle
		break;
	default:
		machine_state = BLINKY_4HZ;
	}
}

void increase_state(void)
{
	machine_state++;
}
