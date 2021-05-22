/*
 * myapp.c
 *
 *  Created on: 29. 4. 2021
 *      Author: gkaretka
 */

#include "mysm.h"

void state_machine_executor(void)
{
	update_enable_state();

	if (enabled_state == DISABLED) {
		machine_state = BLOCKED;
	} else {
		if (running_state == R_STOP) {
			machine_state = STOP;
		} else {
			machine_state = RUNNING;
		}
	}

	switch(machine_state)
	{
	case BLOCKED:
		// turn on red, turn off green
		GPIOB->ODR |= GPIO_PIN_1;
		GPIOB->ODR &= ~(GPIO_PIN_2);

		// turn off PWM
		LL_TIM_DisableCounter(TIM1);
		LL_TIM_DisableAllOutputs(TIM1);
		LL_TIM_DisableIT_CC3(TIM1);
		break;

	case RUNNING:
		// turn off red, turn on green
		GPIOB->ODR &= ~(GPIO_PIN_1);
		GPIOB->ODR |= GPIO_PIN_2;

		// turn on PWM
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
		LL_TIM_EnableAllOutputs(TIM1);
		LL_TIM_EnableCounter(TIM1);
		LL_TIM_EnableIT_UPDATE(TIM1);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
		break;

	case STOP:
		// turn on red, turn on green
		GPIOB->ODR |= GPIO_PIN_1;
		GPIOB->ODR |= GPIO_PIN_2;

		// turn on PWM
		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
		LL_TIM_EnableAllOutputs(TIM1);
		LL_TIM_EnableCounter(TIM1);
		LL_TIM_EnableIT_UPDATE(TIM1);

		LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);

		// set duty cycle to 0
		sw_turnoff();
		break;

	default:
		// does not happen
		break;
	}
}

void run_stop_flip(void)
{
	if (running_state == R_STOP) {
		running_state = R_RUN;
	} else {
		running_state = R_STOP;
	}
}

void update_enable_state(void)
{
	// high blocked, low enabled (jumper present)
	if ((GPIOB->IDR >> 7) & 0x01) {
		enabled_state = DISABLED;
	} else {
		enabled_state = ENABLED;
	}
}
