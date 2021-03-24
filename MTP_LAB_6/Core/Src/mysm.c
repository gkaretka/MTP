/*
 * mysm.c
 *
 *  Created on: Mar 18, 2021
 *      Author: gkaretka
 */

#include "main.h"
#include "mysm.h"

int16_t q15Uax, q15Uax2;
float fUax;

enum
{
	S1	=	0,
	S2	=	1,
	S3	=	2,
	N	=	3,
} ActualState;

void loop(void)
{
/*	HAL_Delay(100);
	q15Uax = FtoQ15(fUax);
	q15Uax2 += FtoQ15(0.1);
*/
}

void my_state_1_handler(void) {

}

void my_state_machine(void)
{
	switch(ActualState)
	{
	case S1:
		my_state_1_handler();
		break;
	case S2:
		break;
	case S3:
		break;
	default:
		break;
	}
}

void my_state_machine_inc_state(void)
{
	if(ActualState == N) {
		ActualState = S1;
	}
}

/*
 * ISR
 * T=100us
 */
void my_tim_isr(void)
{
	my_state_machine();
	TIM1->CCR1 = UtoCCR1(q15Uax);
	TIM1->CCR2 = UtoCCR2(q15Uax);
}
