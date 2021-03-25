/*
 * prj.h
 *
 *  Created on: Mar 18, 2021
 *      Author: gkaretka
 */

#ifndef INC_PRJ_H_
#define INC_PRJ_H_

#include "main.h"

#define FtoQ15(x) 	((int16_t)((float)(x) >= 0.9996f ? 32767 : (float)(x) < -1.f ? -32768 : ((float)(x) * (1 << 15))))

#define MYARR				7999
#define	COUNTER_2HZ_STEP	0.100f	// adjustable step size
#define COUNTER_2HZ_CNT		(((MYARR+1)*COUNTER_2HZ_STEP)/2)

#define UtoCCR1(u)	((int16_t)((((int32_t)(u)/2 + FtoQ15(0.5)) * MYARR) >> 15))
#define UtoCCR2(u)	((int16_t)((((int32_t)(-(u))/2 + FtoQ15(0.5)) * MYARR) >> 15))

enum
{
	BLINKY_4HZ		=	0,
	BLINKY_W8		=	1,
	CNT_UP_DOWN		=	2,
	CNT_UP_W8		=	3,
	SQUARE_WAVE		=	4,
	SQUARE_WAVE_W8	=	5,
	STATE_CNT		=	6,
} machine_state;

void state_machine_executor(void);
void increase_state(void);
int16_t q_add_sat(int16_t a, int16_t b);

#endif /* INC_PRJ_H_ */
