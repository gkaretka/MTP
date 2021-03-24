/*
 * mysm.h
 *
 *  Created on: Mar 18, 2021
 *      Author: gkaretka
 */

#ifndef INC_MYSM_H_
#define INC_MYSM_H_

void loop(void);
void my_tim_isr(void);
void my_state_machine_inc_state(void);

#define FtoQ15(x) 	((int16_t)((float)(x) >= 0.999995f ? 32767 : (float)(x) < -1.f ? -32768 : ((float)(x) * (1 << 15))))

#define MYARR		6399
#define UtoCCR1(u)	((int16_t)((((int32_t)(u)/2 + FtoQ15(0.5)) * MYARR) >> 15))

#define UtoCCR2(u)	((int16_t)((((int32_t)(-(u))/2 + FtoQ15(0.5)) * MYARR) >> 15))


#endif /* INC_MYSM_H_ */
