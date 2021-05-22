/*
 * myapp.h
 *
 *  Created on: 29. 4. 2021
 *      Author: gkaretka
 */

#ifndef INC_MYSM_H_
#define INC_MYSM_H_

#include "main.h"
#include "myapp.h"

enum
{
	STOP	= 0,
	RUNNING	= 1,
	BLOCKED = 2,
} machine_state;

enum {
	R_STOP	= 0,
	R_RUN	= 1,
} running_state;

enum
{
	DISABLED = 0,
	ENABLED  = 1,
} enabled_state;

void state_machine_executor(void);
void run_stop_flip(void);
void update_enable_state(void);

#endif /* INC_MYSM_H_ */
