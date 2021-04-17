/*
 * controllers.c
 *
 *  Created on: Apr 15, 2021
 *      Author: gkaretka
 */

#include "controllers.h"

/*
 * Low pass filter
 */

void filter_lp(float in, float *out, filter_lp_inst_t *filter_instance)
{
	*out = ((*out) * filter_instance->c1) + (in * filter_instance->c0);
	return;
}

void filter_lp_init(filter_lp_inst_t *filter_instance)
{
	filter_instance->c0 = filter_instance->Ts / (filter_instance->Ts + filter_instance->tau);
	filter_instance->c1 = 1.0f - filter_instance->c0;
	return;
}


/*
 * Integrator
 */

void integrator_gain(float in, float *out, integrator_inst_t *integrator_instance)
{
	*out = in * integrator_instance->GainTs + (*out);
	return;
}

void integrator_gain_init(integrator_inst_t *integrator_instance)
{
	integrator_instance->GainTs = integrator_instance->Ts * integrator_instance->gain;
	return;
}


/*
 * Regulator PI
 */

void regPI_p(float in, float *out, regPI_inst_t *params)
{
	float up, ui, u;

	up = in * params->Kp;
	ui = in * params->TsTi + params->ui_1;

	u = up + ui;

	if (u > params->h_lim) {
		u = params->h_lim;
	} else if (u < params->l_lim) {
		u = params->l_lim;
	} else {
		params->ui_1 = ui;
	}

	*out = u;

	return;
}

void regPI_p_init(regPI_inst_t *params)
{
	params->TsTi = params->Ts / params->Ti;
	return;
}
