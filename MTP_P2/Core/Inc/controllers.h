/*
 * controllers.h
 *
 *  Created on: Apr 15, 2021
 *      Author: gkaretka
 */

#ifndef INC_CONTROLLERS_H_
#define INC_CONTROLLERS_H_

typedef struct
{
	float Ts;

	float c0;
	float c1;
	float tau;
} filter_lp_inst_t;

typedef struct
{
	float Ts;

	float gain;
	float GainTs;
} integrator_inst_t;


typedef struct
{
	float Ts;

	float Kp;			// proporcne zosilnenie (analog aj digital)
	float Ti;			// integracne zosilnenie (analogova, treba prepocitat na diskretnu)
	float TsTi;			// integracne zosilnenie (prepocitane na diskretne, Ts / Ti)
	float ui_1;			// predosla vzorka integratora (pamat z^-1)

	float h_lim;
	float l_lim;
} regPI_inst_t;

void filter_lp(float in, float *out, filter_lp_inst_t *filter_instance);
void filter_lp_init(filter_lp_inst_t *filter_instance);

void integrator_gain_init(integrator_inst_t *integrator_instance);
void integrator_gain(float in, float *out, integrator_inst_t *integrator_instance);

void regPI_p_init(regPI_inst_t *params);
void regPI_p(float in, float *out, regPI_inst_t *params);

#endif /* INC_CONTROLLERS_H_ */
