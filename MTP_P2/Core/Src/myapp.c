/*
 * myapp.c
 *
 *  Created on: Apr 29, 2021
 *      Author: gkaretka
 */

#include "myapp.h"
#include "dataAcq.h"

filter_lp_inst_t filter_1, filter_2;
integrator_inst_t capacitor_1;

regPI_inst_t pireg_current_1;
regPI_inst_t pireg_voltage_1;

#define RLC_Ra	0.5f		// 0.5R
#define RLC_La	0.000470f 	// 470uH
#define RLC_c	0.0091f		// 9100uF (9.1mF)
#define U_Dmax	24.f		// 24 V

adcdata_t adcdata;
uint16_t dac_o;

float volt_des, volt_c, volt_reg_o;
float m_o, k_o;
float pireg_curr_o, cur_des;

void hw_init(void)
{
	// ADC DMA
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)&adcdata);
	LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, 3);
	LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA_MULTI));
	LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);

	// ADC Calib and start
	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC2));

	LL_ADC_Enable(ADC2);

	LL_ADC_REG_StartConversion(ADC2);

	LL_DMA_ClearFlag_TC1(DMA2);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_1);

	// DAC DMA
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&dac_o);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 1);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_3, (uint32_t)&(DAC1->DHR12R1));
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

	// DAC Enable and start
	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);
	LL_DAC_EnableDMAReq(DAC1, LL_DAC_CHANNEL_1);

	// assign sampling period
	float Ts = ((float)TIM1_ARR + 1.f)/(float)SystemCoreClock;
	filter_2.Ts = pireg_voltage_1.Ts = pireg_current_1.Ts = capacitor_1.Ts = filter_1.Ts = Ts;

	// lp filter 1 (nahrada menica)
	filter_1.tau = (3.f*Ts)/4.f;	// seconds
	filter_lp_init(&filter_1);

	// lp filter 2 (nahrada kotvy)
	filter_2.tau = RLC_La/RLC_Ra;	// seconds
	filter_lp_init(&filter_2);

	// capacitor
	capacitor_1.gain = 1 / RLC_c;	// 1 / C
	integrator_gain_init(&capacitor_1);

	float tm = (3.f*Ts)/2.f;
	float ta = RLC_La/RLC_Ra;

	// current pireg
	pireg_current_1.Kp = (RLC_Ra * ta) / (2.f * tm);
	pireg_current_1.Ti = (2.f * tm) / RLC_Ra;
	pireg_current_1.h_lim = U_Dmax;
	pireg_current_1.l_lim = 0.f;
	regPI_p_init(&pireg_current_1);

	float tsig = 5.f * tm;

	// voltage pireg
	pireg_voltage_1.Kp = RLC_c / (2.f * tsig);
	pireg_voltage_1.Ti = RLC_c / (8.f * tsig * tsig);
	pireg_voltage_1.h_lim = 2.0f;
	pireg_voltage_1.l_lim = -2.0f;
	regPI_p_init(&pireg_voltage_1);

	//ClearBuffer();
}

void sw_turnoff(void)
{
	volt_des = 0;
}

// 10 KHz
void loop(void)
{
	// current regulator
	//volt_c = adcdata.samples.adc_m0; // uncomment for using ADC (not working)
	regPI_p(volt_des - volt_c, &volt_reg_o, &pireg_voltage_1);

	cur_des = volt_reg_o;
	// cur_des = 1; // ladenie prudu
	//k_o = adcdata.samples.adc_m1; // uncomment for using ADC (not working)
	regPI_p(cur_des - k_o, &pireg_curr_o, &pireg_current_1);

	// current output to PWM
	TIM1->CCR1 = (uint32_t)((pireg_curr_o * (float)TIM1_ARR) / U_Dmax);

	// nahrada menica
	filter_lp(pireg_curr_o, &m_o, &filter_1);

	// nahrada kotvy
	m_o *= 1/RLC_Ra;
	filter_lp(m_o, &k_o, &filter_2);

	// k_o -> DAC
	dac_o = k_o;

	// Capacitor (integrator)
	integrator_gain(k_o, &volt_c, &capacitor_1);

	//DumpTrace();
}
