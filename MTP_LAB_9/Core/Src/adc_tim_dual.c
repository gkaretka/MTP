#include "adc_tim_dual.h"


ADC12Data_t ADC12Data;

float filterIn, filterOut,
		filterc0, filterc1,
		filterTau = 0.2f, filterTs;

float dacOut;

void hw_init(void)
{
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC12Data);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA_MULTI));
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC1));

	LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
	while(LL_ADC_IsCalibrationOnGoing(ADC2));

	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);

	LL_ADC_REG_StartConversion(ADC1);
	LL_ADC_REG_StartConversion(ADC2);

	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);

	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

	LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_1);

	filterTs = ((float)TIM1_ARR + 1.f)/(float)SystemCoreClock;
}


void main_loop(void)
{
	filterc0 = filterTs / (filterTau + filterTs);
	filterc1 = 1.f - filterc0;

	LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_1, (uint32_t)(dacOut * ((1 << 12)-1)));
}


void main_isr(void)
{
	filterIn = (float)ADC12Data.samples.ch1_V1 * (1.0f / (1 << 12));
	filterOut = filterc0 * filterIn + filterc1 * filterOut;
}
