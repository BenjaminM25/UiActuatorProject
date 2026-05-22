/*
 * TestADC.c
 *
 *  Created on: 20. apr. 2026
 *      Author: Benjamin
 */

/**
 * @file    TestADC.c
 * @brief   ADC1 injected current-sensing — raw value printout.
 *
 * Hardware:
 *   ADC1 IN1 / IN2 / IN3  →  PA0 / PA1 / PA2  (DRV8316 SOA/SOB/SOC)
 *   Trigger source         →  Software start
 *   Conversion group       →  Injected, 3 ranks
 *
 * Usage:
 *   1. Call MotorApp_Init() first
 *   2. Call TestADC_Init()
 *   3. Call TestADC_Run() from main while(1)
 */

#include "TestADC.h"
#include "adc.h"
#include "main.h"
#include "plot.h"
#include "tim.h"
#include "MotorApp.h"

volatile uint16_t adc_raw[3] = {0, 0, 0};

/* --------------------------------------------------------------------------
 * TestADC_Init
 * -------------------------------------------------------------------------- */
void TestADC_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
    	Error_Handler();
    }

}

/* --------------------------------------------------------------------------
 * TestADC_Run — call from main while(1) loop.
 * -------------------------------------------------------------------------- */
void TestADC_Run(void)
{
    static uint32_t last_print_ms = 0;
    static float current_sum = 0.0f;
    static float current_inst = 0.0f;
    static uint32_t current_count = 0;
    uint32_t now = HAL_GetTick();

    /* Accumulate every call */
    int8_t phase = MotorApp_GetActiveLowPhase();
    if (phase >= 0)
    {
        float current = ((float)adc_raw[phase] - 2044.0f)
                        / 4095.0f * 3.3f / 0.6f;
        current_inst = current < 0.0f ? -current : current;
        current_sum += current_inst;
        current_count++;
    }

    if (now - last_print_ms < 100)
        return;

    last_print_ms = now;

    uint16_t raw_a = adc_raw[0];
    uint16_t raw_b = adc_raw[1];
    uint16_t raw_c = adc_raw[2];

//    Plot_Int("raw_a", (int32_t)raw_a);
//    Plot_Int("raw_b", (int32_t)raw_b);
//    Plot_Int("raw_c", (int32_t)raw_c);

    if (current_count > 0)
    {
        float avg_current = current_sum / current_count;
        Plot_Float("motor_current",      avg_current);
//        Plot_Float("motor_current_raw",  current_sum);
//        Plot_Float("motor_current_inst", current_inst);
//        Plot_Int  ("current_samples",    (int32_t)current_count);


    }

    current_sum = 0.0f;
    current_count = 0;
}

volatile uint32_t adc_cb_count = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_raw[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        adc_raw[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        adc_raw[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
        adc_cb_count++;
    }
}
