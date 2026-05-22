/*
 * TempSensor.cpp
 *
 *  Created on: 14. mai 2026
 *      Author: Benjamin
 */

#include "TempSensor.h"

TempSensor::TempSensor(ADC_HandleTypeDef *hadc, uint32_t periodMs)
    : m_hadc(hadc),
      m_periodMs(periodMs),
      m_tsCal1(0), m_tsCal2(0), m_vrefCal(0), m_slopeInv(0.0f),
      m_state(State::Idle),
      m_convStartMs(0), m_lastSampleMs(0), m_nextSampleMs(0),
      m_tsRaw(0), m_vrefRaw(0), m_vddaMv(TEMPSENSOR_CAL_VREFANALOG), m_tempC(0.0f),
      m_sampleCount(0), m_tempMin(0.0f), m_tempMax(0.0f),
      m_haveSample(false)
{
}

bool TempSensor::begin()
{
    /* Latch factory calibration */
	m_tsCal1  = *TEMPSENSOR_CAL1_ADDR;
	m_tsCal2  = *TEMPSENSOR_CAL2_ADDR;
	m_vrefCal = *VREFINT_CAL_ADDR;

    /* Precompute °C-per-LSB so the per-sample math is cheap.
     * Guard against zero in case the cal region is unreadable. */
    int32_t span = (int32_t)m_tsCal2 - (int32_t)m_tsCal1;
    m_slopeInv = (span != 0) ? (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (float)span
                             : 0.0f;

    /* ADC self-calibration — required on G4 before first use */
    if (HAL_ADCEx_Calibration_Start(m_hadc, ADC_SINGLE_ENDED) != HAL_OK)
        return false;

    /* Throwaway conversion to wake the temperature-sensor buffer
     * (~120 µs startup per DS Table 79). We do this blocking *once*,
     * during init only — never in the update path. */
    if (HAL_ADC_Start(m_hadc) != HAL_OK)
        return false;
    HAL_ADC_PollForConversion(m_hadc, 10);
    (void)HAL_ADC_GetValue(m_hadc);
    HAL_ADC_PollForConversion(m_hadc, 10);
    (void)HAL_ADC_GetValue(m_hadc);
    HAL_ADC_Stop(m_hadc);

    m_state = State::Idle;
    m_nextSampleMs = HAL_GetTick();   /* sample immediately on first update() */
    return (m_slopeInv != 0.0f);
}

bool TempSensor::update()
{
    uint32_t now = HAL_GetTick();

    switch (m_state)
    {
        case State::Idle:
        {
            if ((int32_t)(now - m_nextSampleMs) < 0)
                return false;   /* not time yet */

            if (HAL_ADC_Start(m_hadc) != HAL_OK)
            {
                /* Reschedule and bail — don't get stuck. */
                m_nextSampleMs = now + m_periodMs;
                return false;
            }
            m_convStartMs = now;
            m_state = State::Converting;
            return false;
        }

        case State::Converting:
        {
        	/* Read rank 1 (Vrefint). */
            if (HAL_ADC_PollForConversion(m_hadc, 1) != HAL_OK)
            {
				HAL_ADC_Stop(m_hadc);
				m_state = State::Idle;
				m_nextSampleMs = now + m_periodMs;
                return false;
            }
            uint32_t vref = HAL_ADC_GetValue(m_hadc);

            /* Read rank 2 (temperature sensor). */
            if (HAL_ADC_PollForConversion(m_hadc, 1) != HAL_OK)
                return false;   /* second rank not ready yet — try next call */
            uint32_t ts = HAL_ADC_GetValue(m_hadc);

            HAL_ADC_Stop(m_hadc);

            m_tsRaw   = (uint16_t)ts;
            m_vrefRaw = (uint16_t)vref;

            /* Live VDDA from VREFINT: VDDA = 3.0V * VREFINT_CAL / VREFINT_meas */
            if (m_vrefRaw != 0)
                m_vddaMv = VREFINT_CAL_VREF * (float)m_vrefCal / (float)m_vrefRaw;

            /* Scale TS reading to what it *would* have been at VDDA = 3.0 V */
            float tsScaled = (float)m_tsRaw * m_vddaMv / VREFINT_CAL_VREF;

            /* Two-point linear: T = (tsScaled - CAL1) * (T2 - T1)/(CAL2 - CAL1) + T1 */
            m_tempC = (tsScaled - (float)m_tsCal1) * m_slopeInv + TEMPSENSOR_CAL1_TEMP;

            /* Lifetime tracking */
            if (!m_haveSample)
            {
                m_tempMin = m_tempC;
                m_tempMax = m_tempC;
                m_haveSample = true;
            }
            else
            {
                if (m_tempC < m_tempMin) m_tempMin = m_tempC;
                if (m_tempC > m_tempMax) m_tempMax = m_tempC;
            }
            m_sampleCount++;
            m_lastSampleMs = now;
            m_nextSampleMs = now + m_periodMs;

            m_state = State::Idle;
            return true;
        }
    }
    return false;
}

void TempSensor::resetMinMax()
{
    m_tempMin = m_tempC;
    m_tempMax = m_tempC;
}
