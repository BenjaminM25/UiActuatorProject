/*
 * HallSensor.cpp
 *
 *  Created on: 20. apr. 2026
 *      Author: Benjamin
 */



#include "HallSensor.h"

/* ---------- Hard-coded configuration ---------- */
static const uint8_t HALL_FWD_SEQ[6] = {1, 5, 4, 6, 2, 3};

/* Timer used for high-resolution RPM timing. TIM2 = 32-bit @ 170 MHz on STM32G4. */
#define HALL_TIMER          (&htim2)
#define HALL_TIMER_CLOCK_HZ 170000000U

/* No edges within this time (ms) = stalled */
#define STALL_TIMEOUT_MS    500U
/* ---------------------------------------------- */

HallSensor::HallSensor(GPIO_TypeDef *u_port, uint16_t u_pin,
                       GPIO_TypeDef *v_port, uint16_t v_pin,
                       GPIO_TypeDef *w_port, uint16_t w_pin,
                       uint8_t polePairs)
    : m_uPort(u_port), m_uPin(u_pin),
      m_vPort(v_port), m_vPin(v_pin),
      m_wPort(w_port), m_wPin(w_pin),
      m_polePairs(polePairs),
      m_state(0), m_sector(0), m_direction(0),
      m_edgeCount(0), m_lastTick(0), m_lastTickMs(0), m_rpm(0.0f)
{
}

void HallSensor::begin()
{
    HAL_TIM_Base_Start(HALL_TIMER);

    /* Sample initial state so first edge has a valid "previous" value */
    m_state = readEncoded();
    for (uint8_t i = 0; i < 6; i++)
    {
        if (HALL_FWD_SEQ[i] == m_state)
        {
            m_sector = i;
            break;
        }
    }
    m_lastTick   = __HAL_TIM_GET_COUNTER(HALL_TIMER);
    m_lastTickMs = HAL_GetTick();
}

uint8_t HallSensor::readEncoded()
{
    uint8_t u = (HAL_GPIO_ReadPin(m_uPort, m_uPin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t v = (HAL_GPIO_ReadPin(m_vPort, m_vPin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t w = (HAL_GPIO_ReadPin(m_wPort, m_wPin) == GPIO_PIN_SET) ? 1 : 0;
    return (uint8_t)((u << 2) | (v << 1) | w);
}

void HallSensor::onEdge(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != m_uPin && GPIO_Pin != m_vPin && GPIO_Pin != m_wPin)
    {
        return;
    }

    uint8_t newState = readEncoded();

    /* Ignore spurious interrupts and invalid states (0 = all low, 7 = all high) */
    if (newState == m_state || newState == 0 || newState == 7)
    {
        return;
    }

    /* Find the new state's position in the forward sequence */
    uint8_t newSector = m_sector;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (HALL_FWD_SEQ[i] == newState)
        {
            newSector = i;
            break;
        }
    }

    /* Direction: did we step +1 or -1 in the forward sequence? (mod 6) */
    uint8_t fwdNext = (m_sector + 1) % 6;
    uint8_t revNext = (m_sector + 5) % 6;
    if (newSector == fwdNext)
    {
        m_direction = 1;
        m_edgeCount++;
    }
    else if (newSector == revNext)
    {
        m_direction = -1;
        m_edgeCount--;
    }
    /* else: skipped sector (noise or missed edge) — don't update direction */

    /* RPM from timer delta */
    uint32_t now = __HAL_TIM_GET_COUNTER(HALL_TIMER);
    uint32_t dt  = now - m_lastTick;   /* unsigned wraparound is fine */
    if (dt > 0)
    {
        float edgesPerRev = 6.0f * (float)m_polePairs;
        m_rpm = (60.0f * (float)HALL_TIMER_CLOCK_HZ / edgesPerRev) / (float)dt;
    }

    m_state      = newState;
    m_sector     = newSector;
    m_lastTick   = now;
    m_lastTickMs = HAL_GetTick();
}

float HallSensor::getRpm()
{
    if ((HAL_GetTick() - m_lastTickMs) > STALL_TIMEOUT_MS)
    {
        m_rpm = 0.0f;
    }
    return (m_direction >= 0) ? m_rpm : -m_rpm;
}

//float HallSensor::getFilteredRpm()
//{
//
//}

