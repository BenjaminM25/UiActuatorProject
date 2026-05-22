/*
 * HallSensor.h
 *
 *  Created on: 20. apr. 2026
 *      Author: Benjamin
 */

#ifndef DRV8316_SENSORS_HALLSENSOR_H_
#define DRV8316_SENSORS_HALLSENSOR_H_

#include "main.h"
#include "tim.h"
#include <stdint.h>


#ifdef __cplusplus


class HallSensor
{
public:
    /**
     * @param  u_port, u_pin  GPIO port/pin for Hall U
     * @param  v_port, v_pin  GPIO port/pin for Hall V
     * @param  w_port, w_pin  GPIO port/pin for Hall W
     * @param  polePairs      Motor pole pairs
     */
    HallSensor(GPIO_TypeDef *u_port, uint16_t u_pin,
               GPIO_TypeDef *v_port, uint16_t v_pin,
               GPIO_TypeDef *w_port, uint16_t w_pin,
               uint8_t polePairs);

    /** Start the timer and sample initial state. Call once after construction. */
    void begin();

    /** Call from the EXTI callback for every Hall pin edge. */
    void onEdge(uint16_t GPIO_Pin);

    /** Raw encoded Hall state (1..6, or 0 if not yet sampled). */
    uint8_t getState() const      { return m_state; }

    /** Commutation sector (0..5) mapped from the forward sequence. */
    uint8_t getSector() const     { return m_sector; }

    /** +1 forward, -1 reverse, 0 unknown. */
    int8_t  getDirection() const  { return m_direction; }

    /** Motor shaft RPM (signed). Returns 0 if stalled. */
    float   getRpm();

    /** Coarse rotor position from Hall edges.
     *  One unit per 60° electrical. Positive = forward. */
    int32_t getEdgeCount() const  { return m_edgeCount; }

private:
    uint8_t readEncoded();

    /* Hardware config */
    GPIO_TypeDef *m_uPort;  uint16_t m_uPin;
    GPIO_TypeDef *m_vPort;  uint16_t m_vPin;
    GPIO_TypeDef *m_wPort;  uint16_t m_wPin;
    uint8_t       m_polePairs;

    /* State — volatile because written in ISR, read in main loop */
    volatile uint8_t  m_state;
    volatile uint8_t  m_sector;
    volatile int8_t   m_direction;
    volatile int32_t  m_edgeCount;
    volatile uint32_t m_lastTick;
    volatile uint32_t m_lastTickMs;
    volatile float    m_rpm;
};

#endif /* __cplusplus */


/* C-visible interface — safe to include from C files */
#ifdef __cplusplus
extern "C" {
#endif

void HallSensor_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif


#endif /* DRV8316_SENSORS_HALLSENSOR_H_ */
