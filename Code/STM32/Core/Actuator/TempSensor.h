/*
 * TempSensor.h
 *
 *  Created on: 14. mai 2026
 *      Author: Benjamin
 *
 *  Non-blocking STM32G4 internal junction-temperature sensor.
 *  Wraps ADC5 with VREFINT compensation and factory two-point
 *  calibration (TS_CAL1 @ 30 °C, TS_CAL2 @ 110 °C, VDDA = 3.0 V).
 *
 *  Designed for periodic background sampling during lifetime tests:
 *  call update() as often as you like from the main loop — it self-
 *  paces internally and never blocks waiting for the ADC.
 */

#ifndef INC_TEMPSENSOR_H_
#define INC_TEMPSENSOR_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

class TempSensor
{
public:

    /* If update() doesn't see a fresh conversion within this many ms,
     * isStale() returns true. */
    static constexpr uint32_t STALE_TIMEOUT_MS = 5000;

    /**
     * @param hadc       Pointer to the ADC HAL handle (ADC5 in our case).
     *                   Rank 1 must be Temperature Sensor, Rank 2 Vrefint.
     * @param periodMs   Minimum time between samples (default 1000 ms).
     */
    TempSensor(ADC_HandleTypeDef *hadc, uint32_t periodMs = 1000);

    /** One-time setup: calibrate ADC, do a throwaway conversion to wake
     *  the TS buffer, latch factory calibration constants. */
    bool begin();

    /** Drive the state machine. Call as often as you like — it returns
     *  immediately if it isn't time to sample, or if a conversion is still
     *  in flight. Returns true when a fresh result was just captured. */
    bool update();

    /** Change sample period at runtime (e.g. tighten during a hot phase). */
    void setSampleInterval(uint32_t periodMs) { m_periodMs = periodMs; }

    /* ── Accessors ──────────────────────────────────────────── */
    float    getCelsius()      const { return m_tempC; }
    uint16_t getTsRaw()        const { return m_tsRaw; }
    uint16_t getVrefRaw()      const { return m_vrefRaw; }
    float    getVddaMv()       const { return m_vddaMv; }   /* live VDDA in mV */
    uint32_t getSampleCount()  const { return m_sampleCount; }
    float    getMinCelsius()   const { return m_tempMin; }
    float    getMaxCelsius()   const { return m_tempMax; }
    uint32_t getLastSampleMs() const { return m_lastSampleMs; }

    bool isStale() const
    {
        return (HAL_GetTick() - m_lastSampleMs) > STALE_TIMEOUT_MS;
    }

    /** Reset min/max tracking (e.g. at the start of a test phase). */
    void resetMinMax();

    /* Factory calibration values, exposed for plotting/diagnostics. */
    uint16_t getTsCal1()  const { return m_tsCal1; }
    uint16_t getTsCal2()  const { return m_tsCal2; }
    uint16_t getVrefCal() const { return m_vrefCal; }

private:
    enum class State : uint8_t
    {
        Idle,        /* waiting for next sample tick */
        Converting   /* HAL_ADC_Start issued, waiting for both ranks */
    };

    /* Hardware */
    ADC_HandleTypeDef *m_hadc;

    /* Config */
    uint32_t m_periodMs;

    /* Factory calibration (latched in begin()) */
    uint16_t m_tsCal1;
    uint16_t m_tsCal2;
    uint16_t m_vrefCal;
    float    m_slopeInv;     /* (T2 - T1) / (CAL2 - CAL1), precomputed */

    /* State machine */
    State    m_state;
    uint32_t m_convStartMs;
    uint32_t m_lastSampleMs;
    uint32_t m_nextSampleMs;

    /* Latest results */
    uint16_t m_tsRaw;
    uint16_t m_vrefRaw;
    float    m_vddaMv;
    float    m_tempC;

    /* Lifetime tracking */
    uint32_t m_sampleCount;
    float    m_tempMin;
    float    m_tempMax;
    bool     m_haveSample;
};

#endif /* __cplusplus */

#endif /* INC_TEMPSENSOR_H_ */
