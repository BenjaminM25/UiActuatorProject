/*
 * encoder.h
 *
 *  Created on: 7. mai 2026
 *      Author: Benjamin
 */


/*  AS5600L magnetic encoder with wrap-aware rotation counting.
 *  Sample fast enough that the shaft never moves more than ½ turn
 *  between Update() calls (recommended period: 1–5 ms).
 *
 *  Velocity estimation:
 *    update() finite-differences the position against the previous sample
 *    using HAL_GetTick() to determine dt. A first-order IIR low-pass filter
 *    smooths the result. Use setVelocityFilterHz() to tune the cutoff;
 *    higher = more responsive but noisier, lower = smoother but laggier.
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

class Encoder
{
public:
    /* AS5600L I2C address (7-bit 0x40, shifted for HAL) */
    static constexpr uint16_t I2C_ADDR        = (0x40 << 1);
    static constexpr uint8_t  REG_STATUS      = 0x0B;
    static constexpr uint8_t  REG_RAW_ANGLE_H = 0x0C;
    static constexpr uint8_t  MD_BIT          = (1 << 5);
    static constexpr uint32_t I2C_TIMEOUT_MS  = 10;

    /* Counts per revolution (12-bit) */
    static constexpr uint16_t CPR             = 4096;

    /**
     * @param hi2c          I2C handle the AS5600L is wired to
     * @param gearRatio     Gear ratio between encoder shaft and output
     *                      shaft (e.g. 24.0f for the 1:24 unit). Pass 1.0f
     *                      if the encoder is on the output shaft directly.
     */
    Encoder(I2C_HandleTypeDef *hi2c, float gearRatio = 1.0f);

    /** Read initial angle, zero rotation count. Call once after MX init. */
    bool begin();

    /** Sample the angle and update the wrap-aware rotation counter.
     *  Also updates the filtered velocity estimate.
     *  @return true if the I2C read succeeded. */
    bool update(float dt_s);
    bool update();

    /** Zero the rotation count and re-baseline the last angle.
     *  Also resets the velocity estimate to zero. */
    void resetCount();

    /* ── Position accessors ─────────────────────────────────── */
    int32_t  getRotations()    const { return m_rotations; }
    uint16_t getRawAngle()     const { return m_currentAngle; }
    bool     isMagnetOK()      const { return m_magnetOK; }
    float    getGearRatio()    const { return m_gearRatio; }

    /** Position as fractional encoder-shaft revolutions (rot + frac). */
    float getPositionRev() const
    {
        return -((float)m_rotations + (float)m_currentAngle / (float)CPR);
    }

    /** Accumulated angle in degrees on the encoder shaft (continues
     *  past 360°). */
    float getPositionDeg() const
    {
        return getPositionRev() * 360.0f;
    }

    /** Position in revolutions of the gearbox output shaft. */
    float getOutputRev() const
    {
        return (m_gearRatio != 0.0f) ? (getPositionRev() / m_gearRatio)
                                     : getPositionRev();
    }

    /* ── Velocity accessors ─────────────────────────────────── */

    /** Raw (unfiltered) encoder-shaft velocity in revolutions per second.
     *  Noisy at low speeds due to quantization; use the filtered version
     *  for control or display. */
    float getVelocityRevPerSec() const { return m_velRaw; }

    /** Filtered encoder-shaft velocity in revolutions per second.
     *  This is what you should use for control loops and plots. */
    float getVelocityRevPerSecFiltered() const { return m_velFiltered; }

    /** Filtered encoder-shaft RPM (revolutions per minute). */
    float getRpm() const { return m_velFiltered * 60.0f; }

    /** Filtered OUTPUT-shaft RPM (after gear reduction).
     *  This is what you typically want to display for an actuator. */
    float getRpmOutput() const
    {
        return (m_gearRatio != 0.0f) ? (m_velFiltered * 60.0f / m_gearRatio)
                                     : (m_velFiltered * 60.0f);
    }

    /** Filtered velocity in rad/s on the encoder shaft. */
    float getVelocityRadPerSec() const
    {
        return m_velFiltered * 6.28318530718f;
    }

    /* ── Velocity filter configuration ──────────────────────── */

    /** Set the low-pass filter cutoff frequency for the velocity estimate.
     *  Typical values:
     *    5–10  Hz : smooth display values, lots of lag
     *    20–50 Hz : reasonable for slow control loops
     *    > 100 Hz : minimal filtering, very noisy at low speeds
     *  Pass a very large value (e.g. 10000) to effectively disable. */
    void setVelocityFilterHz(float fc_hz);

private:
    /* Half-range wrap threshold (2048 counts) */
    static constexpr uint16_t WRAP_THRESHOLD = CPR / 2;

    /* Low-level I2C helpers */
    bool    readAngle(uint16_t &out);
    uint8_t readStatus();

    /* Hardware */
    I2C_HandleTypeDef *m_hi2c;
    float              m_gearRatio;

    /* Position state */
    int32_t  m_rotations;
    uint16_t m_lastAngle;
    uint16_t m_currentAngle;
    bool     m_initialised;
    bool     m_magnetOK;

    /* Velocity state */
    float    m_velRaw;          /* last raw dPos/dt   [rev/s] */
    float    m_velFiltered;     /* low-pass output    [rev/s] */
    float    m_velFilterFc;     /* filter cutoff Hz */
    float    m_lastPositionRev; /* position at previous successful update */
    uint32_t m_lastUpdateTick;  /* HAL_GetTick() at previous update */
    bool     m_velPrimed;       /* false until we have one valid baseline */
};

#endif /* __cplusplus */


/* ────────────────────────────────────────────────────────────
 *  Optional C-visible diagnostic shim (matches the old
 *  TESTENCODER_Update behavior — keep for compatibility with
 *  any C code still calling it).
 * ──────────────────────────────────────────────────────────── */
#ifdef __cplusplus
extern "C" {
#endif

void TESTENCODER_Update(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif



#endif /* INC_ENCODER_H_ */
