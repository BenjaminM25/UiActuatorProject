/*
 * encoder.cpp
 *
 *  Created on: 7. mai 2026
 *      Author: Benjamin
 */

#include <Encoder.h>
#include "usbd_cdc_if.h"
#include <stdio.h>

/* =========================================================================
 * Construction / init
 * ========================================================================= */

Encoder::Encoder(I2C_HandleTypeDef *hi2c, float gearRatio)
    : m_hi2c(hi2c),
      m_gearRatio(gearRatio),
      m_rotations(0),
      m_lastAngle(0),
      m_currentAngle(0),
      m_initialised(false),
      m_magnetOK(false),
      m_velRaw(0.0f),
      m_velFiltered(0.0f),
      m_velFilterFc(20.0f),       /* default: 20 Hz cutoff (smooth display) */
      m_lastPositionRev(0.0f),
      m_lastUpdateTick(0),
      m_velPrimed(false)
{
}

bool Encoder::begin()
{
    m_rotations = 0;

    uint16_t angle;
    if (readAngle(angle))
    {
        m_lastAngle    = angle;
        m_currentAngle = angle;
        m_initialised  = true;
    }
    else
    {
        m_initialised = false;
    }

    uint8_t status = readStatus();
    m_magnetOK = (status != 0xFF) && ((status & MD_BIT) != 0);

    /* Velocity state — wait for the first update() to prime */
    m_velRaw          = 0.0f;
    m_velFiltered     = 0.0f;
    m_lastPositionRev = getPositionRev();
    m_lastUpdateTick  = HAL_GetTick();
    m_velPrimed       = false;

    return m_initialised;
}

/* =========================================================================
 * Per-loop update
 * ========================================================================= */

bool Encoder::update(float dt_s)
{
    uint16_t angle;
    if (!readAngle(angle)) return false;

    /* Wrap detection — unchanged */
    if (!m_initialised) {
        m_lastAngle = angle;
        m_currentAngle = angle;
        m_initialised = true;
    } else {
        int32_t delta = (int32_t)angle - (int32_t)m_lastAngle;
        if      (delta >  WRAP_THRESHOLD) m_rotations--;
        else if (delta < -WRAP_THRESHOLD) m_rotations++;
        m_lastAngle = angle;
        m_currentAngle = angle;
    }

    /* Velocity with caller-supplied dt — no HAL_GetTick() at all */
    float pos_rev = getPositionRev();

    if (!m_velPrimed) {
        m_lastPositionRev = pos_rev;
        m_velRaw = 0.0f;
        m_velFiltered = 0.0f;
        m_velPrimed = true;
    } else if (dt_s > 0.0f) {
        m_velRaw = (pos_rev - m_lastPositionRev) / dt_s;

        float omega = 6.28318530718f * m_velFilterFc;
        float a = (omega * dt_s) / (1.0f + omega * dt_s);
        if (a < 0.0f) a = 0.0f;
        if (a > 1.0f) a = 1.0f;

        m_velFiltered = (1.0f - a) * m_velFiltered + a * m_velRaw;
        m_lastPositionRev = pos_rev;
    }
    return true;
}


/* Keep the old one as a thin wrapper for non-time-critical callers */
bool Encoder::update()
{
    uint32_t now = HAL_GetTick();
    float dt_s = (m_velPrimed && now > m_lastUpdateTick)
                   ? (float)(now - m_lastUpdateTick) * 0.001f
                   : 0.001f;
    m_lastUpdateTick = now;
    return update(dt_s);
}

/* =========================================================================
 * Reset
 * ========================================================================= */

void Encoder::resetCount()
{
    m_rotations = 0;

    /* Re-baseline so the next update() doesn't see a phantom wrap */
    uint16_t angle;
    if (readAngle(angle))
    {
        m_lastAngle    = angle;
        m_currentAngle = angle;
        m_initialised  = true;
    }

    /* Reset velocity state too — otherwise the position jump from
     * resetCount() would produce a huge velocity spike. */
    m_velRaw          = 0.0f;
    m_velFiltered     = 0.0f;
    m_lastPositionRev = getPositionRev();
    m_lastUpdateTick  = HAL_GetTick();
    m_velPrimed       = true;     /* baseline is now valid */
}

/* =========================================================================
 * Velocity filter configuration
 * ========================================================================= */

void Encoder::setVelocityFilterHz(float fc_hz)
{
    if (fc_hz < 0.1f)  fc_hz = 0.1f;     /* avoid division-by-zero issues */
    m_velFilterFc = fc_hz;
}

/* =========================================================================
 * Low-level I2C
 * ========================================================================= */

bool Encoder::readAngle(uint16_t &out)
{
    uint8_t reg    = REG_RAW_ANGLE_H;
    uint8_t buf[2] = {0, 0};

    if (HAL_I2C_Master_Transmit(m_hi2c, I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK)
        return false;
    if (HAL_I2C_Master_Receive(m_hi2c, I2C_ADDR, buf, 2, I2C_TIMEOUT_MS) != HAL_OK)
        return false;

    out = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return true;
}

uint8_t Encoder::readStatus()
{
    uint8_t reg = REG_STATUS;
    uint8_t val = 0xFF;

    if (HAL_I2C_Master_Transmit(m_hi2c, I2C_ADDR, &reg, 1, I2C_TIMEOUT_MS) != HAL_OK)
        return 0xFF;
    if (HAL_I2C_Master_Receive(m_hi2c, I2C_ADDR, &val, 1, I2C_TIMEOUT_MS) != HAL_OK)
        return 0xFF;

    return val;
}

/* =========================================================================
 * Backward-compatible C diagnostic
 * ========================================================================= */

extern "C" void TESTENCODER_Update(I2C_HandleTypeDef *hi2c)
{
    /* Static instance so repeated calls accumulate rotation count.
     * Default gear ratio 1.0f — change here if you want gearbox-aware
     * output from the diagnostic. */
    static Encoder s_diagEnc(hi2c, 1.0f);
    static bool    s_started = false;

    if (!s_started)
    {
        s_diagEnc.begin();
        s_started = true;
    }

    bool ok = s_diagEnc.update();

    char msg[80];
    int  len;

    if (!ok)
    {
        len = snprintf(msg, sizeof(msg), "I2C ERROR - bus may be locked up\r\n");
    }
    else
    {
        float deg = s_diagEnc.getRawAngle() * (360.0f / 4096.0f);
        len = snprintf(msg, sizeof(msg),
                       "raw=%4u deg=%7.2f rot=%ld magOK=%c\r\n",
                       s_diagEnc.getRawAngle(),
                       deg,
                       (long)s_diagEnc.getRotations(),
                       s_diagEnc.isMagnetOK() ? '1' : '0');
    }

    for (int i = 0; i < 3; i++)
    {
        if (CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len) == USBD_OK)
            break;
        HAL_Delay(1);
    }
}
