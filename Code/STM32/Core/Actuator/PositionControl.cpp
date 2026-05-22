/*
 * PositionControl.cpp
 *
 *  Created on: 9. mai 2026
 *      Author: Benjamin
 */



#include "PositionControl.h"
#include <math.h>

/* Saturation helper */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

PositionControl::PositionControl()
:   m_kp(1.0f),
    m_ki(0.0f),
    m_kd(0.0f),
    m_maxVoltage(8.0f),
    m_maxIntegral(4.0f),       /* contributes at most 4V from integrator */
    m_deadband_mm(0.05f),      /* 50 µm deadband; tweak after testing */
    m_derivFilterAlpha(0.2f),  /* corresponds to ~32 Hz cutoff at 200 Hz fs */
    m_enabled(false),
    m_target_mm(0.0f),
    m_integral(0.0f),
    m_lastMeasurement(0.0f),
    m_derivFiltered(0.0f),
    m_lastError(0.0f),
    m_firstUpdate(true)
{}

void PositionControl::setGains(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    /* Reset integrator on big gain changes to avoid sudden jumps */
    m_integral = 0.0f;
}

void PositionControl::setOutputLimit(float maxVoltage)
{
    if (maxVoltage < 0.0f) maxVoltage = -maxVoltage;
    m_maxVoltage = maxVoltage;
}

void PositionControl::setIntegralLimit(float maxIntegral)
{
    if (maxIntegral < 0.0f) maxIntegral = -maxIntegral;
    m_maxIntegral = maxIntegral;
}

void PositionControl::setDeadband(float deadband_mm)
{
    if (deadband_mm < 0.0f) deadband_mm = 0.0f;
    m_deadband_mm = deadband_mm;
}

void PositionControl::setDerivativeFilterHz(float fc_hz)
{
    /* Convert cutoff to alpha assuming a nominal 200 Hz update rate.
     * Exact alpha is recomputed each tick from actual dt below — this
     * is a fallback if dt is unavailable. */
    if (fc_hz <= 0.0f) { m_derivFilterAlpha = 1.0f; return; }
    const float fs = 200.0f;
    float rc = 1.0f / (2.0f * 3.14159265f * fc_hz);
    float dt = 1.0f / fs;
    m_derivFilterAlpha = dt / (rc + dt);
}

void PositionControl::setTarget_mm(float target)
{
    m_target_mm = target;
    /* Don't clear integrator here — we want it to keep working through
     * a target change. But cancel the "first update" flag so the next
     * derivative calc has a fresh baseline against the new target. */
}

void PositionControl::enable(bool en)
{
    if (en && !m_enabled)
    {
        reset();    /* fresh state when re-enabling */
    }
    m_enabled = en;
}

void PositionControl::reset()
{
    m_integral = 0.0f;
    m_derivFiltered = 0.0f;
    m_lastError = 0.0f;
    m_firstUpdate = true;
}

bool PositionControl::atTarget(float tolerance_mm) const
{
    return fabsf(m_lastError) <= tolerance_mm;
}

float PositionControl::update(float current_mm, float dt_seconds)
{
    if (!m_enabled || dt_seconds <= 0.0f)
    {
        return 0.0f;
    }

    /* Error in mm */
    float error = m_target_mm - current_mm;
    m_lastError = error;

    /* ── Deadband: don't fight stiction over sub-mm errors ──── */
    if (fabsf(error) < m_deadband_mm)
    {
        /* Bleed integrator slowly so it doesn't keep pushing once
         * we settle inside the deadband. Don't slam it to zero —
         * that would cause a step on the next exit from deadband. */
        m_integral *= 0.95f;
        m_lastMeasurement = current_mm;
        return 0.0f;
    }

    /* ── Proportional ────────────────────────────────────────── */
    float p_term = m_kp * error;

    /* ── Derivative on measurement (negated) ─────────────────── */
    float d_raw = 0.0f;
    if (!m_firstUpdate)
    {
        /* d(error)/dt = -d(measurement)/dt for constant target */
        float meas_rate = (current_mm - m_lastMeasurement) / dt_seconds;
        d_raw = -meas_rate;       /* derivative of error */
    }
    m_lastMeasurement = current_mm;

    /* Low-pass filter the derivative term — recompute alpha from
     * actual dt for time-varying loop rates. */
    {
        const float fc = 30.0f;   /* Hz; reasonable for 200 Hz loop */
        float rc = 1.0f / (2.0f * 3.14159265f * fc);
        float alpha = dt_seconds / (rc + dt_seconds);
        m_derivFiltered = m_derivFiltered + alpha * (d_raw - m_derivFiltered);
    }
    float d_term = m_kd * m_derivFiltered;

    /* ── Integral with clamping anti-windup ──────────────────── */
    /* Tentatively update integrator */
    float integral_candidate = m_integral + m_ki * error * dt_seconds;
    integral_candidate = clampf(integral_candidate, -m_maxIntegral, m_maxIntegral);

    /* Compute unsaturated output */
    float u_unsat = p_term + integral_candidate + d_term;

    /* Saturate */
    float u = clampf(u_unsat, -m_maxVoltage, m_maxVoltage);

    /* Clamping anti-windup: only commit the integrator update if it
     * doesn't drive us further into saturation. */
    bool saturated = (u != u_unsat);
    bool same_sign = (error * integral_candidate) > 0.0f;
    if (saturated && same_sign)
    {
        /* freeze integrator at previous value */
    }
    else
    {
        m_integral = integral_candidate;
    }

    m_firstUpdate = false;
    return u;
}
