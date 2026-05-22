/*
 * PositionControl.h
 *
 *  Created on: 9. mai 2026
 *      Author: Benjamin
 */


/*  PID position controller. Operates in millimetres of carriage travel.
 *  The output is a SIGNED voltage command; the caller is responsible for
 *  splitting it into magnitude (motor.setVoltage) and direction (sign fed
 *  into the hall ISR commutation).
 *
 *  Features:
 *    - Output saturation
 *    - Integral anti-windup (clamping method)
 *    - Derivative on measurement (no derivative kick on setpoint changes)
 *    - First-order low-pass on the derivative term
 *    - Deadband around target (prevents buzzing on backlash/stiction)
 */

#ifndef INC_POSITIONCONTROL_H_
#define INC_POSITIONCONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus

class PositionControl
{
public:
    PositionControl();

    /* ── Configuration ───────────────────────────────────────── */
    void setGains(float kp, float ki, float kd);
    void setOutputLimit(float maxVoltage);          /* symmetric ± */
    void setIntegralLimit(float maxIntegral);       /* anti-windup clamp */
    void setDeadband(float deadband_mm);            /* zero output below this error */
    void setDerivativeFilterHz(float fc_hz);        /* LPF cutoff for D-term */

    /* ── Runtime control ─────────────────────────────────────── */
    void setTarget_mm(float target);
    float getTarget_mm() const { return m_target_mm; }

    void enable(bool en);                           /* false → resets state */
    bool isEnabled() const { return m_enabled; }

    void reset();                                   /* clear integrator/derivative state */

    /* Run one tick.
     *   current_mm  : measured carriage position
     *   dt_seconds  : time since last call
     * Returns signed voltage command. Sign = direction.
     * If disabled or in deadband, returns 0.
     */
    float update(float current_mm, float dt_seconds);

    /* ── Diagnostics ─────────────────────────────────────────── */
    float getError_mm()    const { return m_lastError; }
    float getIntegralTerm() const { return m_integral; }
    float getDerivativeTerm() const { return m_derivFiltered; }
    bool  atTarget(float tolerance_mm) const;

    /* ── Accessors for plotting ──────────────────────────────── */
    float getKp() const { return m_kp; }
    float getKi() const { return m_ki; }
    float getKd() const { return m_kd; }

private:
    /* Gains */
    float m_kp;
    float m_ki;
    float m_kd;

    /* Limits / config */
    float m_maxVoltage;
    float m_maxIntegral;
    float m_deadband_mm;
    float m_derivFilterAlpha;   /* 0..1; higher = faster response, more noise */

    /* State */
    bool  m_enabled;
    float m_target_mm;
    float m_integral;
    float m_lastMeasurement;
    float m_derivFiltered;
    float m_lastError;
    bool  m_firstUpdate;
};

#endif /* __cplusplus */


#endif /* INC_POSITIONCONTROL_H_ */
