#include "MotorApp.h"
#include "BLDCDriver3PWM.h"
#include "BLDCDriver6PWM.h"
#include "BLDCMotor.h"
#include "plot.h"
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "HallSensor.h"
#include "usbd_cdc_if.h"
#include "LimitSwitch.h"
#include <stdlib.h>   /* abs() */
#include <string.h>   /* memcpy */
#include <ctype.h>    /* tolower */
#include <Encoder.h>
#include "math.h"
#include "TempSensor.h"
#include "adc.h"
#include "CANBus.h"

/* =========================================================================
 * Configuration
 * ========================================================================= */

#define SUPPLY_VOLTAGE          12.0f
#define FAULT_CHECK_MS          10
#define PLOT_MS                 100
#define ENDSTOP_CHECK_MS        10
#define KICK_INTERVAL_MS        50      /* startup-from-rest kick rate; ISR handles motion */
#define GEAR_RATIO              24.0f

//#define ENCODER_UPDATE_MS       1
#define FILTER_FC_HERTZ         930.0f
//#define FILTER_FC_HERTZ        0.0f
#define Target                  25.0f      /* target position in encoder revs */

/* PID config — TIM6 is the control-loop tick. PID_DT MUST match the
 * actual TIM6 update rate set in CubeMX.
 *   1 kHz tick → PID_DT = 0.001f
 *   500 Hz    → PID_DT = 0.002f
 *   etc. */
#define PID_DT                  0.001f
#define MAX_PID_V               12.0f      /* output saturation, V — keep low for bench */

#define Kp                      56.8859823808594f      /* V per rev — error of 0.4 rev → 2V (saturated) */
#define Ki                      593.026668448812f
#define Kd                      1.03592960739312f

//#define Kp                      0.062f      /* V per rev — error of 0.4 rev → 2V (saturated) */
//#define Ki                      7.2926f
//#define Kd                      0.0f

static float Kff = 0.25761f;   /* V·s/rev — set after identification */
//static float Kff = 0.0f;   /* V·s/rev — set after identification */


#define CYCLE_OUT_POS     25.0f    	  /* rev - 4 * 25 = 100mm stroke*/
#define CYCLE_IN_POS      0.0f
#define CYCLE_VMAX        15.1375f    /* rev/s — your thermal cap */
#define CYCLE_AMAX        18.32627f    /* rev/s² */


/* ---------------- CAN command protocol ---------------- */
#define CAN_ID_GOTO             0x100   /* payload: float32 LE, target rev */
#define CAN_ID_AUTOCYCLE        0x101   /* no payload */
#define CAN_ID_STOP             0x102   /* no payload */
#define CAN_ID_HOMING           0x103   /* no payload */
#define CAN_ID_PING             0x104   /* no payload */

#define CAN_ID_STATUS_REPLY     0x200   /* MCU -> bus, status snapshot */
#define CAN_ID_ACK_REPLY        0x201   /* MCU -> bus, command ack/err */
static void CAN_DispatchCommands(void);
static void CAN_Reply(uint32_t id, const uint8_t *data, uint8_t len);


/* Default normal-operation drive direction (used only as initial value
 * for s_pidDir before the PID has run). +1 forward, -1 reverse. */
#define DIRECTION               (1)

/* Homing: which physical direction drives the carriage TOWARD the limit
 * switch. Flip the sign if your switch is mounted at the other end. */
#define DIR_TOWARD_SWITCH       (-1)
#define DIR_AWAY_FROM_SWITCH    (-(DIR_TOWARD_SWITCH))

/* Velocity-loop tuning */
#define HOMING_TARGET_RPM       7200.0f
#define HOMING_VOLTAGE_MIN      1.6f
#define HOMING_VOLTAGE_MAX      8.0f
#define HOMING_KI               0.05f   /* V per (rpm·s) — tune empirically */
#define HOMING_BACKOFF_EDGES    180      /* small backoff at end (output ~1/4 turn) */
#define HOMING_SETTLE_MS        1000    /* dwell at zero V after switch hit   */
#define HOMING_TICK_MS          2       /* state-machine + commutation period */

/* Stall watchdog */
#define HOMING_STALL_RPM        5.0f    /* below this = "not moving" */
#define HOMING_STALL_TIMEOUT_MS 2000U




#define TEMP_LIMIT_C   55.0f
#define TEMP_RESET_C   50.0f
#define TEMP_SAMPLE_INTERVAL_MS		500U

/* ── Thermal runaway guard ──────────────────────────────────────────
 * Catches a sustained linear temperature rise like the one observed
 * during the failed gearbox-stall test (slope ~1-3 °C/min over 30+ min).
 *
 * Steady-state slopes are well under 0.1 °C/min, so 0.3 °C/min gives
 * a comfortable margin against false trips. Confirming over 3 minutes
 * with a 5-minute slope window further filters out noise from sensor
 * quantization. */
#define THERMAL_SETTLE_MS              (10U * 60U * 1000U)   /* 10 min */
#define THERMAL_SLOPE_LIMIT_C_PER_MIN  0.3f
#define SLOPE_CONFIRM_MS               (3U * 60U * 1000U)    /* 3 min */
#define SLOPE_WINDOW_MS                (5U * 60U * 1000U)    /* 5 min */

#define TEMP_HISTORY_SIZE  60     /* 5 min at 1 Hz + headroom */

static float    s_tempHistory[TEMP_HISTORY_SIZE];
static uint32_t s_tempHistoryTimes[TEMP_HISTORY_SIZE];
static uint16_t s_tempHistoryIdx     = 0;
static uint16_t s_tempHistoryCount   = 0;
static uint32_t s_slopeBreachStartMs = 0;
static bool     s_thermalRunaway     = false;



static float ThermalGuard_ComputeSlope(void)
{
    if (s_tempHistoryCount < 4) return 0.0f;

    uint32_t now = HAL_GetTick();
    float    t_oldest    = 0.0f;
    uint32_t time_oldest = 0;
    bool     found       = false;

    /* Walk backward from newest, find the oldest sample within window */
    for (uint16_t i = 0; i < s_tempHistoryCount; i++) {
        uint16_t idx = (s_tempHistoryIdx + TEMP_HISTORY_SIZE - 1 - i) % TEMP_HISTORY_SIZE;
        if (now - s_tempHistoryTimes[idx] > SLOPE_WINDOW_MS) break;
        t_oldest    = s_tempHistory[idx];
        time_oldest = s_tempHistoryTimes[idx];
        found       = true;
    }

    if (!found) return 0.0f;

    uint16_t newestIdx   = (s_tempHistoryIdx + TEMP_HISTORY_SIZE - 1) % TEMP_HISTORY_SIZE;
    float    t_newest    = s_tempHistory[newestIdx];
    uint32_t time_newest = s_tempHistoryTimes[newestIdx];

    if (time_newest == time_oldest) return 0.0f;

    float dt_min = (float)(time_newest - time_oldest) / 60000.0f;
    return (t_newest - t_oldest) / dt_min;
}

static void ThermalGuard_AddSample(float tempC)
{
    s_tempHistory[s_tempHistoryIdx]      = tempC;
    s_tempHistoryTimes[s_tempHistoryIdx] = HAL_GetTick();
    s_tempHistoryIdx = (s_tempHistoryIdx + 1) % TEMP_HISTORY_SIZE;
    if (s_tempHistoryCount < TEMP_HISTORY_SIZE) s_tempHistoryCount++;
}




#define USB_CMD_BUF_SIZE        128

enum class HomingState : uint8_t {
    Idle, MoveToSwitch, Settle, Backoff, PostBackoffSettle ,Done
};

static volatile HomingState s_homingState   = HomingState::Idle;
static volatile bool        s_homingActive  = false;
static volatile bool        s_motorEnabled  = true;   /* gate for ISR commutation */
static volatile int8_t      s_homingDir     = 0;
static volatile float       s_homingVoltage = 0.0f;
static int32_t              s_backoffStartEdges = 0;
static uint32_t             s_settleDeadline    = 0;

static bool homingCompleted = false;

/* PID drive direction — written by the PID, read by the Hall ISR / kick.
 * Replaces the hardcoded DIRECTION macro for normal-operation drive. */
static volatile int8_t      s_pidDir        = DIRECTION;

/* Set true once the startup hold has expired. Until then the PID stays
 * dormant so it can't fight a not-yet-stable system. */
static volatile bool        s_pidEnabled    = false;

/* USB command line buffer — written from CDC ISR, drained from main loop. */
static volatile uint8_t  s_cmdBuf[USB_CMD_BUF_SIZE];
static volatile uint16_t s_cmdLen      = 0;
static volatile bool     s_cmdReady    = false;
static volatile bool     s_cmdOverflow = false;




/* ── Run mode ────────────────────────────────────────────────────────
 * Set to 1 to run an open-loop voltage step for system ID.
 * Set to 0 to run the closed-loop PID controller. */
#define OPEN_LOOP_STEP_TEST     0

/* Open-loop step parameters */
#define STEP_VOLTAGE            4.0f    /* applied during the step */
#define STEP_DIRECTION          (+1)    /* +1 or -1 */
#define STEP_PRE_HOLD_MS        500     /* zero-V dwell before step (baseline) */
#define STEP_DURATION_MS        5000    /* how long the step is applied */
#define STEP_POST_HOLD_MS       1500    /* zero-V dwell after, to watch decel */

#define DEADBAND_FWD          2.0f    /* V to break stiction going forward */
#define DEADBAND_REV          1.65f    /* V to break stiction going reverse */
#define DEADBAND_VEL_THRESH   0.01f   /* rev/s — below this, no stiction FF */



/* ── Step-test state (driven from TIM6 ISR) ────────────── */
enum class StepPhase : uint8_t { Idle, PreHold, Step, PostHold, Done };
static volatile StepPhase  s_stepPhase     = StepPhase::Idle;
static volatile uint32_t   s_stepPhaseStart = 0;   /* ms timestamp */
static volatile float      s_stepCmdV      = 0.0f; /* what the ISR last commanded */
static volatile int8_t     s_stepDir       = +1;



/* Trapezoidal trajectory parameters */
typedef struct {
    float p0, p1;        /* start, end (rev) */
    float vmax;          /* peak velocity magnitude (rev/s) */
    float amax;          /* acceleration magnitude (rev/s²) */

    /* Computed at start of move */
    float dir;           /* +1 or -1 */
    float t_a;           /* accel time */
    float t_c;           /* cruise time (0 for triangle profile) */
    float T;             /* total move time */
    float v_pk;          /* peak velocity actually reached */

    /* Runtime */
    float t;             /* elapsed time since move started */
    bool  active;
    bool  done;
} Trapezoid;

static Trapezoid s_traj;
static volatile float s_refPos = 0.0f;
static volatile float s_refVel = 0.0f;






/* Sinusoidal setpoint generator */
#define SIN_AMPLITUDE_REV   7.0f       /* peak position amplitude */
#define SIN_FREQ_HZ         0.159155f   /* setpoint frequency (= 0.5 rad/s) */
#define SIN_OFFSET_REV      7.0f       /* DC offset — keeps position positive */

static volatile float s_sinTarget    = SIN_OFFSET_REV;
static volatile float s_sinTargetVel = 0.0f;
static volatile float s_sinPhase     = 4.712388f;   /* 3π/2 — starts at min of sine */




static void updateSinusoidalTarget(void)
{
    const float two_pi = 6.28318530718f;
    const float omega  = two_pi * SIN_FREQ_HZ;

    s_sinPhase += omega * PID_DT;
    if (s_sinPhase >= two_pi) s_sinPhase -= two_pi;   /* keep bounded */

    s_sinTarget    = SIN_OFFSET_REV + SIN_AMPLITUDE_REV * sinf(s_sinPhase);
    s_sinTargetVel = SIN_AMPLITUDE_REV * omega * cosf(s_sinPhase);
}



/* Handles */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern I2C_HandleTypeDef hi2c3;


/* =========================================================================
 * Object instances
 * ========================================================================= */

//BLDCDriver3PWM driver(
//    &hspi3,
//    DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin,
//    DRVOFF_GPIO_Port,      DRVOFF_Pin,
//    nSLEEP_GPIO_Port,      nSLEEP_Pin,
//    PWMAL_GPIO_Port,       PWMAL_Pin,
//    PWMBL_GPIO_Port,       PWMBL_Pin,
//    PWMCL_GPIO_Port,       PWMCL_Pin,
//    &htim1
//);

BLDCDriver6PWM driver(
    &hspi3,
    DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin,
    DRVOFF_GPIO_Port,      DRVOFF_Pin,
    nSLEEP_GPIO_Port,      nSLEEP_Pin,
    &htim1
);



BLDCMotor motor(
    HALL_U_GPIO_Port, HALL_U_Pin,
    HALL_V_GPIO_Port, HALL_V_Pin,
    HALL_W_GPIO_Port, HALL_W_Pin
);

HallSensor hall(
    HALL_U_GPIO_Port, HALL_U_Pin,
    HALL_V_GPIO_Port, HALL_V_Pin,
    HALL_W_GPIO_Port, HALL_W_Pin,
    1);

LimitSwitch endstop(LIMIT_SWITCH_GPIO_Port, LIMIT_SWITCH_Pin);

Encoder encoder(&hi2c3, GEAR_RATIO);

TempSensor tempsense(&hadc5, TEMP_SAMPLE_INTERVAL_MS);

/* =========================================================================
 * Forward declarations
 * ========================================================================= */

static void Homing_Update(void);
static void Homing_CommutationTick(void);
static void USB_DispatchCommand(void);
static void USB_Reply(const char *msg);
static bool token_equals_ci(const char *tok, uint16_t len, const char *kw);

/* =========================================================================
 * Public API
 * ========================================================================= */

void MotorApp_Init(void)
{
    driver.init();
    driver.clearFault();

    motor.linkDriver(&driver);
    motor.setSupplyVoltage(SUPPLY_VOLTAGE);
    motor.init();

    hall.begin();
    encoder.begin();

    encoder.setVelocityFilterHz(5);

    /* Start with motor de-energised — PID will drive it once the
     * startup hold expires and TIM6 starts ticking. */
    motor.setVoltage(0.0f);

    /* Start TIM6 base interrupts so HAL_TIM_PeriodElapsedCallback
     * fires our PID. If you've already started TIM6 elsewhere
     * (e.g. in main.c after MX_TIM6_Init), this call is harmless. */
    HAL_TIM_Base_Start_IT(&htim6);

    tempsense.begin();


}



static void Traj_Start(float p0, float p1, float vmax, float amax)
{
    s_traj.p0    = p0;
    s_traj.p1    = p1;
    s_traj.vmax  = vmax;
    s_traj.amax  = amax;
    s_traj.dir   = (p1 >= p0) ? +1.0f : -1.0f;

    float D = (p1 - p0) * s_traj.dir;     /* always positive */
    float d_a = (vmax * vmax) / (2.0f * amax);

    if (2.0f * d_a <= D) {
        /* Trapezoid */
        s_traj.t_a  = vmax / amax;
        s_traj.t_c  = (D - 2.0f * d_a) / vmax;
        s_traj.v_pk = vmax;
    } else {
        /* Triangle — distance too short for full vmax */
        s_traj.v_pk = sqrtf(D * amax);
        s_traj.t_a  = s_traj.v_pk / amax;
        s_traj.t_c  = 0.0f;
    }

    s_traj.T      = 2.0f * s_traj.t_a + s_traj.t_c;
    s_traj.t      = 0.0f;
    s_traj.active = true;
    s_traj.done   = false;
}

static void Traj_Update(void)
{
    if (!s_traj.active) {
        s_refVel = 0.0f;
        return;
    }

    s_traj.t += PID_DT;

    float v_mag, p_mag;   /* magnitudes; sign applied at the end */

    if (s_traj.t >= s_traj.T) {
        /* Move complete — sit at p1 */
        s_traj.t      = s_traj.T;
        s_traj.active = false;
        s_traj.done   = true;
        v_mag = 0.0f;
        p_mag = s_traj.dir * (s_traj.p1 - s_traj.p0);   /* full distance */
    }
    else if (s_traj.t < s_traj.t_a) {
        /* Accelerating */
        v_mag = s_traj.amax * s_traj.t;
        p_mag = 0.5f * s_traj.amax * s_traj.t * s_traj.t;
    }
    else if (s_traj.t < s_traj.t_a + s_traj.t_c) {
        /* Cruising */
        float dt = s_traj.t - s_traj.t_a;
        v_mag = s_traj.v_pk;
        p_mag = 0.5f * s_traj.v_pk * s_traj.t_a + s_traj.v_pk * dt;
    }
    else {
        /* Decelerating */
        float dt = s_traj.t - s_traj.t_a - s_traj.t_c;
        v_mag = s_traj.v_pk - s_traj.amax * dt;
        p_mag = 0.5f * s_traj.v_pk * s_traj.t_a
              + s_traj.v_pk * s_traj.t_c
              + s_traj.v_pk * dt - 0.5f * s_traj.amax * dt * dt;
    }

    s_refPos = s_traj.p0 + s_traj.dir * p_mag;
    s_refVel = s_traj.dir * v_mag;
}


/* ── Direct-setpoint mode (used by goTo) ─────────────────────────────
 * When s_useDirectSetpoint is true, the PID chases s_directSetpoint
 * directly with no trajectory and no velocity/stiction feedforward.
 * When false, the trajectory (s_refPos/s_refVel) drives the PID — this
 * is what the auto-cycle uses. */
static volatile bool  s_useDirectSetpoint = false;
static volatile float s_directSetpoint    = 0.0f;

/* Gates the auto-cycle in MotorApp_Run(). goTo() clears it so cycling
 * doesn't overwrite the user's target on the next s_traj.done. */
static volatile bool s_autoCycleEnabled = true;




/* =========================================================================
 * Simple bench-test PID
 *
 * Output is in volts. Sign indicates direction; magnitude is fed to the
 * motor and clamped to MAX_PID_V. The hall ISR uses s_pidDir to commutate
 * in the right direction.
 *
 * Driven from the TIM6 update interrupt. PID_DT MUST match the actual
 * TIM6 update rate configured in CubeMX.
 * ========================================================================= */


static float prevError    = 0.0f;
static float integral_sum = 0.0f;
static float d_filt         = 0.0f;

static void pidVoltage(float target, float fc_hz)
{
    float dt = PID_DT;

    float error = target - encoder.getPositionRev();

    float p_term = Kp * error;

    /* Raw derivative, then low-pass filter just the D path */
        float omega = 2.0f * 3.14159265f * fc_hz;
        float a = dt / ((1.0f / omega) + dt);

        float d_raw  = (error - prevError) / dt;
        d_filt       = (1.0f - a) * d_filt + a * d_raw;
        float d_term = Kd * d_filt;

        /* Feedforward */
        float ff_velocity = Kff * s_refVel;
        float ff_stiction = 0.0f;
        if (s_refVel >  DEADBAND_VEL_THRESH)      ff_stiction =  DEADBAND_FWD;
        else if (s_refVel < -DEADBAND_VEL_THRESH) ff_stiction = -DEADBAND_REV;

        float output = p_term + integral_sum + d_term + ff_velocity + ff_stiction;

        /* Conditional integration anti-windup — now on raw error */
        bool saturated_high = (output >  MAX_PID_V) && (error > 0.0f);
        bool saturated_low  = (output < -MAX_PID_V) && (error < 0.0f);
        if (!saturated_high && !saturated_low) {
            integral_sum += Ki * error * dt;
        }

        if (output >  MAX_PID_V) output =  MAX_PID_V;
        if (output < -MAX_PID_V) output = -MAX_PID_V;

        prevError = error;   /* store RAW error for next derivative */

	/* Split signed output into magnitude + direction */
	if (output > 0.01f) {
		s_pidDir = +1;
		motor.setVoltage(output);
	} else if (output < -0.01f) {
		s_pidDir = -1;
		motor.setVoltage(-output);
	} else {
		motor.setVoltage(0.0f);
		/* keep s_pidDir at previous value — no commutation chatter */
	}
}



/* ── Plot snapshot — written by TIM6 ISR, read by MotorApp_Run() ── */
static volatile float    g_plot_pos   = 0.0f;
static volatile float    g_plot_vel   = 0.0f;
static volatile float    g_plot_velf  = 0.0f;
static volatile float    g_plot_vhall   = 0.0f;
static volatile float    g_plot_vhallf  = 0.0f;
static volatile float    g_plot_pref  = 0.0f;
static volatile float    g_plot_p     = 0.0f;
static volatile float    g_plot_vref  = 0.0f;
static volatile float    g_plot_v     = 0.0f;
static volatile uint32_t g_plot_t_ms  = 0;
static volatile uint32_t g_plot_seq   = 0;

static volatile uint32_t g_plot_cycle = 0;
static bool cycle_initialised = false;
static bool going_out = true;
static uint32_t cycle_count = 0;

/* Hall velocity IIR low-pass — matches encoder velocity filter for fair comparison */
static float s_vhallFilt = 0.0f;
#define VHALL_FILTER_FC_HZ   5.0f   /* same as encoder.setVelocityFilterHz(20) */

/* HAL TIM6 update callback — drives the PID.
 * Marked extern "C" because HAL calls it from C code; without this,
 * the C++ name-mangled version won't override the weak HAL stub. */

#define PLOT_DECIM		1
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

    /* Sample the encoder at exactly 1 kHz */
    encoder.update(PID_DT);

#if OPEN_LOOP_STEP_TEST
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - s_stepPhaseStart;

    switch (s_stepPhase)
    {
    case StepPhase::Idle:
        s_stepCmdV = 0.0f;
        break;

    case StepPhase::PreHold:
        s_stepCmdV = 0.0f;
        if (elapsed >= STEP_PRE_HOLD_MS) {
            s_stepPhase      = StepPhase::Step;
            s_stepPhaseStart = now;
        }
        break;

    case StepPhase::Step:
        s_stepCmdV = STEP_VOLTAGE;
        if (elapsed >= STEP_DURATION_MS) {
            s_stepPhase      = StepPhase::PostHold;
            s_stepPhaseStart = now;
        }
        break;

    case StepPhase::PostHold:
        s_stepCmdV = 0.0f;
        if (elapsed >= STEP_POST_HOLD_MS) {
            s_stepPhase      = StepPhase::Done;
            s_stepPhaseStart = now;
        }
        break;

    case StepPhase::Done:
    default:
        s_stepCmdV = 0.0f;
        break;
    }

    motor.setVoltage(s_stepCmdV);
    s_pidDir = s_stepDir;

#else
//    if (s_pidEnabled && !s_homingActive)
//    {
//        pidVoltage(Target, FILTER_FC_HERTZ);
//    }
//    uint32_t now = HAL_GetTick();


    uint32_t now = HAL_GetTick();
//	if (s_pidEnabled && !s_homingActive)
//	{
//		updateSinusoidalTarget();
//		pidVoltage(s_sinTarget, FILTER_FC_HERTZ);
//	}

    if (s_pidEnabled && !s_homingActive)
    {
        if (s_useDirectSetpoint) {
            /* Direct mode: kill the trajectory feedforward (s_refVel=0) so the
             * PID acts as pure PID against a static setpoint. */
            s_refVel = 0.0f;
            s_refPos = s_directSetpoint;   /* keep the plot's p_ref meaningful */
            pidVoltage(s_directSetpoint, FILTER_FC_HERTZ);
        } else {
            Traj_Update();
            pidVoltage(s_refPos, FILTER_FC_HERTZ);
        }
    }

#endif


	g_plot_pos   = encoder.getPositionRev();
	g_plot_vel   = encoder.getVelocityRevPerSec();
	g_plot_velf  = encoder.getVelocityRevPerSecFiltered();

	float v_hall_raw = hall.getRpm() / (60.0f * GEAR_RATIO);

	/* First-order IIR: y[n] = (1-a)*y[n-1] + a*x[n] */
	const float omega = 2.0f * 3.14159265f * VHALL_FILTER_FC_HZ;
	const float a     = PID_DT / ((1.0f / omega) + PID_DT);
	s_vhallFilt = (1.0f - a) * s_vhallFilt + a * v_hall_raw;

	g_plot_vhall  = v_hall_raw;
	g_plot_vhallf = s_vhallFilt;

	g_plot_pref  = s_refPos;
	g_plot_p     = g_plot_pos;
	g_plot_vref  = s_refVel;
	g_plot_v     = g_plot_velf;
	g_plot_cycle = cycle_count;
	g_plot_t_ms  = now;
	g_plot_seq++;
}














void MotorApp_Run(void)
{
    uint32_t now = HAL_GetTick();

    /* For sending and receiving commands */
    USB_DispatchCommand();
    CAN_DispatchCommands();

    static bool overTempFault = false;

    if (tempsense.update())
    {
        float    temp   = tempsense.getCelsius();
        uint32_t now_ms = HAL_GetTick();

        ThermalGuard_AddSample(temp);

        /* ── Absolute limit (existing) ─── */
        if (!overTempFault && !s_thermalRunaway && temp >= TEMP_LIMIT_C)
        {
            overTempFault = true;
            motor.setVoltage(0.0f);
            s_pidEnabled   = false;
            s_motorEnabled = false;
            s_homingActive = false;
            driver.phaseFloat(0);
            driver.phaseFloat(1);
            driver.phaseFloat(2);
            Plot_Int("fault_overtemp", 1);
            USB_Reply("FAULT: overtemperature");
        }
        else if (overTempFault && !s_thermalRunaway && temp <= TEMP_RESET_C)
        {
            overTempFault = false;
            prevError    = 0.0f;
            integral_sum = 0.0f;
            s_pidEnabled = true;
            USB_Reply("temperature recovered");
        }

        /* ── Slope guard (new) ─── */
        if (now_ms >= THERMAL_SETTLE_MS && !s_thermalRunaway)
        {
            float slope = ThermalGuard_ComputeSlope();
            Plot_Float("temp_slope", slope);

            if (slope > THERMAL_SLOPE_LIMIT_C_PER_MIN)
            {
                if (s_slopeBreachStartMs == 0)
                    s_slopeBreachStartMs = now_ms;

                if (now_ms - s_slopeBreachStartMs >= SLOPE_CONFIRM_MS)
                {
                    s_thermalRunaway = true;
                    overTempFault    = true;
                    motor.setVoltage(0.0f);
                    s_pidEnabled    = false;
                    s_motorEnabled  = false;
                    s_homingActive  = false;
                    driver.phaseFloat(0);
                    driver.phaseFloat(1);
                    driver.phaseFloat(2);

                    Plot_Int("fault_thermal_runaway", 1);

                    char msg[96];
                    snprintf(msg, sizeof(msg),
                        "FAULT: thermal runaway, slope=%.2f C/min temp=%.1f",
                        (double)slope, (double)temp);
                    USB_Reply(msg);
                }
            }
            else
            {
                /* slope healthy — reset confirmation timer */
                s_slopeBreachStartMs = 0;
            }
        }
    }

    /* Block all normal operation while faulted */
    if (overTempFault)
    {
        motor.setVoltage(0.0f);
    }



#if !OPEN_LOOP_STEP_TEST
    if (!homingCompleted)
    {
        MotorApp_StartHoming();
        homingCompleted = true;
    }
#endif


    /* ------------------------------------------------------------------
     * Startup hold: keep motor de-energised for STARTUP_HOLD_MS after
     * boot so the operator has time to connect Teleplot and start logging.
     * ------------------------------------------------------------------ */
    static const uint32_t STARTUP_HOLD_MS = 2000;  /* 2 s — adjust as needed */
    static bool startupHoldDone = false;

    if (!startupHoldDone)
    {
        motor.setVoltage(0.0f);   /* make sure windings are off */
        s_pidEnabled = false;     /* keep the PID dormant */

        /* No need to plot here — the TIM6 ISR is already streaming
         * pos/vel/cmd at 1 kHz. Teleplot will show the flat baseline. */

        if (now >= STARTUP_HOLD_MS) {
            startupHoldDone = true;
            prevError    = 0.0f;
            integral_sum = 0.0f;
    #if OPEN_LOOP_STEP_TEST
            s_stepDir        = (STEP_DIRECTION >= 0) ? +1 : -1;
            s_stepPhase      = StepPhase::PreHold;
            s_stepPhaseStart = now;
            USB_Reply("step test started");
    #else
            s_pidEnabled = true;
    #endif
        }
        return;
    }




    /* ------------------------------------------------------------------
     * Plot (always runs)
     * ------------------------------------------------------------------ */
    /* ------------------------------------------------------------------
	 * Plot drain — non-blocking, every iteration.
	 * Producer (TIM6 ISR) writes samples; this just empties the ring.
	 * ------------------------------------------------------------------ */

    static uint32_t lastSeq = 0;
    static uint32_t lastMotorPlot = 0;
    if (g_plot_seq != lastSeq && (now - lastMotorPlot >= PLOT_MS))
    {
        lastSeq = g_plot_seq;
        lastMotorPlot = now;
        Plot_Sample4f3(g_plot_t_ms,
            "p_ref",  g_plot_pref * 4,
            "p",      g_plot_p * 4,
            "v_ref",  g_plot_vref * 4,
            "v",      g_plot_v * 4);
        Plot_Sample4f3(g_plot_t_ms,
                    "temp",    tempsense.getCelsius(),
                    "pos_error",  (g_plot_pref - g_plot_p) * 4,
                    "hall_raw", g_plot_vhallf,
                    "cycle_count", (float)cycle_count);
    }




    /* ------------------------------------------------------------------
     * Homing path: when active, owns the motor and skips normal drive.
     * ------------------------------------------------------------------ */
    if (s_homingActive)
    {
        static uint32_t lastHoming = 0;
        if (now - lastHoming >= HOMING_TICK_MS)
        {
            lastHoming = now;
            Homing_Update();
            Homing_CommutationTick();
        }
    }
    else if (!overTempFault)
    {

        /* Slow startup-from-rest kick. Once the rotor is moving, the Hall
         * ISR commutates far faster than this and these calls are
         * effectively no-ops. Uses s_pidDir so the rotor kicks in the
         * direction the PID wants to go. */
        static uint32_t lastKick = 0;
        static uint32_t last_limit = 0;
        if (now - lastKick >= KICK_INTERVAL_MS)
        {
            lastKick = now;
            motor.run(hall.getState(), s_pidDir);
        }
        else if (now - last_limit >= ENDSTOP_CHECK_MS && endstop.isPressed())
        {
        	motor.setVoltage(0.0f);
        }
    }


    if (s_pidEnabled && !s_homingActive && s_autoCycleEnabled)
    {
    	if (!cycle_initialised)
    	{
    	    Traj_Start(encoder.getPositionRev(),
    	               CYCLE_OUT_POS,
    	               CYCLE_VMAX,
    	               CYCLE_AMAX);
    	    cycle_initialised = true;
    	    going_out = true;
    	}
        else if (s_traj.done)
        {
            if (!going_out)
			{
            	cycle_count++;
			}

            going_out = !going_out;

            /* Start from the COMMANDED endpoint of the last move, not the
			 * measured position. This keeps every cycle's trajectory identical
			 * regardless of how well the PID tracked the previous move. */
			float p_start = going_out ? CYCLE_IN_POS  : CYCLE_OUT_POS;
			float p_end   = going_out ? CYCLE_OUT_POS : CYCLE_IN_POS;
			Traj_Start(p_start, p_end, CYCLE_VMAX, CYCLE_AMAX);
        }

    }




    /* ------------------------------------------------------------------
     * Fault check (always runs)
     * ------------------------------------------------------------------ */
    static uint32_t lastFault = 0;
    if (now - lastFault >= FAULT_CHECK_MS)
    {
        lastFault = now;
        if (driver.isFault())
        {
            motor.setVoltage(0.0f);
            driver.clearFault();
        }
    }

}


void MotorApp_SimpleRun(void)
{
    static bool started = false;
    if (!started)
    {
        started = true;
        motor.setVoltage(6.0f);
    }
    motor.run(hall.getState(), 1);
}


/* =========================================================================
 * Position commands
 * ========================================================================= */

#define TRAVEL_MIN_REV   0.0f
#define TRAVEL_MAX_REV   CYCLE_OUT_POS    /* 25 rev = 100 mm */

void MotorApp_GoTo(float targetRev)
{
    if (s_homingActive)   { USB_Reply("goTo rejected: homing");       return; }
    if (!homingCompleted) { USB_Reply("goTo rejected: not homed");    return; }
    if (!s_pidEnabled)    { USB_Reply("goTo rejected: pid disabled"); return; }

    if (targetRev < TRAVEL_MIN_REV) targetRev = TRAVEL_MIN_REV;
    if (targetRev > TRAVEL_MAX_REV) targetRev = TRAVEL_MAX_REV;

    /* Disable cycling and the trajectory, then switch to direct mode. */
    s_autoCycleEnabled  = false;
    s_traj.active       = false;
    s_directSetpoint    = targetRev;
    s_useDirectSetpoint = true;
    __DSB();

    char msg[64];
    snprintf(msg, sizeof(msg), "goTo %.3f rev", (double)targetRev);
    USB_Reply(msg);
}

void MotorApp_StartAutoCycle(void)
{
    /* Leave direct mode and re-arm cycling from the current position. */
    s_useDirectSetpoint = false;
    s_autoCycleEnabled  = true;
    cycle_initialised   = false;
    USB_Reply("auto-cycle enabled");
}



/* =========================================================================
 * Homing
 * ========================================================================= */

void MotorApp_StartHoming(void)
{
    if (driver.isFault())
    {
        driver.clearFault();
        HAL_Delay(1);
    }

    s_backoffStartEdges = 0;
    s_settleDeadline    = 0;
    s_homingState   = HomingState::MoveToSwitch;
    s_homingDir     = DIR_TOWARD_SWITCH;
    s_homingVoltage = HOMING_VOLTAGE_MIN;
    s_motorEnabled  = true;
    s_homingActive  = true;
}

bool MotorApp_HomingDone(void)
{
    return s_homingState == HomingState::Done;
}

bool MotorApp_HomingActive(void)
{
    return s_homingActive;
}


static uint32_t s_lastMotionMs = 0;
static HomingState s_prevState = HomingState::Idle;

static void Homing_Update(void)
{
    if (!s_homingActive) return;

    bool     pressed = endstop.isPressed();
    int32_t  pos     = hall.getEdgeCount();
    uint32_t now     = HAL_GetTick();
    float    dt      = HOMING_TICK_MS * 0.001f;  /* fixed tick = clean dt */

    /* Reset integrator + watchdog on entry to a driving state */
    bool stateEntry = (s_homingState != s_prevState);
    s_prevState = s_homingState;

    switch (s_homingState)
    {
    case HomingState::MoveToSwitch:
    {
        if (stateEntry) {
            s_homingVoltage = HOMING_VOLTAGE_MIN;
            s_lastMotionMs  = now;
        }

        s_motorEnabled = true;
        s_homingDir    = DIR_TOWARD_SWITCH;

        /* I-only velocity loop */
        float rpm     = fabsf(hall.getRpm());
        float rpm_err = HOMING_TARGET_RPM - rpm;
        s_homingVoltage += HOMING_KI * rpm_err * dt;
        if (s_homingVoltage < HOMING_VOLTAGE_MIN) s_homingVoltage = HOMING_VOLTAGE_MIN;
        if (s_homingVoltage > HOMING_VOLTAGE_MAX) s_homingVoltage = HOMING_VOLTAGE_MAX;
        motor.setVoltage(s_homingVoltage);

        /* Stall watchdog — only meaningful when we're commanding near max */
        if (rpm >= HOMING_STALL_RPM) s_lastMotionMs = now;
        if (s_homingVoltage >= HOMING_VOLTAGE_MAX * 0.95f &&
            (now - s_lastMotionMs) > HOMING_STALL_TIMEOUT_MS)
        {
            motor.setVoltage(0.0f);
            s_homingActive = false;
            s_homingState  = HomingState::Idle;
            USB_Reply("homing FAULT: stalled approaching switch");
            return;
        }

        if (pressed)
        {
            s_motorEnabled  = false;
            s_homingDir     = 0;
            s_homingVoltage = 0.0f;
            motor.setVoltage(0.0f);
            driver.phaseFloat(0);
            driver.phaseFloat(1);
            driver.phaseFloat(2);

            s_settleDeadline = now + HOMING_SETTLE_MS;
            s_homingState    = HomingState::Settle;
            USB_Reply("homing switch hit, settling");
        }
        break;
    }

    case HomingState::Settle:
        /* Unchanged — motor is off, nothing to regulate */
        s_motorEnabled  = false;
        s_homingDir     = 0;
        s_homingVoltage = 0.0f;

        if ((int32_t)(now - s_settleDeadline) >= 0)
        {
            s_backoffStartEdges = pos;
            s_homingState       = HomingState::Backoff;
            USB_Reply("homing backing off");
        }
        break;

    case HomingState::Backoff:
    {
        if (stateEntry) {
            s_homingVoltage = HOMING_VOLTAGE_MIN;
            s_lastMotionMs  = now;
        }

        s_motorEnabled = true;
        s_homingDir    = DIR_AWAY_FROM_SWITCH;

        float rpm     = fabsf(hall.getRpm());
        float rpm_err = HOMING_TARGET_RPM - rpm;
        s_homingVoltage += HOMING_KI * rpm_err * dt;
        if (s_homingVoltage < HOMING_VOLTAGE_MIN) s_homingVoltage = HOMING_VOLTAGE_MIN;
        if (s_homingVoltage > HOMING_VOLTAGE_MAX) s_homingVoltage = HOMING_VOLTAGE_MAX;
        motor.setVoltage(s_homingVoltage);

        if (rpm >= HOMING_STALL_RPM) s_lastMotionMs = now;
        if (s_homingVoltage >= HOMING_VOLTAGE_MAX * 0.95f &&
            (now - s_lastMotionMs) > HOMING_STALL_TIMEOUT_MS)
        {
            motor.setVoltage(0.0f);
            s_homingActive = false;
            s_homingState  = HomingState::Idle;
            USB_Reply("homing FAULT: stalled during backoff");
            return;
        }

        if (abs(pos - s_backoffStartEdges) >= HOMING_BACKOFF_EDGES && !pressed)
        {
            s_motorEnabled  = false;
            s_homingDir     = 0;
            s_homingVoltage = 0.0f;
            motor.setVoltage(0.0f);
            driver.phaseFloat(0);
            driver.phaseFloat(1);
            driver.phaseFloat(2);

            encoder.resetCount();
            s_settleDeadline = now + 2000;   /* 2 s dwell */
            s_homingState    = HomingState::PostBackoffSettle;
        }
        break;
    }

    case HomingState::PostBackoffSettle:
        /* Hold motor off, wait for dwell to elapse before declaring done */
        s_motorEnabled  = false;
        s_homingDir     = 0;
        s_homingVoltage = 0.0f;

        if ((int32_t)(now - s_settleDeadline) >= 0)
        {


            s_homingState = HomingState::Done;
            USB_Reply("homing settle complete");
        }
        break;

    case HomingState::Done:
        s_motorEnabled  = false;
        s_homingVoltage = 0.0f;
        s_homingDir     = 0;
        motor.setVoltage(0.0f);
        s_homingActive  = false;
        prevError       = 0.0f;
        integral_sum    = 0.0f;

        /* Boot into direct-setpoint mode, parked at home. */
		s_autoCycleEnabled  = false;
		s_directSetpoint    = 0.0f;
		s_useDirectSetpoint = true;

        s_pidEnabled    = true;
        USB_Reply("homing done, PID enabled");
        return;

    default:
        s_homingState  = HomingState::Idle;
        s_homingActive = false;
        s_motorEnabled = false;
        motor.setVoltage(0.0f);
        return;
    }
}

static void Homing_CommutationTick(void)
{
    if (!s_homingActive || !s_motorEnabled || s_homingDir == 0) return;
    motor.run(hall.getState(), s_homingDir);
}

/* =========================================================================
 * USB command interface
 *
 * Recognised commands (case-insensitive, terminated by \r or \n):
 *   homing / home  — start homing sequence
 *   stop           — abort homing AND freeze the PID at zero V
 *   go             — re-enable the PID after a stop
 *   status         — print current state
 *   ping           — replies "pong"
 * ========================================================================= */

void MotorApp_USB_Receive(const uint8_t *buf, uint32_t len)
{
    if (s_cmdReady)
    {
        s_cmdOverflow = true;
        return;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t c = buf[i];

        if (c == '\r' || c == '\n')
        {
            if (s_cmdLen > 0)
            {
                s_cmdReady = true;
                return;
            }
            continue;
        }

        if (s_cmdLen < (USB_CMD_BUF_SIZE - 1))
        {
            s_cmdBuf[s_cmdLen++] = c;
        }
        else
        {
            s_cmdOverflow = true;
        }
    }
}

static void USB_Reply(const char *msg)
{
    char buf[96];
    int n = snprintf(buf, sizeof(buf), ">log:%s|t\n", msg);
    if (n > 0) CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
}

static bool token_equals_ci(const char *tok, uint16_t len, const char *kw)
{
    uint16_t klen = (uint16_t)strlen(kw);
    if (len != klen) return false;
    for (uint16_t i = 0; i < len; i++)
    {
        if (tolower((unsigned char)tok[i]) != tolower((unsigned char)kw[i]))
        {
            return false;
        }
    }
    return true;
}

static void USB_DispatchCommand(void)
{
    if (!s_cmdReady) return;

    char     line[USB_CMD_BUF_SIZE];
    uint16_t len = s_cmdLen;
    if (len >= USB_CMD_BUF_SIZE) len = USB_CMD_BUF_SIZE - 1;
    memcpy(line, (const void *)s_cmdBuf, len);
    line[len] = '\0';

    s_cmdLen      = 0;
    s_cmdReady    = false;
    bool overflowed = s_cmdOverflow;
    s_cmdOverflow = false;

    char *p = line;
    while (*p == ' ' || *p == '\t') p++;

    char *e = p;
    while (*e && *e != ' ' && *e != '\t') e++;
    uint16_t tlen = (uint16_t)(e - p);

    if (tlen == 0) return;

    if (token_equals_ci(p, tlen, "homing") || token_equals_ci(p, tlen, "home"))
    {
        if (s_homingActive)
        {
            USB_Reply("homing already running");
        }
        else
        {
            MotorApp_StartHoming();
            USB_Reply("homing started");
        }
    }
    else if (token_equals_ci(p, tlen, "stop"))
    {
        s_homingActive = false;
        s_motorEnabled = false;
        s_homingState  = HomingState::Idle;
        s_homingDir    = 0;
        s_pidEnabled   = false;       /* freeze the PID too */
        motor.setVoltage(0.0f);
        USB_Reply("stop ok");
    }
    else if (token_equals_ci(p, tlen, "go"))
    {
        /* Re-enable the PID after a 'stop' */
        prevError    = 0.0f;
        integral_sum = 0.0f;
        s_pidEnabled = true;
        USB_Reply("pid enabled");
    }
    else if (token_equals_ci(p, tlen, "status"))
    {
        char reply[160];
        int n = snprintf(reply, sizeof(reply),
            ">log:status homing=%d state=%d edges=%ld switch=%d pid=%d dir=%d pos=%.3f|t\n",
            (int)s_homingActive,
            (int)s_homingState,
            (long)hall.getEdgeCount(),
            (int)endstop.isPressed(),
            (int)s_pidEnabled,
            (int)s_pidDir,
            (double)encoder.getPositionRev());
        if (n > 0) CDC_Transmit_FS((uint8_t *)reply, (uint16_t)n);
    }
    else if (token_equals_ci(p, tlen, "ping"))
    {
        USB_Reply("pong");
    }
    else if (token_equals_ci(p, tlen, "goto") || token_equals_ci(p, tlen, "g"))
    {
        char *arg = e;
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0') {
            USB_Reply("err goto needs a position in rev");
        } else {
            float target = strtof(arg, nullptr);
            MotorApp_GoTo(target);
        }
    }
    else if (token_equals_ci(p, tlen, "cycle"))
    {
        MotorApp_StartAutoCycle();
    }
    else
    {
        USB_Reply("err unknown command");
    }

    if (overflowed)
    {
        USB_Reply("warn rx buffer overflow");
    }
}



/* =========================================================================
 * CAN command interface
 *
 * One CAN ID per command — no string parsing. Payloads are little-endian.
 * Replies are emitted on CAN_ID_ACK_REPLY (1 byte: 0=ok, 1=err, 2=warn)
 * and optionally on CAN_ID_STATUS_REPLY.
 *
 * RX is already buffered by CANBus (its ISR pushes into a ring). We just
 * drain it here from the main loop, exactly the same way USB does.
 * ========================================================================= */

static void CAN_Reply(uint32_t id, const uint8_t *data, uint8_t len)
{
    can.send(id, data, len);
}

static void CAN_DispatchCommands(void)
{
    CANBus::Frame f;
    while (can.receive(f))
    {
        switch (f.id)
        {
        case CAN_ID_GOTO:
        {
            if (f.len < 4)
            {
                uint8_t err = 1;
                CAN_Reply(CAN_ID_ACK_REPLY, &err, 1);
                break;
            }
            /* Decode 4-byte little-endian float32 — type-punning via memcpy
             * is the only portable way (avoids strict-aliasing UB). */
            float target;
            memcpy(&target, f.data, sizeof(float));
            MotorApp_GoTo(target);

            uint8_t ok = 0;
            CAN_Reply(CAN_ID_ACK_REPLY, &ok, 1);
            break;
        }

        case CAN_ID_AUTOCYCLE:
            MotorApp_StartAutoCycle();
            { uint8_t ok = 0; CAN_Reply(CAN_ID_ACK_REPLY, &ok, 1); }
            break;

        case CAN_ID_STOP:
            s_homingActive = false;
            s_motorEnabled = false;
            s_homingState  = HomingState::Idle;
            s_homingDir    = 0;
            motor.setVoltage(0.0f);
            { uint8_t ok = 0; CAN_Reply(CAN_ID_ACK_REPLY, &ok, 1); }
            break;

        case CAN_ID_HOMING:
            if (!s_homingActive) MotorApp_StartHoming();
            { uint8_t ok = 0; CAN_Reply(CAN_ID_ACK_REPLY, &ok, 1); }
            break;

        case CAN_ID_PING:
            CAN_Reply(CAN_ID_ACK_REPLY, (const uint8_t *)"PONG", 4);
            break;

        default:
            /* Unknown ID — silently ignore (the filter is accept-all so
             * we'll see traffic intended for other nodes too). */
            break;
        }
    }
}




/* =========================================================================
 * Misc accessors / callbacks
 * ========================================================================= */

int8_t MotorApp_GetActiveLowPhase(void)
{
    return motor.getActiveLowPhase();
}

void MotorApp_HallCallback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != HALL_U_Pin && GPIO_Pin != HALL_V_Pin && GPIO_Pin != HALL_W_Pin)
    {
        return;
    }

    /* Always update the hall position tracker — we want edge counts even
     * when the motor is being held at zero voltage. */
    hall.onEdge(GPIO_Pin);

    /* Commutate only when allowed. During homing settle / done / stop, the
     * motor-enable gate is off so rotor wiggle doesn't re-energize windings. */
    if (s_homingActive)
    {
        if (s_motorEnabled && s_homingDir != 0)
        {
            motor.run(hall.getState(), s_homingDir);
        }
    }
    else
    {
    	int8_t commDir = s_pidDir;
    	motor.run(hall.getState(), commDir);
    }
}
