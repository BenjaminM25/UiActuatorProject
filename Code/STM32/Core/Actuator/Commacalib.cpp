/*
 * Commacalib.cpp
 *
 *  Created on: 19. apr. 2026
 *      Author: Benjamin
 */


/**
 * @file    CommCalib.cpp
 * @brief   Commutation calibration implementation.
 *
 * Steps through all 6 phase combinations at high duty to overcome
 * gearbox detent forces. For each combination:
 *   1. Energises the phase pair for LOCK_MS milliseconds
 *   2. Reads Hall state multiple times and takes majority vote
 *   3. Plots result via Teleplot
 *
 * Expected output - 6 unique Hall states (1-6), one per combination:
 *   AB_locks → X
 *   BA_locks → X
 *   BC_locks → X
 *   CB_locks → X
 *   CA_locks → X
 *   AC_locks → X
 *
 * Use results to build TRAP_TABLE:
 *   For forward sequence 1→3→2→6→4→5:
 *   When Hall=N, apply phase pair that locks to NEXT Hall state.
 */

#include "CommCalib.h"
#include "BLDCDriver3PWM.h"
#include "plot.h"
#include "main.h"
#include "tim.h"
#include "spi.h"
#include "usbd_cdc_if.h"

/* =========================================================================
 * Configuration
 * ========================================================================= */

#define LOCK_MS         3000    /* ms to hold each position - wiggle shaft during this time */
#define LOCK_DUTY       0.4f    /* full duty to maximise torque against gearbox */
#define SAMPLE_COUNT    10      /* Hall readings to take for majority vote */
#define SAMPLE_DELAY_MS 50      /* ms between Hall samples */

/* =========================================================================
 * Hardware objects (separate from MotorApp)
 * ========================================================================= */

static BLDCDriver3PWM calibDriver(
    &hspi3,
    DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin,
    DRVOFF_GPIO_Port,      DRVOFF_Pin,
    nSLEEP_GPIO_Port,      nSLEEP_Pin,
    PWMAL_GPIO_Port,       PWMAL_Pin,
    PWMBL_GPIO_Port,       PWMBL_Pin,
    PWMCL_GPIO_Port,       PWMCL_Pin,
    &htim1
);

/* =========================================================================
 * Helpers
 * ========================================================================= */

static uint8_t readHall(void)
{
    uint8_t u = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t v = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t w = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) == GPIO_PIN_SET ? 1 : 0;
    return (u << 2) | (v << 1) | w;
}

/**
 * @brief  Majority vote over SAMPLE_COUNT readings.
 *         Returns the most frequently seen Hall state.
 */
static uint8_t readHallMajority(void)
{
    uint8_t counts[8] = {0};
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        uint8_t h = readHall();
        if (h < 8) counts[h]++;
        HAL_Delay(SAMPLE_DELAY_MS);
    }
    uint8_t best = 0;
    for (int i = 1; i < 8; i++) {
        if (counts[i] > counts[best]) best = i;
    }
    return best;
}

/**
 * @brief  Force a phase combination, wait for rotor to lock, read Hall state.
 * @param  high   Phase to drive high (0=A, 1=B, 2=C)
 * @param  low    Phase to drive low  (0=A, 1=B, 2=C)
 * @param  name   Label for Teleplot
 */
static void testCombo(uint8_t high, uint8_t low, const char* name)
{
	uint8_t flt = 3 - high - low;

    calibDriver.phaseFloat(0);
    calibDriver.phaseFloat(1);
    calibDriver.phaseFloat(2);
    HAL_Delay(10);
    calibDriver.clearFault();

    calibDriver.setChannelDuty(high, LOCK_DUTY);
    calibDriver.setChannelDuty(low,  0.0f);
    calibDriver.phaseFloat(flt);

    /* Flush periodically during the lock wait */
    for (int t = 0; t < LOCK_MS / 100; t++) {
        HAL_Delay(100);
        Plot_Flush();
    }

    uint8_t hall = readHallMajority();

    Plot_Int(name, hall);
    HAL_Delay(50);
    Plot_Flush();
    HAL_Delay(50);

    calibDriver.phaseFloat(0);
    calibDriver.phaseFloat(1);
    calibDriver.phaseFloat(2);
    HAL_Delay(200);
    Plot_Flush();
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void CommCalib_Run(void)
{
	Plot_Int("calib_alive", 1);
	HAL_Delay(200);

    /* Disable Hall interrupt so commutation doesn't interfere */
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

    /* Initialise driver */
    calibDriver.init();
    calibDriver.clearFault();

    Plot_Int("calib_step", 1);
    HAL_Delay(100);

    /* Test all 6 phase combinations */
    /* Format: high phase, low phase, label */
    testCombo(0, 1, "AB_locks");  /* A high, B low */
    testCombo(1, 0, "BA_locks");  /* B high, A low */
    testCombo(1, 2, "BC_locks");  /* B high, C low */
    testCombo(2, 1, "CB_locks");  /* C high, B low */
    testCombo(2, 0, "CA_locks");  /* C high, A low */
    testCombo(0, 2, "AC_locks");  /* A high, C low */

    Plot_Int("calib_step", 2);
    HAL_Delay(100);

    /* Safe state */
    calibDriver.phaseFloat(0);
    calibDriver.phaseFloat(1);
    calibDriver.phaseFloat(2);
    calibDriver.disable();

    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    Plot_Int("calib_done", 1);
}

