/*
 * Diagnostics.cpp
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */



/**
 * @file    Diagnostics.cpp
 * @brief   System diagnostics for BLDC motor controller bring-up.
 *
 * Tests in order:
 *  1. USB plotting (if you see this, USB works)
 *  2. DRV8316 SPI communication
 *  3. DRV8316 register config verification
 *  4. nFAULT pin state
 *  5. Hall sensor reading
 *  6. TIM1 PWM start (no motor drive, just checks timer starts)
 *  7. Low voltage motor drive attempt (1V, checks for faults)
 */

#include "Diagnostics.h"
#include "drv8316.h"
#include "drv8316_registers.h"
#include "BLDCDriver3PWM.h"
#include "BLDCMotor.h"
#include "plot.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

/* =========================================================================
 * Hardware objects
 * ========================================================================= */

static DRV8316 diagDrv(
    &hspi3,
    DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin,
    DRVOFF_GPIO_Port,      DRVOFF_Pin,
    nSLEEP_GPIO_Port,      nSLEEP_Pin
);

static BLDCDriver3PWM diagDriver(
    &hspi3,
    DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin,
    DRVOFF_GPIO_Port,      DRVOFF_Pin,
    nSLEEP_GPIO_Port,      nSLEEP_Pin,
    PWMAL_GPIO_Port,       PWMAL_Pin,
    PWMBL_GPIO_Port,       PWMBL_Pin,
    PWMCL_GPIO_Port,       PWMCL_Pin,
    &htim1
);

static BLDCMotor diagMotor(
    HALL_U_GPIO_Port, HALL_U_Pin,
    HALL_V_GPIO_Port, HALL_V_Pin,
    HALL_W_GPIO_Port, HALL_W_Pin
);

/* =========================================================================
 * Helpers
 * ========================================================================= */

static void wait(void)
{
    HAL_Delay(100);
}

static void plotPass(const char* name, int pass)
{
    Plot_Int(name, pass ? 1 : 0);
    wait();
}

static void section(int num)
{
    Plot_Int("section", num);
    wait();
}

/* =========================================================================
 * Diagnostics
 * ========================================================================= */

void Diagnostics_Run(void)
{
    /* ---------------------------------------------------------------
     * Section 1: USB
     * If you see this in Teleplot, USB CDC is working.
     * --------------------------------------------------------------- */
    section(1);
    plotPass("USB_OK", 1);

    /* ---------------------------------------------------------------
     * Section 2: DRV8316 SPI basic communication
     * Wake device manually and read IC_Status.
     * Expected: raw value changes from 0 (all OK) or has NPOR set.
     * --------------------------------------------------------------- */
    section(2);

    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRVOFF_GPIO_Port, DRVOFF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    uint16_t raw0 = diagDrv.readRegister(IC_Status_ADDR);
    IC_Status s0;
    s0.reg = raw0 & 0xFF;

    /* A valid SPI response will have upper byte = address echo.
     * If raw0 == 0xFFFF, MISO is floating (SPI not working). */
    int spiWorking = (raw0 != 0xFFFF) && (raw0 != 0x0000);
    plotPass("SPI_COMMS_OK", spiWorking);
    Plot_Int("raw_IC_status", raw0);
    wait();

    /* ---------------------------------------------------------------
     * Section 3: DRV8316 full init and register verification
     * --------------------------------------------------------------- */
    section(3);

    diagDrv.init();
    diagDrv.clearFault();
    HAL_Delay(10);

    /* Read back Control_2 - expect 0x2C (3PWM, SLEW=01, SDO push-pull) */
    uint16_t ctrl2_raw = diagDrv.readRegister(Control__2_ADDR);
    uint8_t ctrl2 = ctrl2_raw & 0xFF;
    plotPass("CTRL2_3PWM_OK", ctrl2 == 0x2C);
    Plot_Int("ctrl2_val", ctrl2);
    wait();

    /* Read back Control_6 - expect 0x11 (buck disabled) */
    uint16_t ctrl6_raw = diagDrv.readRegister(Control__6_ADDR);
    uint8_t ctrl6 = ctrl6_raw & 0xFF;
    plotPass("CTRL6_BUCK_DIS_OK", ctrl6 == 0x11);
    Plot_Int("ctrl6_val", ctrl6);
    wait();

    /* Check status after init and clear */
    uint16_t status_raw = diagDrv.readRegister(IC_Status_ADDR);
    IC_Status sf;
    sf.reg = status_raw & 0xFF;
    plotPass("DRV_NO_FAULT", sf.FAULT == 0);
    plotPass("DRV_NO_BKFLT", sf.BK_FLT == 0);
    plotPass("DRV_NO_OVP",   sf.OVP == 0);
    plotPass("DRV_NO_OCP",   sf.OCP == 0);
    wait();

    /* ---------------------------------------------------------------
     * Section 4: nFAULT pin
     * Should be HIGH (no fault) = GPIO reads SET
     * --------------------------------------------------------------- */
    section(4);
    int nfaultHigh = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin) == GPIO_PIN_SET;
    plotPass("nFAULT_OK", nfaultHigh);

    /* ---------------------------------------------------------------
     * Section 5: Hall sensors
     * Read Hall state - should be 1-6 (not 0 or 7)
     * --------------------------------------------------------------- */
    section(5);
    uint8_t u = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t v = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t w = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t hall = (u << 2) | (v << 1) | w;

    Plot_Int("Hall_U", u);
    wait();
    Plot_Int("Hall_V", v);
    wait();
    Plot_Int("Hall_W", w);
    wait();
    Plot_Int("Hall_state", hall);
    wait();
    plotPass("Hall_VALID", hall >= 1 && hall <= 6);

    /* ---------------------------------------------------------------
     * Section 6: TIM1 PWM channels
     * Start at 0% duty - just verifies timer starts without hanging
     * --------------------------------------------------------------- */
    section(6);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    plotPass("TIM1_OK", 1);

    /* ---------------------------------------------------------------
     * Section 7: BLDCDriver3PWM + BLDCMotor init
     * Full driver init, link motor, set 1V drive
     * Watch nFAULT - if it lights up something is wrong with PWM
     * --------------------------------------------------------------- */
    section(7);

    /* Stop TIM1 first since section 6 already started it */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    diagDriver.init();
    diagMotor.linkDriver(&diagDriver);
    diagMotor.setSupplyVoltage(5.0f);
    diagMotor.init();
    diagMotor.setVoltage(1.0f);

    /* Run a few commutation steps */
    for (int i = 0; i < 10; i++) {
        diagMotor.run(hall, 1);
        HAL_Delay(10);
    }

    /* Check for faults after brief motor run */
    int noFaultAfterRun = !diagDriver.isFault();
    plotPass("MOTOR_NO_FAULT", noFaultAfterRun);

    uint8_t hallAfter = (
        (HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin) == GPIO_PIN_SET ? 1 : 0) << 2 |
        (HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin) == GPIO_PIN_SET ? 1 : 0) << 1 |
        (HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin) == GPIO_PIN_SET ? 1 : 0)
    );
    Plot_Int("Hall_after", hallAfter);
    wait();

    /* Stop motor safely */
    diagMotor.setVoltage(0.0f);
    diagDriver.disable();

    /* ---------------------------------------------------------------
     * Done
     * --------------------------------------------------------------- */
    section(99);
    plotPass("DIAG_COMPLETE", 1);
}
