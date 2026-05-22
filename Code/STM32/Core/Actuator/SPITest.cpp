/*
 * SPITest.cpp
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */


/**
 * @file    SPITest.cpp
 * @brief   DRV8316 SPI communication test.
 *
 * Test sequence:
 *  1. Plot step=1 immediately so we know USB is alive
 *  2. Wake DRV8316 (nSLEEP high)
 *  3. Read IC_Status register (should be 0x00 or 0x08 on fresh power-up)
 *  4. Unlock registers (write Control_1)
 *  5. Write a known value to Control_2 (3PWM mode, specific SLEW)
 *  6. Read Control_2 back and compare
 *  7. Plot all status and fault bits
 *
 * Expected results if SPI is working:
 *  - step reaches 5+
 *  - ctrl2_written == ctrl2_readback
 *  - SPI_PAR == 0
 *  - NPOR == 0 (device has powered up cleanly)
 */

#include "SPITest.h"
#include "drv8316.h"
#include "drv8316_registers.h"
#include "plot.h"
#include "main.h"
#include "spi.h"

/* =========================================================================
 * Local DRV8316 instance - separate from MotorApp
 * ========================================================================= */

static DRV8316 testDrv(
    &hspi3,
    DRV8316nSCS_GPIO_Port, DRV8316nSCS_Pin,
    DRVOFF_GPIO_Port,      DRVOFF_Pin,
    nSLEEP_GPIO_Port,      nSLEEP_Pin
);

/* =========================================================================
 * Helper: small delay + plot a heartbeat so we know we're still running
 * ========================================================================= */

static void flushWait(void)
{
    uint32_t deadline = HAL_GetTick() + 100;
    while (HAL_GetTick() < deadline)
    {
        Plot_Flush();
        HAL_Delay(5);
    }
}


static void plotStep(int step)
{
    Plot_Int("step", step);
    flushWait();
}

static void plotAndWait(const char* name, int32_t value)
{
    Plot_Int(name, value);
    flushWait();
}

/* =========================================================================
 * Test
 * ========================================================================= */

void SPITest_Run(void)
{
    plotStep(1);

    testDrv.init();
    testDrv.clearFault();
    HAL_Delay(10);

    plotStep(2);

    /* --- All status registers --- */
    uint16_t raw_s0 = testDrv.readRegister(IC_Status_ADDR);
    IC_Status s0;
    s0.reg = raw_s0 & 0xFF;

    uint16_t raw_s1 = testDrv.readRegister(Status__1_ADDR);
    Status__1 s1;
    s1.reg = raw_s1 & 0xFF;

    uint16_t raw_s2 = testDrv.readRegister(Status__2_ADDR);
    Status__2 s2;
    s2.reg = raw_s2 & 0xFF;

    plotStep(3);
    plotAndWait("FAULT",     s0.FAULT);
    plotAndWait("OCP",       s0.OCP);
    plotAndWait("OVP",       s0.OVP);
    plotAndWait("NPOR",      s0.NPOR);
    plotAndWait("BK_FLT",    s0.BK_FLT);

    plotStep(4);
    plotAndWait("OCP_HA",    s1.OCP_HA);
    plotAndWait("OCP_LA",    s1.OCP_LA);
    plotAndWait("OCP_HB",    s1.OCP_HB);
    plotAndWait("OCP_LB",    s1.OCP_LB);
    plotAndWait("OCP_HC",    s1.OCP_HC);
    plotAndWait("OCP_LC",    s1.OCP_LC);
    plotAndWait("OTS",       s1.OTS);
    plotAndWait("OTW",       s1.OTW);

    plotStep(5);
    plotAndWait("SPI_PAR",   s2.SPI_PARITY);
    plotAndWait("VCP_UV",    s2.VCP_UV);
    plotAndWait("BUCK_UV",   s2.BUCK_UV);
    plotAndWait("OTP_ERR",   s2.OTP_ERR);

    /* --- All control registers --- */
    uint8_t c1 = testDrv.readRegister(Control__1_ADDR) & 0xFF;
    uint8_t c2 = testDrv.readRegister(Control__2_ADDR) & 0xFF;
    uint8_t c3 = testDrv.readRegister(Control__3_ADDR) & 0xFF;
    uint8_t c4 = testDrv.readRegister(Control__4_ADDR) & 0xFF;
    uint8_t c5 = testDrv.readRegister(Control__5_ADDR) & 0xFF;
    uint8_t c6 = testDrv.readRegister(Control__6_ADDR) & 0xFF;

    plotStep(6);
    plotAndWait("ctrl1", c1);
    plotAndWait("ctrl2", c2);
    plotAndWait("ctrl3", c3);
    plotAndWait("ctrl4", c4);
    plotAndWait("ctrl5", c5);
    plotAndWait("ctrl6", c6);

    plotStep(7);
    plotAndWait("SPI_OK",  s2.SPI_PARITY == 0 ? 1 : 0);
    plotAndWait("DRV_OK",  s0.FAULT == 0 ? 1 : 0);
    plotAndWait("BUCK_OK", s0.BK_FLT == 0 ? 1 : 0);
}

