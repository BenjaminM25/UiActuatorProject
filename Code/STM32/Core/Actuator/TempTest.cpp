/*
 * TempTest.cpp
 *
 *  Created on: 14. mai 2026
 *      Author: Benjamin
 */


#include "TempTest.h"
#include "TempSensor.h"
#include "plot.h"
#include "adc.h"

static TempSensor s_temp(&hadc5, TEMP_SAMPLE_INTERVAL_MS);
static bool		  s_started = false;


extern "C" void TEMPTEST_Init(void)
{
	if (!s_started)
	{
		s_temp.begin();
		s_started = true;

		/* Plot the factory calibration once at startup to check */
		Plot_Int("ts_cal1", s_temp.getTsCal1());
		Plot_Int("ts_cal2", s_temp.getTsCal2());
		Plot_Int("vrefcal1", s_temp.getVrefCal());
	}
}

extern "C" void TEMPTEST_Run(void)
{
	/* Non-blocking -- Returns true on the call that captured a new sample. */
	if (s_temp.update())
	{
		if (s_temp.getCelsius() > 60)
		{
			return;
		}
		Plot_Float("temp_C",     s_temp.getCelsius());
		Plot_Int  ("ts_raw",     s_temp.getTsRaw());
		Plot_Int  ("vref_raw",   s_temp.getVrefRaw());
		Plot_Float("vdda_mV",    s_temp.getVddaMv());
		Plot_Float("temp_max_C", s_temp.getMaxCelsius());
		Plot_Float("temp_min_C", s_temp.getMinCelsius());
	}
}
