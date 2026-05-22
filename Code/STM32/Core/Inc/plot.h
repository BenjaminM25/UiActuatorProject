/*
 * plot.h
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */

#ifndef SRC_PLOT_H_
#define SRC_PLOT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ------------------------------------------------------------------ *
 * Sample-based API — preferred path. All channels in one call share
 * the same MCU timestamp, so they line up perfectly on the plot.
 *
 * Internally these scale floats to int32 (×1000 for "f3" variants,
 * ×10000 for "f4") and emit "%ld" rather than "%f", because %f in
 * an ISR is expensive on Cortex-M.
 *
 * On the host side, set the Teleplot multiplier accordingly, or
 * just read the integer values directly (e.g. 12345 → 12.345).
 * ------------------------------------------------------------------ */

/* 1-channel timestamped sample, value scaled by 1000.
 *   Emits ">name:T:V_scaled\n"  where V_scaled = (int32)(value * 1000) */
void Plot_Sample1f3(uint32_t t_ms, const char *n1, float v1);

/* 4-channel timestamped sample, all scaled by 1000.
 *   Emits four lines sharing the same timestamp. */
void Plot_Sample4f3(uint32_t t_ms,
                    const char *n1, float v1,
                    const char *n2, float v2,
                    const char *n3, float v3,
                    const char *n4, float v4);

/* ------------------------------------------------------------------ *
 * Legacy API — kept so existing call sites compile unchanged. These
 * stamp with HAL_GetTick() at call time and use the same ring.
 * ------------------------------------------------------------------ */
void Plot_Float(const char *name, float value);
void Plot_Int(const char *name, int32_t value);
void Plot_Multi(const char *a, int32_t va,
                const char *b, int32_t vb,
                const char *c, int32_t vc,
                const char *d, int32_t vd);

/* Non-blocking. Call from main loop — every iteration is fine. */
void Plot_Flush(void);

/* Diagnostics: number of bytes currently queued, and a counter of
 * samples dropped due to ring overflow since boot. Plot these to
 * see if you're losing data. */
uint16_t Plot_QueuedBytes(void);
uint32_t Plot_DroppedCount(void);

#ifdef __cplusplus
}
#endif



#endif /* SRC_PLOT_H_ */
