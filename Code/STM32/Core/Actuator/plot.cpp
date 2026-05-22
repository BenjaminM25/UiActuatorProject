/*
 * plot.cpp
 *
 *  Created on: 18. apr. 2026
 *      Author: Benjamin
 */


/*
 * plot.cpp
 *
 *  Byte-ring-buffer Teleplot streamer over USB CDC.
 *
 *  Producer-safe: the ring write path reads plot_tail once into a local
 *  and writes plot_head with a single store, so a higher-priority
 *  context (the flusher in main loop) sees a consistent view.
 *
 *  Single producer assumed (the TIM6 ISR). If you ever call Plot_*
 *  from multiple ISR priorities, wrap the append with a critical
 *  section.
 */

#include "plot.h"
#include "usbd_cdc_if.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* Power-of-two size — wrap becomes a mask. 4 KB is comfortable for
 * 1 kHz × ~60-byte lines with main-loop drain latency in the ms range. */
#define PLOT_BUF_SIZE   4096u
#define PLOT_BUF_MASK   (PLOT_BUF_SIZE - 1u)

/* Per-Flush transmit chunk cap. USB FS bulk packets are 64 B, but the
 * CDC class driver buffers internally — 256 B chunks give good
 * throughput without monopolising the USB stack for too long. */
#define PLOT_TX_CHUNK   256u

static char              plot_buf[PLOT_BUF_SIZE];
static volatile uint16_t plot_head = 0;   /* producer writes */
static volatile uint16_t plot_tail = 0;   /* consumer (flush) reads */
static volatile uint32_t plot_dropped = 0;

/* ------------------------------------------------------------------ *
 * Ring helpers
 * ------------------------------------------------------------------ */

static inline uint16_t ring_used(uint16_t head, uint16_t tail)
{
    return (uint16_t)((head - tail) & PLOT_BUF_MASK);
}

static inline uint16_t ring_free(uint16_t head, uint16_t tail)
{
    /* -1 so head == tail always means empty, never full */
    return (uint16_t)((tail - head - 1u) & PLOT_BUF_MASK);
}

/* Append a formatted block of `len` bytes from `src` to the ring.
 * Atomic-ish: either the whole block goes in or none of it does
 * (so a reader never sees a half-line). Returns true on success. */
static bool ring_append(const char *src, uint16_t len)
{
    uint16_t head = plot_head;
    uint16_t tail = plot_tail;          /* snapshot once */

    if (ring_free(head, tail) < len) {
        plot_dropped++;
        return false;
    }

    /* Copy in up to two segments (wrap) */
    uint16_t first = PLOT_BUF_SIZE - head;
    if (first > len) first = len;
    memcpy(&plot_buf[head], src, first);

    uint16_t rest = len - first;
    if (rest > 0) {
        memcpy(&plot_buf[0], src + first, rest);
    }

    /* Single store publishes the new head to the consumer */
    plot_head = (uint16_t)((head + len) & PLOT_BUF_MASK);
    return true;
}

/* ------------------------------------------------------------------ *
 * Sample-based API (preferred — call from the 1 kHz ISR)
 * ------------------------------------------------------------------ */

void Plot_Sample1f3(uint32_t t_ms, const char *n1, float v1)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
        ">%s:%lu:%.3f\n",
        n1, (unsigned long)t_ms, (double)v1);
    if (n > 0) ring_append(buf, (uint16_t)n);
}

void Plot_Sample4f3(uint32_t t_ms,
                    const char *n1, float v1,
                    const char *n2, float v2,
                    const char *n3, float v3,
                    const char *n4, float v4)
{
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        ">%s:%lu:%.3f\n>%s:%lu:%.3f\n>%s:%lu:%.3f\n>%s:%lu:%.3f\n",
        n1, (unsigned long)t_ms, (double)v1,
        n2, (unsigned long)t_ms, (double)v2,
        n3, (unsigned long)t_ms, (double)v3,
        n4, (unsigned long)t_ms, (double)v4);
    if (n > 0) ring_append(buf, (uint16_t)n);
}

/* ------------------------------------------------------------------ *
 * Legacy API — kept compatible so existing call sites still work.
 * These stamp with HAL_GetTick() at the moment of the call.
 *
 * NOTE: HAL_GetTick() is safe from a higher-priority ISR only if the
 * SysTick priority is higher (lower numeric) than the caller. In your
 * default Cube setup SysTick is priority 15 (lowest), so calling
 * HAL_GetTick() from TIM6 (typical pri 5) will return a stale tick
 * that doesn't advance. Use Plot_Sample*() with an explicit timestamp
 * from the ISR, or bump SysTick priority to 0.
 * ------------------------------------------------------------------ */

void Plot_Float(const char *name, float value)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
        ">%s:%lu:%.3f\n",
        name, (unsigned long)HAL_GetTick(), (double)value);
    if (n > 0) ring_append(buf, (uint16_t)n);
}

void Plot_Int(const char *name, int32_t value)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
        ">%s:%lu:%ld\n",
        name, (unsigned long)HAL_GetTick(), (long)value);
    if (n > 0) ring_append(buf, (uint16_t)n);
}

void Plot_Multi(const char *a, int32_t va,
                const char *b, int32_t vb,
                const char *c, int32_t vc,
                const char *d, int32_t vd)
{
    char buf[192];
    uint32_t t = HAL_GetTick();
    int n = snprintf(buf, sizeof(buf),
        ">%s:%lu:%ld\n>%s:%lu:%ld\n>%s:%lu:%ld\n>%s:%lu:%ld\n",
        a, (unsigned long)t, (long)va,
        b, (unsigned long)t, (long)vb,
        c, (unsigned long)t, (long)vc,
        d, (unsigned long)t, (long)vd);
    if (n > 0) ring_append(buf, (uint16_t)n);
}

/* ------------------------------------------------------------------ *
 * Non-blocking flush.
 *
 * Hands the largest contiguous chunk (capped at PLOT_TX_CHUNK) to
 * CDC_Transmit_FS. If USB is busy, returns immediately — the next
 * main-loop iteration will retry. NEVER blocks, NEVER calls HAL_Delay.
 * ------------------------------------------------------------------ */

void Plot_Flush(void)
{
    uint16_t head = plot_head;          /* snapshot */
    uint16_t tail = plot_tail;
    if (head == tail) return;           /* empty */

    /* Contiguous bytes from tail to either head or end-of-buffer */
    uint16_t len;
    if (head > tail) {
        len = (uint16_t)(head - tail);
    } else {
        len = (uint16_t)(PLOT_BUF_SIZE - tail);
    }
    if (len > PLOT_TX_CHUNK) len = PLOT_TX_CHUNK;

    if (CDC_Transmit_FS((uint8_t *)&plot_buf[tail], len) == USBD_OK) {
        plot_tail = (uint16_t)((tail + len) & PLOT_BUF_MASK);
    }
    /* else: USB busy — drop out, try again next main-loop pass. */
}

/* ------------------------------------------------------------------ *
 * Diagnostics
 * ------------------------------------------------------------------ */

uint16_t Plot_QueuedBytes(void)
{
    return ring_used(plot_head, plot_tail);
}

uint32_t Plot_DroppedCount(void)
{
    return plot_dropped;
}
