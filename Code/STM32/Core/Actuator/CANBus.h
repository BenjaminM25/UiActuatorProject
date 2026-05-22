/*
 * CANBus.h
 *
 *  Created on: 19. mai 2026
 *      Author: Benjamin
 */


/*  Classic CAN driver wrapping the STM32 FDCAN peripheral in CAN 2.0B mode.
 *  RX is interrupt-driven into a small ring buffer; TX is non-blocking and
 *  pushes frames into the hardware TX FIFO.
 *
 *  Typical use:
 *      CANBus can(&hfdcan2);
 *      can.begin();                // sets filter, starts peripheral, enables IRQ
 *      can.send(0x123, data, 8);   // transmit
 *      can.enableAutoEcho(true);   // optional: bounce every RX back out
 *      ...
 *      can.update();               // call from main loop
 */

#ifndef ACTUATOR_CANBUS_H_
#define ACTUATOR_CANBUS_H_

#include "main.h"
#include "fdcan.h"
#include <stdint.h>
#include <stdbool.h>
#include "CANBus.h"

#ifdef __cplusplus

class CANBus
{
public:
    static constexpr uint8_t  RX_BUFFER_SIZE = 8;   /* power-of-two preferred */
    static constexpr uint8_t  MAX_DLC        = 8;   /* classic CAN payload */

    /** One received classic-CAN frame. */
    struct Frame
    {
        uint32_t id;                 /* 11-bit standard or 29-bit extended */
        uint8_t  data[MAX_DLC];
        uint8_t  len;                /* 0..8 */
        bool     extended;           /* true = 29-bit ID */
    };

    /**
     * @param hfdcan  FDCAN handle (already initialised by CubeMX).
     */
    explicit CANBus(FDCAN_HandleTypeDef *hfdcan);

    /** Configure accept-all filter, start the peripheral, enable RX FIFO 0
     *  notifications and register this instance for ISR dispatch.
     *  @return true on success. */
    bool begin();

    /** Service the RX ring buffer. Call from the main loop. Performs the
     *  deferred auto-echo TX (if enabled) outside of ISR context. */
    void update();

    /* ── Transmit ───────────────────────────────────────────── */

    /** Queue a standard (11-bit) frame for transmission.
     *  @param id    11-bit identifier
     *  @param data  payload bytes (may be nullptr if len == 0)
     *  @param len   payload length, 0..8
     *  @return true if the frame was accepted by the TX FIFO. */
    bool send(uint32_t id, const uint8_t *data, uint8_t len);

    /** Queue an extended (29-bit) frame for transmission. */
    bool sendExtended(uint32_t id, const uint8_t *data, uint8_t len);

    /* ── Receive ────────────────────────────────────────────── */

    /** True if at least one frame is waiting in the RX ring. */
    bool available() const;

    /** Pop the oldest frame from the RX ring.
     *  @param out  destination frame
     *  @return true if a frame was returned. */
    bool receive(Frame &out);

    /* ── Echo ───────────────────────────────────────────────── */

    /** Re-transmit the most recently received frame on the same ID. Does
     *  not consume frames from the RX ring — use receive() for that.
     *  @return true if a frame was available and the echo was queued. */
    bool echoLast();

    /** When enabled, every received frame is automatically re-transmitted
     *  on the next update() call. Note: this duplicates messages on the
     *  bus and can create a feedback loop if two echoing nodes share the
     *  same arbitration ID — use distinct IDs for request/response. */
    void enableAutoEcho(bool on) { m_autoEcho = on; }
    bool autoEchoEnabled() const { return m_autoEcho; }

    /* ── Diagnostics ────────────────────────────────────────── */

    uint32_t getTxOk()    const { return m_txOk; }
    uint32_t getTxFail()  const { return m_txFail; }
    uint32_t getRxCount() const { return m_rxCount; }
    uint32_t getRxLost()  const { return m_rxLost; }
    uint32_t getError()   const;
    uint32_t getPsr()     const;

    /** ISR entry point — invoked by the C-linkage trampoline below.
     *  Should not be called from user code. */
    void onRxFifo0(uint32_t RxFifo0ITs);

private:
    bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool extended);

    FDCAN_HandleTypeDef *m_hfdcan;

    /* RX ring buffer — written from ISR, read from main loop */
    volatile Frame    m_rx[RX_BUFFER_SIZE];
    volatile uint8_t  m_rxHead;     /* next slot to write (ISR) */
    volatile uint8_t  m_rxTail;     /* next slot to read (main) */
    volatile uint32_t m_rxCount;
    volatile uint32_t m_rxLost;     /* dropped because ring was full */

    /* Last received frame, for echoLast() */
    Frame    m_lastRx;
    bool     m_lastRxValid;

    /* Auto-echo bookkeeping — set in ISR, consumed in update() */
    bool              m_autoEcho;
    volatile uint8_t  m_echoPending;

    /* TX counters */
    uint32_t m_txOk;
    uint32_t m_txFail;
};

#endif /* __cplusplus */


/* ──────────────────────────────────────────────────────────────────────────
 *  C-visible interface — safe to include from C files (e.g. main.c).
 *
 *  A single static CANBus instance bound to hfdcan2 lives inside CANBus.cpp.
 *  These wrappers let C code drive it without touching the C++ class, mirroring
 *  the MotorApp_*  /  HallSensor_Callback pattern used elsewhere in this project.
 * ────────────────────────────────────────────────────────────────────────── */
#ifdef __cplusplus
extern CANBus can;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Configure filter, start FDCAN2, enable RX FIFO 0 notifications.
 *  Call once after MX_FDCAN2_Init(). Returns 1 on success, 0 on failure. */
uint8_t  CANBus_Init(void);

/** Service the RX ring (drives auto-echo TX). Call from while(1). */
void     CANBus_Update(void);

/** Queue a standard (11-bit) frame. Returns 1 on success, 0 on failure. */
uint8_t  CANBus_Send(uint32_t id, const uint8_t *data, uint8_t len);

/** Queue an extended (29-bit) frame. */
uint8_t  CANBus_SendExtended(uint32_t id, const uint8_t *data, uint8_t len);

/** Re-transmit the most recently received frame. */
uint8_t  CANBus_EchoLast(void);

/** Enable / disable automatic echo of every received frame. */
void     CANBus_EnableAutoEcho(uint8_t on);

/** True (1) if at least one frame is waiting. */
uint8_t  CANBus_Available(void);

/** Pop one frame from the RX ring into the caller's buffers.
 *  @param id    out: identifier
 *  @param data  out: payload buffer (must hold at least 8 bytes)
 *  @param len   out: payload length (0..8)
 *  @return 1 if a frame was returned, 0 if the ring was empty. */
uint8_t  CANBus_Receive(uint32_t *id, uint8_t *data, uint8_t *len);

/* ── Diagnostics ── */
uint32_t CANBus_GetTxOk(void);
uint32_t CANBus_GetTxFail(void);
uint32_t CANBus_GetRxCount(void);
uint32_t CANBus_GetRxLost(void);
uint32_t CANBus_GetError(void);
uint32_t CANBus_GetPsr(void);

/** ISR trampoline — already wired via the HAL weak override. Exposed here
 *  in case you want to call it from a hand-written interrupt path. */
void     CANBus_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_CANBUS_H_ */
