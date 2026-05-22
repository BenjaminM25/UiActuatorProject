/*
 * CANBus.cpp
 *
 *  Created on: 19. mai 2026
 *      Author: Benjamin
 */

#include "CANBus.h"
#include <string.h>

/* ── ISR-dispatch instance pointer ──
 * HAL routes all RX FIFO 0 events through a single weak callback. We register
 * the active CANBus here in begin() and the C-linkage trampoline dispatches
 * to it. If multiple FDCAN peripherals are ever used, this can be extended to
 * a small lookup table keyed on hfdcan->Instance.
 */
static CANBus *s_canInstance = nullptr;


/* ──────────────────────────────────────────────────────────── */
CANBus::CANBus(FDCAN_HandleTypeDef *hfdcan)
    : m_hfdcan(hfdcan),
      m_rxHead(0), m_rxTail(0),
      m_rxCount(0), m_rxLost(0),
      m_lastRxValid(false),
      m_autoEcho(false), m_echoPending(0),
      m_txOk(0), m_txFail(0)
{
    memset(&m_lastRx, 0, sizeof(m_lastRx));
}

/* ──────────────────────────────────────────────────────────── */
bool CANBus::begin()
{
    /* Accept-all filter into RX FIFO 0 (mask = 0 -> don't care). */
    FDCAN_FilterTypeDef filter;
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;
    if (HAL_FDCAN_ConfigFilter(m_hfdcan, &filter) != HAL_OK)
    {
        return false;
    }

    HAL_FDCAN_ConfigGlobalFilter(m_hfdcan,
        FDCAN_REJECT,           /* non-matching standard frames */
        FDCAN_REJECT,           /* non-matching extended frames */
        FDCAN_REJECT_REMOTE,    /* remote standard */
        FDCAN_REJECT_REMOTE);   /* remote extended */

    if (HAL_FDCAN_Start(m_hfdcan) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ActivateNotification(m_hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        return false;
    }

    /* Register for ISR dispatch */
    s_canInstance = this;
    return true;
}

/* ──────────────────────────────────────────────────────────── */
void CANBus::update()
{
    /* Auto-echo is set in the ISR and acted upon here so we never call
     * HAL TX from interrupt context. Echo every pending frame in the ring. */
    if (m_autoEcho && m_echoPending)
    {
        Frame f;
        while (receive(f))
        {
            sendFrame(f.id, f.data, f.len, f.extended);
        }
        m_echoPending = 0;
    }
}

/* ── Transmit ─────────────────────────────────────────────── */
bool CANBus::send(uint32_t id, const uint8_t *data, uint8_t len)
{
    return sendFrame(id, data, len, /*extended=*/false);
}

bool CANBus::sendExtended(uint32_t id, const uint8_t *data, uint8_t len)
{
    return sendFrame(id, data, len, /*extended=*/true);
}

bool CANBus::sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool extended)
{
    if (len > MAX_DLC)
    {
        len = MAX_DLC;
    }

    /* DLC encoding for classic CAN: 0..8 map 1:1 to the FDCAN_DLC_BYTES_x macros. */
    static const uint32_t dlcTable[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    FDCAN_TxHeaderTypeDef hdr;
    hdr.Identifier          = id;
    hdr.IdType              = extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = dlcTable[len];
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    uint8_t buf[MAX_DLC] = {0};
    if (data != nullptr && len > 0)
    {
        memcpy(buf, data, len);
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(m_hfdcan, &hdr, buf) == HAL_OK)
    {
        m_txOk++;
        return true;
    }
    m_txFail++;
    return false;
}

/* ── Receive ──────────────────────────────────────────────── */
bool CANBus::available() const
{
    return m_rxHead != m_rxTail;
}

bool CANBus::receive(Frame &out)
{
    if (m_rxHead == m_rxTail)
    {
        return false;
    }
    /* Copy out of the volatile ring slot */
    const volatile Frame &src = m_rx[m_rxTail];
    out.id       = src.id;
    out.len      = src.len;
    out.extended = src.extended;
    for (uint8_t i = 0; i < MAX_DLC; i++)
    {
        out.data[i] = src.data[i];
    }
    m_rxTail = (uint8_t)((m_rxTail + 1) % RX_BUFFER_SIZE);
    return true;
}

/* ── Echo ─────────────────────────────────────────────────── */
bool CANBus::echoLast()
{
    if (!m_lastRxValid)
    {
        return false;
    }
    return sendFrame(m_lastRx.id, m_lastRx.data, m_lastRx.len, m_lastRx.extended);
}

/* ── Diagnostics ──────────────────────────────────────────── */
uint32_t CANBus::getError() const
{
    return HAL_FDCAN_GetError(m_hfdcan);
}

uint32_t CANBus::getPsr() const
{
    return m_hfdcan->Instance->PSR;
}

/* ── ISR path ─────────────────────────────────────────────── */
void CANBus::onRxFifo0(uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u)
    {
        return;
    }

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[MAX_DLC] = {0};

    if (HAL_FDCAN_GetRxMessage(m_hfdcan, FDCAN_RX_FIFO0,
                               &rxHeader, rxData) != HAL_OK)
    {
        return;
    }

    /* Reserve next slot in the ring. Drop if full. */
    uint8_t nextHead = (uint8_t)((m_rxHead + 1) % RX_BUFFER_SIZE);
    if (nextHead == m_rxTail)
    {
        m_rxLost++;
        return;
    }

    volatile Frame &slot = m_rx[m_rxHead];
    slot.id       = rxHeader.Identifier;
    slot.extended = (rxHeader.IdType == FDCAN_EXTENDED_ID);

    /* HAL stores DLC as the FDCAN_DLC_BYTES_x macro value, which for
     * classic CAN (0..8 bytes) is identical to the byte count. */
    uint8_t len = (uint8_t)rxHeader.DataLength;
    if (len > MAX_DLC) len = MAX_DLC;
    slot.len = len;

    for (uint8_t i = 0; i < MAX_DLC; i++)
    {
        slot.data[i] = (i < len) ? rxData[i] : 0;
    }

    m_rxHead = nextHead;
    m_rxCount++;

    /* Snapshot for echoLast() */
    m_lastRx.id       = slot.id;
    m_lastRx.extended = slot.extended;
    m_lastRx.len      = slot.len;
    for (uint8_t i = 0; i < MAX_DLC; i++)
    {
        m_lastRx.data[i] = slot.data[i];
    }
    m_lastRxValid = true;

    /* Flag deferred auto-echo work for the main loop */
    if (m_autoEcho)
    {
        m_echoPending = 1;
    }
}


/* ── C-linkage glue ───────────────────────────────────────── */
extern "C" void CANBus_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                       uint32_t RxFifo0ITs)
{
    (void)hfdcan;
    if (s_canInstance != nullptr)
    {
        s_canInstance->onRxFifo0(RxFifo0ITs);
    }
}

/* HAL weak override. We forward to the trampoline so application C code can
 * call CANBus_RxFifo0Callback directly too if it ever needs to (mirrors the
 * HallSensor_Callback pattern in this project). */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs)
{
    CANBus_RxFifo0Callback(hfdcan, RxFifo0ITs);
}


/* ──────────────────────────────────────────────────────────────────────────
 *  Single static instance + C-linkage wrappers
 *
 *  This lets main.c (a pure C translation unit) drive CAN without ever
 *  including the C++ class. Same pattern as MotorApp_* in this project.
 * ────────────────────────────────────────────────────────────────────────── */
extern "C" FDCAN_HandleTypeDef hfdcan2;   /* defined in fdcan.c */

CANBus can(&hfdcan2);

extern "C" uint8_t CANBus_Init(void)
{
    return can.begin() ? 1u : 0u;
}

extern "C" void CANBus_Update(void)
{
    can.update();
}

extern "C" uint8_t CANBus_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
    return can.send(id, data, len) ? 1u : 0u;
}

extern "C" uint8_t CANBus_SendExtended(uint32_t id, const uint8_t *data, uint8_t len)
{
    return can.sendExtended(id, data, len) ? 1u : 0u;
}

extern "C" uint8_t CANBus_EchoLast(void)
{
    return can.echoLast() ? 1u : 0u;
}

extern "C" void CANBus_EnableAutoEcho(uint8_t on)
{
    can.enableAutoEcho(on != 0u);
}

extern "C" uint8_t CANBus_Available(void)
{
    return can.available() ? 1u : 0u;
}

extern "C" uint8_t CANBus_Receive(uint32_t *id, uint8_t *data, uint8_t *len)
{
    CANBus::Frame f;
    if (!can.receive(f))
    {
        return 0u;
    }
    if (id  != nullptr) *id  = f.id;
    if (len != nullptr) *len = f.len;
    if (data != nullptr)
    {
        for (uint8_t i = 0; i < CANBus::MAX_DLC; i++)
        {
            data[i] = f.data[i];
        }
    }
    return 1u;
}

extern "C" uint32_t CANBus_GetTxOk(void)    { return can.getTxOk();    }
extern "C" uint32_t CANBus_GetTxFail(void)  { return can.getTxFail();  }
extern "C" uint32_t CANBus_GetRxCount(void) { return can.getRxCount(); }
extern "C" uint32_t CANBus_GetRxLost(void)  { return can.getRxLost();  }
extern "C" uint32_t CANBus_GetError(void)   { return can.getError();   }
extern "C" uint32_t CANBus_GetPsr(void)     { return can.getPsr();     }
