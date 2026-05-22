/*
 * TestCAN.c
 *
 *  Created on: 17. apr. 2026
 *      Author: Benjamin
 */

#include "testcan.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan2;

/* ── Shared state between ISR and main loop ── */
static volatile uint8_t  can_rx_pending = 0;
static char              can_rx_msg[128];
static uint16_t          can_rx_len     = 0;

/* ── TX diagnostic counters ── */
static uint32_t tx_ok_count   = 0;
static uint32_t tx_fail_count = 0;

/* ────────────────────────────────────────────────────────────
 *  TESTCAN_Init
 * ──────────────────────────────────────────────────────────── */
void TESTCAN_Init(void)
{
    FDCAN_FilterTypeDef filter;
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;   /* match value */
    filter.FilterID2    = 0x000;   /* mask: 0 = don't care -> accept all standard IDs */
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
        FDCAN_REJECT,           /* non-matching standard frames -> reject */
        FDCAN_REJECT,           /* non-matching extended frames -> reject */
        FDCAN_REJECT_REMOTE,    /* reject remote standard frames */
        FDCAN_REJECT_REMOTE);   /* reject remote extended frames */

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        uint8_t msg[] = ">can_status:0\r\n";  /* 0 = start failed */
        CDC_Transmit_FS(msg, sizeof(msg) - 1);
        return;
    }

    HAL_FDCAN_ActivateNotification(&hfdcan2,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    uint8_t msg[] = ">can_status:1\r\n";  /* 1 = ready */
    CDC_Transmit_FS(msg, sizeof(msg) - 1);
}

/* ────────────────────────────────────────────────────────────
 *  TESTCAN_Update
 *  Transmit one frame. Call periodically from the main loop.
 *  Sends counters in Teleplot format every ~100 calls so they
 *  can be plotted live without flooding the USB pipe.
 * ──────────────────────────────────────────────────────────── */
void TESTCAN_Update(void)
{
    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier          = 0x123;
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;

    uint8_t txData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    HAL_StatusTypeDef status =
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData);

    if (status == HAL_OK)
    {
        tx_ok_count++;
    }
    else
    {
        tx_fail_count++;
    }

    /* Rate-limit Teleplot output to roughly every 100 calls. */
    static uint32_t print_counter = 0;
    if ((++print_counter % 100u) == 0u)
    {
        uint32_t err    = HAL_FDCAN_GetError(&hfdcan2);
        uint32_t txfree = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
        uint32_t psr    = hfdcan2.Instance->PSR;

        char msg[192];
        int n = snprintf(msg, sizeof(msg),
                         ">tx_ok:%lu\r\n"
                         ">tx_fail:%lu\r\n"
                         ">tx_free:%lu\r\n"
                         ">can_err:%lu\r\n"
                         ">can_psr:%lu\r\n",
                         (unsigned long)tx_ok_count,
                         (unsigned long)tx_fail_count,
                         (unsigned long)txfree,
                         (unsigned long)err,
                         (unsigned long)psr);
        if (n > 0)
        {
            CDC_Transmit_FS((uint8_t *)msg, (uint16_t)n);
        }
    }
}

/* ────────────────────────────────────────────────────────────
 *  TESTCAN_Flush
 *  Call every main loop iteration to print pending RX messages.
 *  Safely transmits USB data outside of interrupt context.
 * ──────────────────────────────────────────────────────────── */
void TESTCAN_Flush(void)
{
    if (can_rx_pending)
    {
        can_rx_pending = 0;
        CDC_Transmit_FS((uint8_t *)can_rx_msg, can_rx_len);
    }
}

/* ────────────────────────────────────────────────────────────
 *  TESTCAN_RxCallback
 *  Called from interrupt context via HAL_FDCAN_RxFifo0Callback.
 *  Builds a Teleplot-formatted message containing:
 *    - the received ID (plottable)
 *    - the first data byte (plottable)
 *    - a text log line with the full frame for the console
 * ──────────────────────────────────────────────────────────── */
void TESTCAN_RxCallback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8] = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                               &rxHeader, rxData) != HAL_OK)
    {
        return;
    }

    int len = snprintf(can_rx_msg, sizeof(can_rx_msg),
                       ">rx_id:%lu\r\n"
                       ">rx_byte0:%u\r\n"
                       "RX 0x%03lX: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                       (unsigned long)rxHeader.Identifier,
                       (unsigned)rxData[0],
                       (unsigned long)rxHeader.Identifier,
                       rxData[0], rxData[1], rxData[2], rxData[3],
                       rxData[4], rxData[5], rxData[6], rxData[7]);

    if (len < 0)
    {
        return;
    }
    if ((size_t)len >= sizeof(can_rx_msg))
    {
        len = (int)sizeof(can_rx_msg) - 1;
    }

    can_rx_len     = (uint16_t)len;
    can_rx_pending = 1;
}

/* ────────────────────────────────────────────────────────────
 *  HAL_FDCAN_RxFifo0Callback
 *  HAL dispatches RX FIFO 0 events here (overrides the __weak
 *  default). Bridges to our TESTCAN_RxCallback.
 * ──────────────────────────────────────────────────────────── */
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0u)
//    {
//        TESTCAN_RxCallback(hfdcan);
//    }
//}
