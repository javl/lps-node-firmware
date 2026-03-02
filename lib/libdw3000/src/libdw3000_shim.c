/*
 * libdw3000_shim.c
 *
 * Implements the libdw1000-compatible API used by the existing LPS algorithm
 * files by forwarding calls to the Qorvo DW3000 decadriver.
 *
 * The single global dwDevice_t pointer (g_dev) is set by dwInit() which must
 * be called after dwt_initialise() succeeds.  All shim functions are safe to
 * call from the single uwbTask FreeRTOS task context used by this firmware.
 */

#include "libdw3000_shim.h"

#include <string.h>
#include <stdio.h>

/* dwt_readsystime is implemented in deca_device.c but not declared in
 * deca_device_api.h — forward-declare it here so the shim can call it. */
extern void dwt_readsystime(uint8_t *timestamp);

/* =========================================================================
 * Module-private state
 * ========================================================================= */

/* The application's single device handle.  Set by dwInit(). */
static dwDevice_t *g_dev = NULL;

/* =========================================================================
 * Internal DW3000 ISR callbacks
 *
 * These are registered with dwt_setcallbacks() inside dwInit() and forward
 * each event to the corresponding DW1000-style dwHandler_t stored in g_dev.
 * ========================================================================= */

static void _cb_txok(const dwt_cb_data_t *cb)
{
    (void)cb;
    if (g_dev && g_dev->handleSent) {
        g_dev->handleSent(g_dev);
    }
}

static void _cb_rxok(const dwt_cb_data_t *cb)
{
    if (g_dev) {
        /* datalength includes 2 FCS bytes; strip them so dwGetDataLength()
         * returns the application-visible payload length, matching DW1000
         * behaviour.                                                       */
        g_dev->rxDataLength = (cb->datalength > 2u) ? (cb->datalength - 2u) : 0u;
        if (g_dev->handleReceived) {
            g_dev->handleReceived(g_dev);
        }
    }
}

static void _cb_rxto(const dwt_cb_data_t *cb)
{
    (void)cb;
    if (g_dev && g_dev->handleReceiveTimeout) {
        g_dev->handleReceiveTimeout(g_dev);
    }
}

static void _cb_rxerr(const dwt_cb_data_t *cb)
{
    (void)cb;
    if (g_dev && g_dev->handleReceiveFailed) {
        g_dev->handleReceiveFailed(g_dev);
    }
}

/* =========================================================================
 * Shim initialisation
 * ========================================================================= */

void dwInit(dwDevice_t *dev, dwOps_t *ops)
{
    (void)ops;  /* vtable is unused; DW3000 uses direct platform functions */

    g_dev = dev;

    /* Zero application-level state */
    dev->wait4resp    = false;
    dev->useScheduled = false;
    dev->rxDataLength = 0;

    /* Register internal forwarder callbacks with the DW3000 driver.
     * cbSPIErr and cbSPIRdy are not used by this firmware.              */
    dwt_setcallbacks(_cb_txok, _cb_rxok, _cb_rxto, _cb_rxerr, NULL, NULL);
}

/* =========================================================================
 * Callback attachment
 * ========================================================================= */

void dwAttachSentHandler(dwDevice_t *dev, dwHandler_t h)           { dev->handleSent           = h; }
void dwAttachReceivedHandler(dwDevice_t *dev, dwHandler_t h)        { dev->handleReceived        = h; }
void dwAttachReceiveTimeoutHandler(dwDevice_t *dev, dwHandler_t h)  { dev->handleReceiveTimeout  = h; }
void dwAttachReceiveFailedHandler(dwDevice_t *dev, dwHandler_t h)   { dev->handleReceiveFailed   = h; }

/* =========================================================================
 * Interrupt forwarding
 * ========================================================================= */

void dwHandleInterrupt(dwDevice_t *dev)
{
    (void)dev;
    /* dwt_isr() reads the DW3000 SYS_STATUS register and dispatches to
     * whichever internal callback applies (_cb_txok, _cb_rxok, …).       */
    dwt_isr();
}

/* =========================================================================
 * Timestamp accessors
 * ========================================================================= */

void dwGetTransmitTimestamp(dwDevice_t *dev, dwTime_t *t)
{
    (void)dev;
    dwt_readtxtimestamp(t->raw);
}

void dwGetReceiveTimestamp(dwDevice_t *dev, dwTime_t *t)
{
    (void)dev;
    dwt_readrxtimestamp(t->raw);
}

void dwGetRawReceiveTimestamp(dwDevice_t *dev, dwTime_t *t)
{
    (void)dev;
    /* DW3000 does not separate "raw" from "adjusted" timestamps; the
     * value returned by dwt_readrxtimestamp() is already the best
     * estimate of the actual RX time including internal corrections.    */
    dwt_readrxtimestamp(t->raw);
}

void dwGetSystemTimestamp(dwDevice_t *dev, dwTime_t *t)
{
    (void)dev;
    dwt_readsystime(t->raw);
}

void dwCorrectTimestamp(dwDevice_t *dev, dwTime_t *t)
{
    /* No-op: DW3000 timestamps do not require a separate software
     * correction step equivalent to DW1000's LDE phase correction.     */
    (void)dev;
    (void)t;
}

/* =========================================================================
 * RX/TX data
 * ========================================================================= */

int dwGetDataLength(dwDevice_t *dev)
{
    return (int)(dev->rxDataLength);
}

void dwGetData(dwDevice_t *dev, uint8_t *data, int n)
{
    (void)dev;
    dwt_readrxdata(data, (uint16_t)n, 0);
}

void dwSetData(dwDevice_t *dev, const uint8_t *data, int n)
{
    (void)dev;
    /* Write TX payload bytes into the DW3000 TX buffer at offset 0.
     * Frame control length = data bytes + 2 FCS bytes (auto-appended).
     * ranging = 1: marks the PHR ranging bit, enabling RX/TX timestamping
     * needed by all TWR and TDoA protocols in this firmware.             */
    dwt_writetxdata((uint16_t)n, (uint8_t *)data, 0);
    dwt_writetxfctrl((uint16_t)(n + 2), 0, 1 /* ranging */);
}

/* =========================================================================
 * Radio control
 * ========================================================================= */

void dwNewReceive(dwDevice_t *dev)
{
    /* Force the radio back to IDLE so a clean RX can be set up.
     * This is safe to call even when the radio is already idle.         */
    dwt_forcetrxoff();
    dev->useScheduled = false;
}

void dwSetDefaults(dwDevice_t *dev)
{
    /* No-op: DW3000 has no per-packet "apply defaults" step.  The radio
     * configuration is fully set by dwt_configure() at init time and
     * persists across packets.                                           */
    (void)dev;
}

void dwStartReceive(dwDevice_t *dev)
{
    int mode = dev->useScheduled ? DWT_START_RX_DELAYED : DWT_START_RX_IMMEDIATE;
    dev->useScheduled = false;

    if (dwt_rxenable(mode) != DWT_SUCCESS) {
        /* Delayed start failed (past due); fall back to immediate RX.   */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

void dwNewTransmit(dwDevice_t *dev)
{
    dwt_forcetrxoff();
    dev->wait4resp    = false;
    dev->useScheduled = false;
}

void dwWaitForResponse(dwDevice_t *dev, bool enable)
{
    dev->wait4resp = enable;
}

void dwStartTransmit(dwDevice_t *dev)
{
    uint8_t mode = dev->useScheduled ? DWT_START_TX_DELAYED
                                     : DWT_START_TX_IMMEDIATE;
    if (dev->wait4resp) {
        mode |= DWT_RESPONSE_EXPECTED;
    }

    dev->useScheduled = false;
    dev->wait4resp    = false;

    if (dwt_starttx(mode) != DWT_SUCCESS) {
        /* Delayed TX missed its window; attempt immediate fallback.
         * The algorithm will handle any resulting timing error.          */
        dwt_starttx(DWT_START_TX_IMMEDIATE |
                    (dev->wait4resp ? DWT_RESPONSE_EXPECTED : 0));
    }
}

void dwIdle(dwDevice_t *dev)
{
    (void)dev;
    dwt_forcetrxoff();
}

/* =========================================================================
 * Scheduled TX/RX
 * ========================================================================= */

void dwSetTxRxTime(dwDevice_t *dev, dwTime_t futureTime)
{
    /* DX_TIME register in DW3000 holds bits [39:9] of the 40-bit system
     * time counter, packed into a 32-bit value.  This is equivalent to
     * right-shifting the 40-bit timestamp by 8 bits.
     *
     * The algorithm code always calls adjustTxRxTime() before this
     * function to zero the 9 LSBs, so the mapping is exact.             */
    uint32_t schedTime = (uint32_t)(futureTime.full >> 8);
    dwt_setdelayedtrxtime(schedTime);
    dev->useScheduled = true;
}

/* =========================================================================
 * Receive timeout
 * ========================================================================= */

void dwSetReceiveWaitTimeout(dwDevice_t *dev, uint16_t timeout)
{
    (void)dev;
    /* DW3000 dwt_setrxtimeout() takes microseconds (uint32_t).
     * DW1000 timeout units were approximately 1 µs each so the value
     * can be passed through directly.  Pass 0 to disable.               */
    dwt_setrxtimeout((uint32_t)timeout);
}

void dwWriteSystemConfigurationRegister(dwDevice_t *dev)
{
    /* No-op: DW3000 applies SYS_CFG changes via dwt_configure(); there is
     * no separate "commit" needed mid-operation.                          */
    (void)dev;
}

/* =========================================================================
 * Error string
 * ========================================================================= */

char *dwStrError(int errCode)
{
    switch (errCode) {
        case DWT_SUCCESS: return "OK";
        case DWT_ERROR:   return "DW3000 error";
        default:          return "Unknown DW3000 error";
    }
}
