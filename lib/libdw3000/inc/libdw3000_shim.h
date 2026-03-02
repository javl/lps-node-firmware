/*
 * libdw3000_shim.h
 *
 * Compatibility shim that exposes the libdw1000 API surface expected by the
 * existing LPS algorithm files (uwb_twr_anchor.c, uwb_twr_tag.c,
 * uwb_tdoa_anchor2.c, uwb_tdoa_anchor3.c, uwb_sniffer.c) while forwarding
 * all calls to the Qorvo DW3000 decadriver.
 *
 * REQUIREMENTS
 * ============
 * The Qorvo DW3000 decadriver must be present in lib/libdw3000/decadriver/:
 *   deca_device.c
 *   deca_device_api.h
 *   deca_regs.h
 *   deca_vals.h
 *   deca_version.h
 *
 * See lib/libdw3000/README.md for instructions on obtaining these files.
 *
 * KEY DIFFERENCES FROM DW1000
 * ============================
 * - No device handle in the driver: DW3000 uses internal global state.
 *   dwDevice_t is kept as an application-level struct that stores callbacks
 *   and flags; it is NOT passed to dwt_* functions.
 * - Callbacks: DW3000 uses dwt_setcallbacks(); the shim registers internal
 *   forwarders and routes them to the DW1000-style dwHandler_t functions.
 * - Timestamps: 40-bit, ~15.65 ps/LSB — identical to DW1000. dwTime_t is
 *   source-compatible.
 * - Channel: DW3000 supports channels 5 and 9 only (no channel 2).
 * - No 16 MHz PRF, no 110 kbps data rate.
 * - Scheduled TX/RX: dwSetTxRxTime() maps to dwt_setdelayedtrxtime(); the
 *   start mode (DWT_START_TX_DELAYED / DWT_START_RX_DELAYED) is selected
 *   automatically inside dwStartTransmit() / dwStartReceive().
 */

#ifndef __LIBDW3000_SHIM_H__
#define __LIBDW3000_SHIM_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* Qorvo DW3000 public driver API.
 * The user must copy decadriver/ from foldedtoad/dwm3000 (or Qorvo SDK). */
#include "deca_device_api.h"

/* =========================================================================
 * 40-bit timestamp type (identical resolution to DW1000)
 * ========================================================================= */
typedef union dwTime_u {
    uint8_t  raw[5];
    uint64_t full;
    struct { uint32_t low32; uint8_t  high8; } __attribute__((packed));
    struct { uint8_t  low8;  uint32_t high32; } __attribute__((packed));
} dwTime_t;

/* =========================================================================
 * Device handle
 *
 * DW3000 driver is stateless from the caller's perspective (global pdw3000local
 * inside deca_device.c). dwDevice_t is kept purely to satisfy the existing
 * algorithm signatures and to carry application-level callbacks and flags.
 * ========================================================================= */
struct dwDevice_s;

typedef void (*dwHandler_t)(struct dwDevice_s *dev);

typedef struct dwDevice_s {
    void        *userdata;

    /* Flags managed by dwWaitForResponse() and dwSetTxRxTime() */
    bool         wait4resp;     /* request auto-RX after TX             */
    bool         useScheduled;  /* use delayed TX/RX start              */

    /* Frame length of the last received packet (FCS stripped).
     * Written inside the shim's internal cbRxOk callback;
     * read by dwGetDataLength().                                       */
    uint16_t     rxDataLength;

    /* DW1000-style application event callbacks */
    dwHandler_t  handleSent;
    dwHandler_t  handleReceived;
    dwHandler_t  handleReceiveTimeout;
    dwHandler_t  handleReceiveFailed;

    /* Kept for header compatibility with code that inspects antennaDelay */
    dwTime_t     antennaDelay;
} dwDevice_t;

/* =========================================================================
 * Minimal dwOps_t — retained only so headers that reference it still compile.
 * DW3000 driver does NOT use a vtable; SPI access is via global writetospi/
 * readfromspi functions defined in src/dwOps.c.
 * ========================================================================= */
typedef enum { dwSpiSpeedLow, dwSpiSpeedHigh } dwSpiSpeed_t;

typedef struct dwOps_s {
    void (*spiRead)    (struct dwDevice_s *, const void *, size_t, void *, size_t);
    void (*spiWrite)   (struct dwDevice_s *, const void *, size_t, const void *, size_t);
    void (*spiSetSpeed)(struct dwDevice_s *, dwSpiSpeed_t);
    void (*delayms)    (struct dwDevice_s *, unsigned int);
    void (*reset)      (struct dwDevice_s *);
} dwOps_t;

/* =========================================================================
 * Shim initialisation
 *
 * Must be called once, AFTER dwt_initialise() succeeds.
 * Stores 'dev' as the global device pointer and registers the shim's
 * internal ISR forwarders with dwt_setcallbacks().
 * 'ops' is accepted for source compatibility but ignored at runtime.
 * ========================================================================= */
void dwInit(dwDevice_t *dev, dwOps_t *ops);

/* =========================================================================
 * Callback attachment
 * ========================================================================= */
void dwAttachSentHandler           (dwDevice_t *dev, dwHandler_t h);
void dwAttachReceivedHandler       (dwDevice_t *dev, dwHandler_t h);
void dwAttachReceiveTimeoutHandler (dwDevice_t *dev, dwHandler_t h);
void dwAttachReceiveFailedHandler  (dwDevice_t *dev, dwHandler_t h);

/* =========================================================================
 * Interrupt forwarding — call from the FreeRTOS UWB task
 * ========================================================================= */
void dwHandleInterrupt(dwDevice_t *dev);

/* =========================================================================
 * Timestamp accessors
 * ========================================================================= */
void dwGetTransmitTimestamp    (dwDevice_t *dev, dwTime_t *t);
void dwGetReceiveTimestamp     (dwDevice_t *dev, dwTime_t *t);
/* dwGetRawReceiveTimestamp: on DW3000 the "raw" timestamp is the same as the
 * adjusted one; no separate FPGA LDE correction step exists.              */
void dwGetRawReceiveTimestamp  (dwDevice_t *dev, dwTime_t *t);
void dwGetSystemTimestamp      (dwDevice_t *dev, dwTime_t *t);
/* dwCorrectTimestamp: no-op on DW3000 — timestamp is already corrected.  */
void dwCorrectTimestamp        (dwDevice_t *dev, dwTime_t *t);

/* =========================================================================
 * RX/TX data
 * ========================================================================= */
int  dwGetDataLength(dwDevice_t *dev);
void dwGetData      (dwDevice_t *dev, uint8_t *data, int n);
/* dwSetData: writes TX payload; internally calls dwt_writetxdata +
 * dwt_writetxfctrl (frame length = n + 2 FCS bytes, ranging = 1).        */
void dwSetData      (dwDevice_t *dev, const uint8_t *data, int n);

/* =========================================================================
 * Radio state control
 * ========================================================================= */
/* dwNewReceive: forces radio to IDLE (dwt_forcetrxoff).                   */
void dwNewReceive    (dwDevice_t *dev);
/* dwSetDefaults: no-op on DW3000 (radio is fully re-configured on each
 * start via dwt_configure; no per-packet default state needed).           */
void dwSetDefaults   (dwDevice_t *dev);
/* dwStartReceive: enables RX; uses delayed mode if dwSetTxRxTime was
 * called since the last dwNewReceive, otherwise immediate mode.           */
void dwStartReceive  (dwDevice_t *dev);

/* dwNewTransmit: forces radio to IDLE (dwt_forcetrxoff).                 */
void dwNewTransmit   (dwDevice_t *dev);
/* dwWaitForResponse: when 'enable', dwStartTransmit will add
 * DWT_RESPONSE_EXPECTED so the receiver turns on immediately after TX.   */
void dwWaitForResponse(dwDevice_t *dev, bool enable);
/* dwStartTransmit: starts TX; selects delayed / immediate and
 * RESPONSE_EXPECTED based on flags set by dwSetTxRxTime /
 * dwWaitForResponse.                                                      */
void dwStartTransmit (dwDevice_t *dev);

/* dwIdle: alias for dwt_forcetrxoff.                                      */
void dwIdle(dwDevice_t *dev);

/* =========================================================================
 * Scheduled TX/RX
 *
 * Programs the DX_TIME register via dwt_setdelayedtrxtime().
 * futureTime must have the 9 LSBs zeroed (use adjustTxRxTime() as in the
 * existing algorithm code).  The shim converts via:
 *   schedTime = (uint32_t)(futureTime.full >> 8)
 * which maps the 40-bit DW counter to the 32-bit DX_TIME register.
 * ========================================================================= */
void dwSetTxRxTime(dwDevice_t *dev, dwTime_t futureTime);

/* =========================================================================
 * Receive timeout
 *
 * timeout is passed directly to dwt_setrxtimeout() as microseconds.
 * The DW1000 PAC-based timeout unit (~1 µs) is close enough to the DW3000
 * µs unit for the algorithm timeouts (300, 800) to remain functional.
 * Pass 0 to disable the timeout.
 * ========================================================================= */
void dwSetReceiveWaitTimeout(dwDevice_t *dev, uint16_t timeout);

/* dwWriteSystemConfigurationRegister: no-op on DW3000.
 * DW3000 applies SYS_CFG changes atomically via dwt_configure();
 * no explicit "commit" command is needed mid-flight.                      */
void dwWriteSystemConfigurationRegister(dwDevice_t *dev);

/* =========================================================================
 * Error string
 * ========================================================================= */
char *dwStrError(int errCode);

#endif /* __LIBDW3000_SHIM_H__ */
