/*
 * uwb_tdoa_tag.c — TDOA2 passive tag
 *
 * Receives TDOA2 anchor broadcast packets, extracts per-anchor RX timestamps
 * and anchor positions from the LPP payload.
 *
 * Mode switch (set via build flag -D TDOA_TAG_OFFLOAD=0|1):
 *   TDOA_TAG_OFFLOAD 1  →  gather measurements, print to serial for PC offload
 *   TDOA_TAG_OFFLOAD 0  →  on-device Chan-Ho closed-form 2D solver (Chan & Ho 1994)
 *
 * Per-frame data collected for each anchor i (0..N_ANCHORS-1):
 *   rx_time        — tag-local DW3000 RX timestamp (40-bit, ~15.65 ps/tick)
 *   anchor_tx_time — anchor's own TX timestamp copied from the packet (40-bit)
 *   pos[3]         — anchor position in metres, from LPP_SHORT_ANCHOR_POSITION
 *
 * TDOA between anchors i and j (for the PC solver):
 *   tdoa_ij = (rx_time_i - rx_time_j) - (anchor_tx_time_i - anchor_tx_time_j)
 *   distance_diff = tdoa_ij / (499.2e6 * 128) * 299792458   [metres]
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "uwb.h"
#include "libdw3000_shim.h"
#include "mac.h"
#include "lpp.h"
#include "led.h"

/* --------------------------------------------------------------------------
 * Compile-time mode switch
 * Default to offload; override with -D TDOA_TAG_OFFLOAD=0 for on-device solve
 * -------------------------------------------------------------------------- */
#ifndef TDOA_TAG_OFFLOAD
#define TDOA_TAG_OFFLOAD 1
#endif

/* --------------------------------------------------------------------------
 * Packet format — must match NSLOTS in uwb_tdoa_anchor2.c
 * -------------------------------------------------------------------------- */
#define PACKET_TYPE_TDOA2  0x22
#define NSLOTS             4   /* frame slots; must equal anchor NSLOTS */
#define N_ANCHORS          3   /* active anchors (IDs 0..N_ANCHORS-1)   */
#define TS_TX_SIZE         5   /* bytes per timestamp in rangePacket_t   */
#define MASK_40BIT         0xFFFFFFFFFFULL

typedef struct {
    uint8_t  type;
    uint8_t  pid[NSLOTS];
    uint8_t  timestamps[NSLOTS][TS_TX_SIZE];
    uint16_t distances[NSLOTS];
} __attribute__((packed)) rangePacket_t;

/* Offsets of optional LPP block that follows rangePacket_t */
#define LPP_HEADER_OFF  (sizeof(rangePacket_t))
#define LPP_TYPE_OFF    (sizeof(rangePacket_t) + 1)
#define LPP_PAYLOAD_OFF (sizeof(rangePacket_t) + 2)

/* --------------------------------------------------------------------------
 * Per-frame observation for one anchor
 * -------------------------------------------------------------------------- */
typedef struct {
    bool     valid;
    uint64_t rx_time;         /* tag's local DW3000 clock (40-bit ticks) */
    uint64_t anchor_tx_time;  /* anchor's own TX timestamp (anchor clock, 40-bit) */
    uint64_t anchor_rx_ref;   /* anchor's RX of anchor 0 (anchor clock, 40-bit).
                                 For anchor 0 itself this equals anchor_tx_time. */
    float    pos[3];          /* anchor position x,y,z in metres         */
} anchor_obs_t;

/* Observations accumulated over the current frame */
static anchor_obs_t obs[NSLOTS];
static int          obs_count;

/* Persistent anchor positions — updated whenever an LPP block is seen.
 * Reused in frames where the anchor omits the LPP payload. */
static float  anchor_pos[NSLOTS][3];
static bool   anchor_pos_known[NSLOTS];

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */
static void clear_frame(void)
{
    for (int i = 0; i < NSLOTS; i++) {
        obs[i].valid = false;
    }
    obs_count = 0;
}

static bool try_extract_lpp_pos(const uint8_t *payload, int payload_len,
                                 int anchor_id)
{
    int needed = (int)(LPP_PAYLOAD_OFF + sizeof(struct lppShortAnchorPosition_s));
    if (payload_len < needed) return false;
    if (payload[LPP_HEADER_OFF] != SHORT_LPP)               return false;
    if (payload[LPP_TYPE_OFF]   != LPP_SHORT_ANCHOR_POSITION) return false;

    const struct lppShortAnchorPosition_s *lpp =
        (const struct lppShortAnchorPosition_s *)&payload[LPP_PAYLOAD_OFF];
    memcpy(anchor_pos[anchor_id], lpp->position, sizeof(lpp->position));
    anchor_pos_known[anchor_id] = true;
    return true;
}

/* --------------------------------------------------------------------------
 * Emit — called once per frame when anchor 0 starts a new frame
 * -------------------------------------------------------------------------- */
#if TDOA_TAG_OFFLOAD

static void emit_measurements(void)
{
    /* Need at least 2 anchors with known positions for a useful TDOA pair */
    int valid_with_pos = 0;
    for (int i = 0; i < N_ANCHORS; i++) {
        if (obs[i].valid && anchor_pos_known[i]) valid_with_pos++;
    }
    if (valid_with_pos < 2) return;

    /* One line per frame; columns per anchor separated by tabs:
     *   TDOA <id> <rx_time> <anchor_tx_time> <x> <y> <z>  [repeated] */
    for (int i = 0; i < N_ANCHORS; i++) {
        if (!obs[i].valid || !anchor_pos_known[i]) {
            printf("B TDOA\t%d invalid\r\n", i);
            continue;
        }
        printf("\t%d %llu %llu %llu %.4f %.4f %.4f",
               i,
               (unsigned long long)obs[i].rx_time,
               (unsigned long long)obs[i].anchor_tx_time,
               (unsigned long long)obs[i].anchor_rx_ref,
               anchor_pos[i][0], anchor_pos[i][1], anchor_pos[i][2]);
    }
    printf("\r\n");
}

#else  /* TDOA_TAG_OFFLOAD == 0 — on-device Chan-Ho 2D solver */
/* DW3000 physical constants */
/**
 * @brief DW1000 UWB chip clock frequency in Hz
 *
 * This value is derived from the DW1000's internal clock specifications:
 * - Base frequency: 499.2 MHz (the fundamental crystal oscillator frequency)
 * - Multiplier: 128 (the internal PLL multiplier used for timestamp counter)
 *
 * This gives a timestamp tick rate of approximately 63.8976 GHz, which is used
 * for high-precision time-of-flight measurements in UWB ranging and TDoA positioning.
 * The timestamp counter increments at this rate, providing ~15.65 picosecond resolution.
 */
// static const double CLOCK_FREQ_HZ  = 499.2e6 * 128.0; /* ticks per second */
// static const double SPEED_OF_LIGHT = 299792458.0;       /* metres per second */

#define CLOCK_FREQ_HZ 499.2e6 * 128.0 /* ticks per second */
#define SPEED_OF_LIGHT 299792458.0 /* metres per second */
#define DO_CALIBRATE 1

#define PRINT_LOGS 1
static void emit_measurements(void)
{
    /* 1. Pre-flight checks */
    for (int i = 0; i < 3; i++) {
        if (!obs[i].valid || !anchor_pos_known[i]) return;
    }

    /* 2. Absolute Anchor Coordinates */
    double x0 = anchor_pos[0][0], y0 = anchor_pos[0][1];
    double x1 = anchor_pos[1][0], y1 = anchor_pos[1][1];
    double x2 = anchor_pos[2][0], y2 = anchor_pos[2][1];

    /* 3. TDOA Math (Standardized to 40-bit masking) */
    uint64_t ta1 = (obs[1].anchor_tx_time - obs[1].anchor_rx_ref) & MASK_40BIT;
    uint64_t ta2 = (obs[2].anchor_tx_time - obs[2].anchor_rx_ref) & MASK_40BIT;

    int64_t drx10 = (int64_t)((obs[1].rx_time - obs[0].rx_time) & MASK_40BIT);
    int64_t drx20 = (int64_t)((obs[2].rx_time - obs[0].rx_time) & MASK_40BIT);

    /* Inter-anchor distances for ToF compensation */
    double d01_m = sqrt(pow(x1-x0, 2) + pow(y1-y0, 2));
    double d02_m = sqrt(pow(x2-x0, 2) + pow(y2-y0, 2));
    int64_t tof01 = (int64_t)(d01_m / SPEED_OF_LIGHT * CLOCK_FREQ_HZ); //64448000000.0);
    int64_t tof02 = (int64_t)(d02_m / SPEED_OF_LIGHT * CLOCK_FREQ_HZ); //64448000000.0);

    // Manually subtract the ~71m bias (approx 15264 ticks)
    int64_t bias = 0;
    // int64_t bias = 15264;
    /* Ticks calculation: (Measured Delay) - (Internal Anchor Delay) - (Air Time A0->Ai) */
    int64_t t10 = drx10 - (int64_t)ta1 - tof01 - bias;
    int64_t t20 = drx20 - (int64_t)ta2 - tof02 - bias;

    /* Conversion to meters: R_i - R_0 */
    double d10 = (double)t10 * 0.00469176;
    double d20 = (double)t20 * 0.00469176;

    #define BIAS10 75.1975
    #define BIAS20 76.2128

    #if DO_CALIBRATE == 0
    printf("+");
    /* --- CALIBRATION MODE --- */
    // 1. Define your EXACT physical location during calibration
    double true_x = 2.5, true_y = 0.7;

    // 2. Calculate the "Geometric" TDoA (what it SHOULD be)
    double d0_true = sqrt(pow(true_x - x0, 2) + pow(true_y - y0, 2));
    double d1_true = sqrt(pow(true_x - x1, 2) + pow(true_y - y1, 2));
    double d2_true = sqrt(pow(true_x - x2, 2) + pow(true_y - y2, 2));

    double expected_d10 = d1_true - d0_true;
    double expected_d20 = d2_true - d0_true;

    // 3. Compare to your raw "dirty" d10 and d20 (the ones that currently say ~71m)
    static double bias10 = 0, bias20 = 0;
    static int cal_samples = 0;

    if (cal_samples < 100) { // Collect 100 samples to average out noise
        bias10 += (d10 - expected_d10);
        bias20 += (d20 - expected_d20);
        cal_samples++;
        if (cal_samples == 100) {
            bias10 /= 100.0;
            bias20 /= 100.0;
            printf("\nCALIBRATION COMPLETE!\n");
            printf("Set BIAS10 to: %.4f\n", bias10);
            printf("Set BIAS20 to: %.4f\n", bias20);
        }
        return; // Don't solve yet
    }
    printf("Using bias10: %.4f, bias20: %.4f\n", bias10, bias20);

    // 4. Apply the calibrated bias to all future measurements
    // d10 -= bias10;
    // d20 -= bias20;
    /* --- END CALIBRATION --- */
    #endif

    #ifdef BIAS10
    d10 -= BIAS10;
    #endif
    #ifdef BIAS20
    d20 -= BIAS20;
    #endif

    /* 4. Sanity Check: Range difference cannot exceed physical distance between anchors */
    if (fabs(d10) > d01_m || fabs(d20) > d02_m) {
        if (PRINT_LOGS) printf("POS\tWarning: TDoA outside physical bounds (d10:%.2f/%.2f)\n", d10, d01_m);
    }

    /* 5. Linearization using Local Coordinates (Shift A0 to 0,0) */
    double x1_loc = x1 - x0; double y1_loc = y1 - y0;
    double x2_loc = x2 - x0; double y2_loc = y2 - y0;

    double K1 = x1_loc*x1_loc + y1_loc*y1_loc;
    double K2 = x2_loc*x2_loc + y2_loc*y2_loc;

    /* Solve: A * [x; y] = b - R0 * c */
    double a11 = 2.0 * x1_loc; double a12 = 2.0 * y1_loc;
    double a21 = 2.0 * x2_loc; double a22 = 2.0 * y2_loc;
    double det = a11 * a22 - a12 * a21;

    if (fabs(det) < 1e-6) return;

    double b1 = K1 - d10 * d10;
    double b2 = K2 - d20 * d20;
    double c1 = 2.0 * d10;
    double c2 = 2.0 * d20;

    double inv = 1.0 / det;
    double Px = inv * ( a22 * b1 - a12 * b2);
    double Py = inv * (-a21 * b1 + a11 * b2);
    double Qx = inv * ( a22 * c1 - a12 * c2);
    double Qy = inv * (-a21 * c1 + a11 * c2);

    /* Quadratic: Aq*R0^2 + Bq*R0 + Cq = 0 (Relative to A0 at 0,0) */
    double Aq = 1.0 - Qx*Qx - Qy*Qy;
    double Bq = 2.0 * (Px*Qx + Py*Qy);
    double Cq = -(Px*Px + Py*Py);

    double disc = Bq*Bq - 4.0*Aq*Cq;
    // allow for a little bit of noise
    // if (disc < -0.1) { printf("POS\tNo physical intersection\n"); return; }
    // if (disc < 0) disc = 0;

    // If noise makes the intersection impossible, "disc" goes negative.
    // We treat it as 0 to find the point where the hyperbolas almost touch.
    if (disc < 0) {
        printf("POS\tLow confidence (No hard intersection, disc: %.3f)\n", disc);
        disc = 0;
    }

    double R0a = (-Bq + sqrt(disc)) / (2.0 * Aq);
    double R0b = (-Bq - sqrt(disc)) / (2.0 * Aq);

    // For R0 (distance to A0), we only care about the positive root.
    // In a room, R0 should usually be between 0 and 20 meters.
    double R0 = -1.0;
    if (R0a > 0) R0 = R0a;
    if (R0b > 0 && (R0 < 0 || R0b < R0)) R0 = R0b;

    if (R0 < 0) {
        printf("POS\tError: All solutions result in negative distances.\n");
        return;
    }

    double x_tag = (Px - R0 * Qx) + x0;
    double y_tag = (Py - R0 * Qy) + y0;

    printf("POS\t(%.3f, %.3f) | d10: %.2f d20: %.2f R0: %.2f\n", x_tag, y_tag, d10, d20, R0);
}

#endif /* TDOA_TAG_OFFLOAD */
/* --------------------------------------------------------------------------
 * UWB event handler
 * -------------------------------------------------------------------------- */
static uint32_t tdoa2TagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
    static packet_t rxPacket;

    if (event == eventPacketReceived) {
        dwTime_t rx_time = { .full = 0 };
        dwGetRawReceiveTimestamp(dev, &rx_time);
        dwCorrectTimestamp(dev, &rx_time);

        int data_len    = dwGetDataLength(dev);
        int payload_len = data_len - MAC802154_HEADER_LENGTH;
        dwGetData(dev, (uint8_t *)&rxPacket, data_len);

        int anchor_id = rxPacket.sourceAddress[0];

        if (payload_len >= (int)sizeof(rangePacket_t) &&
            rxPacket.payload[0] == PACKET_TYPE_TDOA2 &&
            anchor_id < NSLOTS)
        {
            rangePacket_t *rp = (rangePacket_t *)rxPacket.payload;

            /* Anchor 0 starts a new frame: flush previous frame first */
            if (anchor_id == 0) {
                emit_measurements();
                clear_frame();
            }
            // printf("R\tReceived packet from anchor %d at local time %llu\r\n",
            //        anchor_id, (unsigned long long)rx_time.full);

            /* Update persistent anchor position from LPP if present */
            try_extract_lpp_pos(rxPacket.payload, payload_len, anchor_id);

            /* Record observation for this slot */
            anchor_obs_t *o = &obs[anchor_id];
            o->valid          = true;
            o->rx_time        = rx_time.full & MASK_40BIT;

            /* Anchor's own TX timestamp (from timestamps[anchor_id]) */
            o->anchor_tx_time = 0;
            memcpy(&o->anchor_tx_time, rp->timestamps[anchor_id], TS_TX_SIZE);
            o->anchor_tx_time &= MASK_40BIT;

            /* Anchor's RX of anchor 0 (from timestamps[0]).
             * For anchor 0 itself this IS the TX time (same value).
             * For others it is when this anchor received anchor 0. */
            o->anchor_rx_ref = 0;
            memcpy(&o->anchor_rx_ref, rp->timestamps[0], TS_TX_SIZE);
            o->anchor_rx_ref &= MASK_40BIT;

            memcpy(o->pos, anchor_pos[anchor_id], sizeof(o->pos));
            obs_count++;

            /* Blink when we have a complete set */
            // if (obs_count == N_ANCHORS) {
            //     ledBlink(ledRanging, true);
            // }
        }
    }

    /* Restart receive immediately — tag never transmits */
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);

    return MAX_TIMEOUT;
}

/* --------------------------------------------------------------------------
 * Init
 * -------------------------------------------------------------------------- */
static void tdoa2TagInit(uwbConfig_t *config, dwDevice_t *dev)
{
    (void)config;

    clear_frame();
    memset(anchor_pos,       0, sizeof(anchor_pos));
    memset(anchor_pos_known, 0, sizeof(anchor_pos_known));

    #if PIN_LED_WS2812
    ledBlink(ledMode, false);  /* continuous red = tag mode running */
    #endif

    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
}

/* --------------------------------------------------------------------------
 * Algorithm descriptor — registered in uwb.c
 * -------------------------------------------------------------------------- */
uwbAlgorithm_t uwbTdoa2TagAlgorithm = {
    .init    = tdoa2TagInit,
    .onEvent = tdoa2TagOnEvent,
};
