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
 *   anchor_tx_time — anchor's own TX timestamp copied from the packet (low 32 bits)
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
#define TS_TX_SIZE         4   /* bytes per timestamp in rangePacket_t   */

typedef struct {
    uint8_t  type;
    uint8_t  pid[NSLOTS];
    uint8_t  timestamps[NSLOTS][TS_TX_SIZE];
    uint16_t distances[NSLOTS];
} __attribute__((packed)) rangePacket_t;

/* DW3000 physical constants */
#ifndef TDOA_TAG_OFFLOAD
static const double CLOCK_FREQ_HZ  = 499.2e6 * 128.0; /* ticks per second */
static const double SPEED_OF_LIGHT = 299792458.0;       /* metres per second */
#endif

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
    uint32_t anchor_tx_time;  /* anchor's TX timestamp, low 32 bits      */
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
    printf("TDOA");
    for (int i = 0; i < N_ANCHORS; i++) {
        if (!obs[i].valid || !anchor_pos_known[i]) continue;
        printf("\t%d %llu %lu %.4f %.4f %.4f",
               i,
               (unsigned long long)obs[i].rx_time,
               (unsigned long)obs[i].anchor_tx_time,
               anchor_pos[i][0], anchor_pos[i][1], anchor_pos[i][2]);
    }
    printf("\r\n");
}

#else  /* TDOA_TAG_OFFLOAD == 0 — on-device Chan-Ho 2D solver */

static void emit_measurements(void)
{
    /* Require all three anchors received with known positions */
    for (int i = 0; i < N_ANCHORS; i++) {
        if (!obs[i].valid || !anchor_pos_known[i]) return;
    }

    double x0 = anchor_pos[0][0], y0 = anchor_pos[0][1];
    double x1 = anchor_pos[1][0], y1 = anchor_pos[1][1];
    double x2 = anchor_pos[2][0], y2 = anchor_pos[2][1];

    /* TDOA in ticks (corrected for slot timing):
     *   d_i0 = R_i - R_0  (how much farther anchor i is vs anchor 0)
     *        = [(rx_i - rx_0) - (tx_i - tx_0)] * c / clock_freq
     * The (int32_t) cast handles 32-bit wraparound of anchor timestamps. */
    int64_t t10 = (int64_t)(obs[1].rx_time - obs[0].rx_time)
                - (int64_t)(int32_t)(obs[1].anchor_tx_time - obs[0].anchor_tx_time);
    int64_t t20 = (int64_t)(obs[2].rx_time - obs[0].rx_time)
                - (int64_t)(int32_t)(obs[2].anchor_tx_time - obs[0].anchor_tx_time);

    double d_10 = (double)t10 / CLOCK_FREQ_HZ * SPEED_OF_LIGHT; /* R1-R0 [m] */
    double d_20 = (double)t20 / CLOCK_FREQ_HZ * SPEED_OF_LIGHT; /* R2-R0 [m] */

    double K0 = x0*x0 + y0*y0;
    double K1 = x1*x1 + y1*y1;
    double K2 = x2*x2 + y2*y2;

    /* Chan-Ho linearization (Chan & Ho, IEEE T-SP 1994):
     *
     * From  R_i = R_0 + d_i0  and  R_i^2 = (x-xi)^2 + (y-yi)^2 :
     *
     *   2*(x1-x0)*x + 2*(y1-y0)*y + 2*d_10*R0 = K1 - K0 - d_10^2   (eq.1)
     *   2*(x2-x0)*x + 2*(y2-y0)*y + 2*d_20*R0 = K2 - K0 - d_20^2   (eq.2)
     *
     * Rewrite as  A*[x,y]^T = b - R0*c
     *   →  [x,y]^T = A^{-1}*b  -  R0 * A^{-1}*c   ≡   P - R0*Q        */
    double a11 = 2.0*(x1-x0), a12 = 2.0*(y1-y0);
    double a21 = 2.0*(x2-x0), a22 = 2.0*(y2-y0);
    double det = a11*a22 - a12*a21;
    if (fabs(det) < 1e-6) { printf("POS\tanchor collinear\r\n"); return; }

    double b1 = K1 - K0 - d_10*d_10;
    double b2 = K2 - K0 - d_20*d_20;
    double c1 = 2.0*d_10;
    double c2 = 2.0*d_20;

    double inv = 1.0 / det;
    double Px = inv * ( a22*b1 - a12*b2);
    double Py = inv * (-a21*b1 + a11*b2);
    double Qx = inv * ( a22*c1 - a12*c2);
    double Qy = inv * (-a21*c1 + a11*c2);

    /* Substitute  x = Px - R0*Qx,  y = Py - R0*Qy  into
     *   R0^2 = (x-x0)^2 + (y-y0)^2
     * → quadratic:  Aq*R0^2 + Bq*R0 + Cq = 0               */
    double ax = Px - x0, ay = Py - y0;
    double Aq = 1.0 - Qx*Qx - Qy*Qy;
    double Bq = 2.0*(ax*Qx + ay*Qy);
    double Cq = -(ax*ax + ay*ay);

    double x_tag, y_tag;

    if (fabs(Aq) < 1e-9) {
        /* Degenerate — linear in R0 */
        if (fabs(Bq) < 1e-9) { printf("POS\tdegenerate\r\n"); return; }
        double R0 = -Cq / Bq;
        if (R0 < 0) { printf("POS\tneg range\r\n"); return; }
        x_tag = Px - R0*Qx;
        y_tag = Py - R0*Qy;
    } else {
        double disc = Bq*Bq - 4.0*Aq*Cq;
        if (disc < 0) { printf("POS\tno solution\r\n"); return; }
        double sq  = sqrt(disc);
        double R0a = (-Bq + sq) / (2.0*Aq);
        double R0b = (-Bq - sq) / (2.0*Aq);

        /* Pick positive root; if both positive take the smaller (closer) one */
        double R0;
        if      (R0a >= 0 && R0b <  0) R0 = R0a;
        else if (R0b >= 0 && R0a <  0) R0 = R0b;
        else if (R0a >= 0 && R0b >= 0) R0 = (R0a < R0b) ? R0a : R0b;
        else { printf("POS\tboth roots neg\r\n"); return; }

        x_tag = Px - R0*Qx;
        y_tag = Py - R0*Qy;
    }

    printf("POS\tx=%.3f y=%.3f\r\n", x_tag, y_tag);
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

            /* Update persistent anchor position from LPP if present */
            try_extract_lpp_pos(rxPacket.payload, payload_len, anchor_id);

            /* Record observation for this slot */
            anchor_obs_t *o = &obs[anchor_id];
            o->valid          = true;
            o->rx_time        = rx_time.full;
            memcpy(&o->anchor_tx_time, rp->timestamps[anchor_id], TS_TX_SIZE);
            memcpy(o->pos, anchor_pos[anchor_id], sizeof(o->pos));
            obs_count++;

            /* Blink when we have a complete set */
            if (obs_count == N_ANCHORS) {
                ledBlink(ledRanging, true);
            }
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

    ledBlink(ledMode, false);  /* continuous red = tag mode running */

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
