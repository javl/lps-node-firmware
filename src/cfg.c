/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * cfg.c — Compile-time node configuration (ESP32-S3 port).
 *
 * All settings come from build flags defined in platformio.ini.
 * cfgWrite* calls update an in-memory store only (no persistence) so
 * the rest of the code (e.g. LPP anchor-position updates via radio) works
 * correctly for the lifetime of the current session.
 *
 * Required build flags (set per-environment in platformio.ini):
 *   NODE_ADDRESS       0-255
 *   NODE_MODE          MODE_ANCHOR / MODE_TAG / …
 *   NODE_ANCHOR_X      X coordinate of the anchor
 *   NODE_ANCHOR_Y      Y coordinate of the anchor
 *   NODE_ANCHOR_Z      Z coordinate of the anchor
 *   UWB_SMART_POWER    0 or 1
 *   UWB_FORCE_TX_POWER 0 or 1
 *   UWB_TX_POWER       32-bit hex
 *   UWB_LOW_BITRATE    0 or 1
 *   UWB_LONG_PREAMBLE  0 or 1
 *   ANCHOR_LIST_SIZE   number of anchors in the default list
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "cfg.h"
#include "uwb.h"

/* Default anchor list — edit here or change ANCHOR_LIST_SIZE in platformio.ini */
#ifndef ANCHOR_LIST_SIZE
#  define ANCHOR_LIST_SIZE 6
#endif
static const uint8_t default_anchor_list[ANCHOR_LIST_SIZE] = {0, 1, 2};

/* ---- tiny in-memory key-value store ------------------------------------- */
#define MAX_CFG_ENTRIES 16

typedef struct {
    ConfigField field;
    uint32_t    u32;
    uint8_t     raw[sizeof(float) * 3]; /* fits 3-element float position */
    uint8_t     len;                    /* 0 = scalar, >0 = byte array    */
    bool        valid;
} CfgEntry;

static CfgEntry store[MAX_CFG_ENTRIES];
static int      nStore = 0;

static CfgEntry *findEntry(ConfigField field)
{
    for (int i = 0; i < nStore; i++) {
        if (store[i].valid && store[i].field == field) return &store[i];
    }
    return NULL;
}

static CfgEntry *getOrCreate(ConfigField field)
{
    CfgEntry *e = findEntry(field);
    if (e) return e;
    if (nStore < MAX_CFG_ENTRIES) {
        e = &store[nStore++];
        e->field = field;
        e->valid = true;
        e->len   = 0;
        return e;
    }
    return NULL; /* store full */
}

/* ---- public API --------------------------------------------------------- */
// Add fallback to prevent issues building
#ifndef NODE_ANCHOR_X
#  define NODE_ANCHOR_X 0
#endif
#ifndef NODE_ANCHOR_Y
#  define NODE_ANCHOR_Y 0
#endif
#ifndef NODE_ANCHOR_Z
#  define NODE_ANCHOR_Z 0
#endif

void cfgInit(void)
{
    cfgWriteU8(cfgAddress,      (uint8_t)NODE_ADDRESS);
    cfgWriteU8(cfgMode,         (uint8_t)NODE_MODE);
    cfgWriteFP32list(cfgAnchorPos, (float[]){NODE_ANCHOR_X, NODE_ANCHOR_Y, NODE_ANCHOR_Z}, 3);
    cfgWriteU8(cfgSmartPower,   (uint8_t)UWB_SMART_POWER);
    cfgWriteU8(cfgForceTxPower, (uint8_t)UWB_FORCE_TX_POWER);
    cfgWriteU32(cfgTxPower,     (uint32_t)UWB_TX_POWER);
    cfgWriteU8(cfgLowBitrate,   (uint8_t)UWB_LOW_BITRATE);
    cfgWriteU8(cfgLongPreamble, (uint8_t)UWB_LONG_PREAMBLE);
    cfgWriteU8list(cfgAnchorlist,
                   (uint8_t *)default_anchor_list,
                   ANCHOR_LIST_SIZE);

    printf("CONFIG\t: Loaded from build flags — "
           "addr=0x%02X mode=%d x=%f y=%f z=%f smart=%d lowBR=%d longPre=%d\r\n",
           NODE_ADDRESS, NODE_MODE, NODE_ANCHOR_X, NODE_ANCHOR_Y, NODE_ANCHOR_Z,
           UWB_SMART_POWER, UWB_LOW_BITRATE, UWB_LONG_PREAMBLE);
}

bool cfgReset(void)
{
    printf("CONFIG\t: cfgReset() is a no-op (compile-time configuration)\r\n");
    return false;
}

bool cfgFieldSize(ConfigField field, uint8_t *size)
{
    CfgEntry *e = findEntry(field);
    if (!e) return false;
    *size = (e->len > 0) ? e->len : 1;
    return true;
}

bool cfgReadU8(ConfigField field, uint8_t *value)
{
    CfgEntry *e = findEntry(field);
    if (!e) return false;
    *value = (uint8_t)(e->u32 & 0xFF);
    return true;
}

bool cfgWriteU8(ConfigField field, uint8_t value)
{
    CfgEntry *e = getOrCreate(field);
    if (!e) return false;
    e->u32 = value;
    e->len = 0;
    return true;
}

bool cfgReadU32(ConfigField field, uint32_t *value)
{
    CfgEntry *e = findEntry(field);
    if (!e) return false;
    *value = e->u32;
    return true;
}

bool cfgWriteU32(ConfigField field, uint32_t value)
{
    CfgEntry *e = getOrCreate(field);
    if (!e) return false;
    e->u32 = value;
    e->len = 0;
    return true;
}

bool cfgReadU8list(ConfigField field, uint8_t list[], uint8_t length)
{
    CfgEntry *e = findEntry(field);
    if (!e || e->len == 0) return false;
    uint8_t n = (length < e->len) ? length : e->len;
    memcpy(list, e->raw, n);
    return true;
}

bool cfgWriteU8list(ConfigField field, uint8_t list[], uint8_t length)
{
    if (length > sizeof(store[0].raw)) return false;
    CfgEntry *e = getOrCreate(field);
    if (!e) return false;
    memcpy(e->raw, list, length);
    e->len = length;
    return true;
}

bool cfgReadFP32listLength(ConfigField field, uint8_t *size)
{
    CfgEntry *e = findEntry(field);
    if (!e || e->len == 0) return false;
    *size = e->len / sizeof(float);
    return true;
}

bool cfgReadFP32list(ConfigField field, float list[], uint8_t length)
{
    CfgEntry *e = findEntry(field);
    if (!e || e->len == 0) return false;
    uint8_t n = e->len / sizeof(float);
    if (length < n) n = length;
    memcpy(list, e->raw, n * sizeof(float));
    return true;
}

bool cfgWriteFP32list(ConfigField field, float list[], uint8_t length)
{
    size_t bytes = length * sizeof(float);
    if (bytes > sizeof(store[0].raw)) return false;
    CfgEntry *e = getOrCreate(field);
    if (!e) return false;
    memcpy(e->raw, list, bytes);
    e->len = (uint8_t)bytes;
    return true;
}

static bool binaryMode = false;

void cfgSetBinaryMode(bool enable) { binaryMode = enable; }
bool cfgIsBinaryMode(void)         { return binaryMode; }











