/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware -- WS2812B LED driver (ESP32-S3)
 *
 * Drives the single onboard WS2812B RGB LED (LOLIN S3 Mini, GPIO47) via the
 * ESP-IDF RMT peripheral.  The three logical LEDs are mapped to RGB channels:
 *
 *   ledMode    -> Red
 *   ledRanging -> Green
 *   ledSync    -> Blue
 *
 * All three states are composited into one pixel on every state change.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"

/* -------------------------------------------------------------------------- */

/* Per-channel brightness when "on" (0-255).  80 is comfortably visible
 * indoors without being distracting.                                        */
#define LED_BRIGHT   10U

/* RMT clock: 10 MHz -> 100 ns per tick.
 * WS2812B timing (Worldsemi datasheet):
 *   bit-0 :  T0H = 400 ns (4 ticks),  T0L = 850 ns (8 ticks)
 *   bit-1 :  T1H = 800 ns (8 ticks),  T1L = 450 ns (4 ticks)
 *   Reset :  > 50 us low -- satisfied by the idle-low state between frames  */
#define RMT_RES_HZ   10000000U

/* -------------------------------------------------------------------------- */

static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_enc  = NULL;

static bool     s_on[N_LEDS];
static bool     s_blinking[N_LEDS];
static uint32_t s_disable_at[N_LEDS];

/* -------------------------------------------------------------------------- */

static void ws2812_update(void)
{
    /* Compose GRB pixel (WS2812B byte order: G, R, B) */
    uint8_t grb[3] = {
        s_on[ledMode]    ? LED_BRIGHT : 0,   /* G */
        s_on[ledRanging] ? LED_BRIGHT : 0,   /* R */
        s_on[ledSync]    ? LED_BRIGHT : 0,   /* B */
    };

    rmt_transmit_config_t tx_cfg = {
        .loop_count       = 0,
        .flags.eot_level  = 0,  /* keep pin LOW after tx -> forms reset pulse */
    };
    rmt_transmit(s_chan, s_enc, grb, sizeof(grb), &tx_cfg);
    rmt_tx_wait_all_done(s_chan, 10 /* ms timeout */);
}

/* -------------------------------------------------------------------------- */

void ledInit(void)
{
    #if PIN_LED_WS2812
    /* 1. Create RMT TX channel on the WS2812B data pin. */
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num          = PIN_LED_WS2812,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags.invert_out  = false,
        .flags.with_dma    = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_chan));

    /* 2. Create bytes encoder with WS2812B bit timings. */
    rmt_bytes_encoder_config_t enc_cfg = {
        /* bit-0: 400 ns HIGH, then 800 ns LOW */
        .bit0 = { .duration0 = 4, .level0 = 1,
                  .duration1 = 8, .level1 = 0 },
        /* bit-1: 800 ns HIGH, then 400 ns LOW */
        .bit1 = { .duration0 = 8, .level0 = 1,
                  .duration1 = 4, .level1 = 0 },
        .flags.msb_first = 1,           /* WS2812B shifts out MSB first */
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_enc));

    /* 3. Enable the channel (must be done before first transmit). */
    ESP_ERROR_CHECK(rmt_enable(s_chan));

    /* 4. Start with all LEDs off. */
    memset(s_on,         0, sizeof(s_on));
    memset(s_blinking,   0, sizeof(s_blinking));
    memset(s_disable_at, 0, sizeof(s_disable_at));
    ws2812_update();
    #endif
}

/* -------------------------------------------------------------------------- */

static inline void setLed(led_e led, bool value)
{
    s_on[led] = value;
    ws2812_update();
}

void ledOn(led_e led) {
    s_blinking[led] = false;
    setLed(led, true);
}

void ledOff(led_e led) {
    s_blinking[led] = false;
    setLed(led, false);
}

void ledBlink(led_e led, bool oneshot)
{
    s_blinking[led] = true;
    if (oneshot) {
        s_disable_at[led] = xTaskGetTickCount() * portTICK_PERIOD_MS + 50;
        setLed(led, true);
    }
}

void ledTick(void)
{
    static uint32_t last_tick;
    static bool     blink_state;
    bool            changed = false;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /* One-shot blink expiry */
    for (int i = 0; i < N_LEDS; i++) {
        if (s_blinking[i] && s_disable_at[i] && s_disable_at[i] < now) {
            s_on[i]         = false;
            s_disable_at[i] = 0;
            s_blinking[i]   = false;
            changed         = true;
        }
    }

    /* Continuous blink at 250 ms half-period */
    if (now > last_tick + 250) {
        blink_state = !blink_state;
        last_tick   = now;
        for (int i = 0; i < N_LEDS; i++) {
            if (s_blinking[i] && !s_disable_at[i]) {
                s_on[i] = blink_state;
                changed  = true;
            }
        }
    }

    if (changed) {
        ws2812_update();
    }
}
