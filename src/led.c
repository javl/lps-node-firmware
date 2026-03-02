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
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"

/* LED GPIO pins — defined by build flags PIN_LED_RANGING/SYNC/MODE */
static const int led_pins[N_LEDS] = {
    [ledRanging] = PIN_LED_RANGING,
    [ledSync]    = PIN_LED_SYNC,
    [ledMode]    = PIN_LED_MODE,
};

static bool isBlinking[N_LEDS];
static uint32_t disableTime[N_LEDS];

void ledInit(void)
{
    uint64_t mask = (1ULL << PIN_LED_RANGING) |
                    (1ULL << PIN_LED_SYNC)    |
                    (1ULL << PIN_LED_MODE);
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    for (int i = 0; i < N_LEDS; i++) {
        gpio_set_level(led_pins[i], 0);
    }
}

static inline void setLed(led_e led, bool value)
{
    gpio_set_level(led_pins[led], value ? 1 : 0);
}

void ledOn(led_e led) {
  isBlinking[led] = false;
  setLed(led, true);
}

void ledOff(led_e led) {
  isBlinking[led] = false;
  setLed(led, false);
}

void ledBlink(led_e led, bool oneshot)
{
    isBlinking[led] = true;
    if (oneshot) {
        disableTime[led] = xTaskGetTickCount() * portTICK_PERIOD_MS + 50;
        setLed(led, true);
    }
}

void ledTick(void)
{
    static uint32_t lastTick;
    static bool     blinkStatus;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (int led = 0; led < N_LEDS; led++) {
        if (isBlinking[led] && disableTime[led] && disableTime[led] < now) {
            setLed(led, false);
            disableTime[led] = 0;
            isBlinking[led]  = false;
        }
    }

    if (now > (lastTick + 250)) {
        blinkStatus = !blinkStatus;
        lastTick    = now;
        for (int led = 0; led < N_LEDS; led++) {
            if (isBlinking[led] && !disableTime[led]) {
                setLed(led, blinkStatus);
            }
        }
    }
}
