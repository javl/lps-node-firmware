/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "button.h"

static ButtonEvent state;

static bool buttonRead(void)
{
    /* BOOT button is active-low */
    return (gpio_get_level(PIN_BUTTON) == 0);
}

void buttonInit(ButtonEvent initialEvent)
{
    /* Configure button pin as input with pull-up (BOOT button is active-low) */
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    state = initialEvent;
}

void buttonProcess(void)
{
    static uint32_t lastTick;
    static uint32_t pressedTick;
    static bool     pressed;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (lastTick != now) {
        lastTick = now;

        if (!pressed && buttonRead() == BUTTON_PRESSED) {
            pressed     = true;
            pressedTick = now;
        } else if (pressed && buttonRead() == BUTTON_RELEASED) {
            pressed = false;
            if ((now - pressedTick) < BUTTON_LONGPRESS_TICK) {
                state = buttonShortPress;
            } else {
                state = buttonLongPress;
            }
        }
    }
}

ButtonEvent buttonGetState(void)
{
    ButtonEvent current = state;
    state = buttonIdle;
    return current;
}


