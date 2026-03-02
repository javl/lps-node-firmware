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
 * LPS node firmware — ESP32-S3 port
 *
 * Configuration is provided at compile time via build flags in platformio.ini.
 * There is no interactive serial menu.
 */
#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "button.h"
#include "cfg.h"
#include "uwb.h"


/* ------------------------------------------------------------------ */

static void handleButton(void)
{
    ButtonEvent be = buttonGetState();

    if (be == buttonShortPress) {
        ledBlink(ledRanging, true);
    } else if (be == buttonLongPress) {
        ledBlink(ledSync, true);
    }

    buttonProcess();
}

static void printConfig(void)
{
    struct uwbConfig_s *c = uwbGetConfig();

    printf("CONFIG\t: Address is 0x%X\r\n", c->address[0]);
    printf("CONFIG\t: Mode is %s\r\n", uwbAlgorithmName(c->mode));
    printf("CONFIG\t: Tag mode anchor list (%i): ", c->anchorListSize);
    for (int i = 0; i < c->anchorListSize; i++) {
        printf("0x%02X ", c->anchors[i]);
    }
    printf("\r\n");
    printf("CONFIG\t: Anchor position enabled: %s\r\n",
           c->positionEnabled ? "true" : "false");
    if (c->positionEnabled) {
        printf("CONFIG\t: Anchor position: %f %f %f\r\n",
               c->position[0], c->position[1], c->position[2]);
    }
    printf("CONFIG\t: SmartPower: %s  ForceTxPower: %s\r\n",
           c->smartPower ? "True" : "False",
           c->forceTxPower ? "True" : "False");
    if (c->forceTxPower) {
        printf("CONFIG\t: TX power setting: %08X\r\n", (unsigned int)c->txPower);
    }
    printf("CONFIG\t: Bitrate: %s  Preamble: %s\r\n",
           c->lowBitrate ? "low" : "normal",
           c->longPreamble ? "long" : "normal");
}

/* ------------------------------------------------------------------ */

void app_main(void)
{
    ledInit();
    buttonInit(buttonIdle);

    /* Flash all LEDs during init */
    ledOn(ledRanging);
    ledOn(ledSync);
    ledOn(ledMode);

    printf("\r\n\r\n====================\r\n");
    printf("SYSTEM\t: LPS node firmware (ESP32-S3)\r\n");

    printf("TEST\t: Initialize UWB ... ");
    cfgInit();
    uwbInit();

    if (uwbTest()) {
        printf("[OK]\r\n");
    } else {
        printf("[ERROR]: %s\r\n", uwbStrError());
        printf("TEST\t: UWB self-test failed, halting!\r\n");
        while (true) {
            ledBlink(ledRanging, false);
            ledBlink(ledSync,    false);
            ledBlink(ledMode,    false);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

    printConfig();

    vTaskDelay(pdMS_TO_TICKS(500));

    ledOff(ledRanging);
    ledOff(ledSync);
    ledOff(ledMode);

    printf("SYSTEM\t: Node started ...\r\n");

    uwbStart();

    /* Main loop: LED tick + button */
    while (1) {
        ledTick();
        handleButton();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
















