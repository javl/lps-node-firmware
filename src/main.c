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

#if PIN_LED_WS2812
#include "led.h"
#endif

#ifdef PIN_BUTTON
#include "button.h"
#endif

#include "cfg.h"
#include "uwb.h"


/* ------------------------------------------------------------------ */

#ifdef PIN_BUTTON
static void handleButton(void)
{
    ButtonEvent be = buttonGetState();

    if (be == buttonShortPress) {

        #ifdef PIN_LED_WS2812
        ledBlink(ledRanging, true);
        #endif
    } else if (be == buttonLongPress) {
        #ifdef PIN_LED_WS2812
        ledBlink(ledSync, true);
        #endif
    }

    buttonProcess();
}
#endif

// static void printConfig(void)
// {
//     struct uwbConfig_s *c = uwbGetConfig();

//     printf("CONFIG\t: Address is 0x%X\r\n", c->address[0]);
//     printf("CONFIG\t: Mode is %s\r\n", uwbAlgorithmName(c->mode));
//     printf("CONFIG\t: Tag mode anchor list (%i): ", c->anchorListSize);
//     for (int i = 0; i < c->anchorListSize; i++) {
//         printf("0x%02X ", c->anchors[i]);
//     }
//     printf("\r\n");
//     printf("CONFIG\t: Anchor position enabled: %s\r\n",
//            c->positionEnabled ? "true" : "false");
//     if (c->positionEnabled) {
//         printf("CONFIG\t: Anchor position: %f %f %f\r\n",
//                c->position[0], c->position[1], c->position[2]);
//     }
//     printf("CONFIG\t: SmartPower: %s  ForceTxPower: %s\r\n",
//            c->smartPower ? "True" : "False",
//            c->forceTxPower ? "True" : "False");
//     if (c->forceTxPower) {
//         printf("CONFIG\t: TX power setting: %08X\r\n", (unsigned int)c->txPower);
//     }
//     printf("CONFIG\t: Bitrate: %s  Preamble: %s\r\n",
//            c->lowBitrate ? "low" : "normal",
//            c->longPreamble ? "long" : "normal");
// }

/* ------------------------------------------------------------------ */

void app_main(void)
{
    #if PIN_LED_WS2812
    ledInit();
    #endif
    #ifdef PIN_BUTTON
    buttonInit(buttonIdle);
    #endif

    #if PIN_LED_WS2812
    /* Flash all LEDs during init */
    ledOn(ledRanging);
    ledOn(ledSync);
    ledOn(ledMode);
    #endif

    cfgInit();
    uwbInit();
    bool uwbOk = uwbTest();

    if (!uwbOk) {
        /* UWB failed — keep blinking, but still fall through to the main loop
         * so we keep printing the error status periodically. */
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    #if PIN_LED_WS2812
    ledOff(ledRanging);
    ledOff(ledSync);
    ledOff(ledMode);
    #endif

    if (uwbOk) {
        uwbStart();
    }

    /* Main loop: LED tick + button + periodic status print.
     *
     * USB-Serial/JTAG CDC has no "port opened" signal, so boot messages
     * printed once at startup are always missed if the monitor isn't already
     * connected.  Instead, we reprint the full status every STATUS_PERIOD_MS
     * so it appears shortly after the monitor is opened at any time. */
#define STATUS_PERIOD_MS 5000
    TickType_t lastStatus = xTaskGetTickCount() - pdMS_TO_TICKS(STATUS_PERIOD_MS);

    while (1) {
        #ifdef PIN_LED_WS2812
        ledTick();
        #endif

        #ifdef PIN_BUTTON
        handleButton();
        #endif

        if ((xTaskGetTickCount() - lastStatus) >= pdMS_TO_TICKS(STATUS_PERIOD_MS)) {
            lastStatus = xTaskGetTickCount();
            // printf("ping\r\n");
        //     printf("\r\n====================\r\n");
        //     printf("SYSTEM\t: LPS node firmware\r\n");
        //     if (uwbOk) {
        //         printConfig();
        //         printf("SYSTEM\t: Node running\r\n");
        //     } else {
        //         printf("TEST\t: UWB self-test FAILED: %s\r\n", uwbStrError());
        //     }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
















