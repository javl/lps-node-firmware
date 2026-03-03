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

// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "nvs_flash.h"


// #include "esp_log.h"

#if PIN_LED_WS2812
#include "led.h"
#endif

#ifdef PIN_BUTTON
#include "button.h"
#endif

#include "cfg.h"
#include "uwb.h"

// #define EXAMPLE_ESP_WIFI_SSID      "SSID"
// #define EXAMPLE_ESP_WIFI_PASS      "PASS"
// #define EXAMPLE_ESP_MAXIMUM_RETRY  2

// #if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
// #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
// #define EXAMPLE_H2E_IDENTIFIER ""
// #elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
// #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
// #define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
// #elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
// #define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
// #define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
// #endif
// #if CONFIG_ESP_WIFI_AUTH_OPEN
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
// #elif CONFIG_ESP_WIFI_AUTH_WEP
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
// #elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
// #endif

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

// /* FreeRTOS event group to signal when we are connected*/
// static EventGroupHandle_t s_wifi_event_group;

// /* The event group allows multiple bits for each event, but we only care about two events:
//  * - we are connected to the AP with an IP
//  * - we failed to connect after the maximum amount of retries */
// #define WIFI_CONNECTED_BIT BIT0
// #define WIFI_FAIL_BIT      BIT1

// static const char *TAG = "wifi station";

// static int s_retry_num = 0;


// static void event_handler(void* arg, esp_event_base_t event_base,
//                                 int32_t event_id, void* event_data)
// {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         esp_wifi_connect();
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
//             esp_wifi_connect();
//             s_retry_num++;
//             ESP_LOGI(TAG, "retry to connect to the AP");
//         } else {
//             xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
//         }
//         ESP_LOGI(TAG,"connect to the AP fail");
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
//         ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
//         s_retry_num = 0;
//         xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
//     }
// }

// void wifi_init_sta(void)
// {
//     s_wifi_event_group = xEventGroupCreate();

//     ESP_ERROR_CHECK(esp_netif_init());

//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_sta();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     esp_event_handler_instance_t instance_any_id;
//     esp_event_handler_instance_t instance_got_ip;
//     ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
//                                                         ESP_EVENT_ANY_ID,
//                                                         &event_handler,
//                                                         NULL,
//                                                         &instance_any_id));
//     ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
//                                                         IP_EVENT_STA_GOT_IP,
//                                                         &event_handler,
//                                                         NULL,
//                                                         &instance_got_ip));

//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = EXAMPLE_ESP_WIFI_SSID,
//             .password = EXAMPLE_ESP_WIFI_PASS,
//             /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
//              * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
//              * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
//              * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
//              */
//             // .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
//             // .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
//             // .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
// #ifdef CONFIG_ESP_WIFI_WPA3_COMPATIBLE_SUPPORT
//             .disable_wpa3_compatible_mode = 0,
// #endif
//         },
//     };
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
//     ESP_ERROR_CHECK(esp_wifi_start() );

//     ESP_LOGI(TAG, "wifi_init_sta finished.");

//     /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
//      * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
//     EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
//             WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
//             pdFALSE,
//             pdFALSE,
//             portMAX_DELAY);

//     /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
//      * happened. */
//     if (bits & WIFI_CONNECTED_BIT) {
//         ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
//                  EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
//     } else if (bits & WIFI_FAIL_BIT) {
//         ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
//                  EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
//     } else {
//         ESP_LOGE(TAG, "UNEXPECTED EVENT");
//     }
// }


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

    //     //Initialize NVS
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //   ESP_ERROR_CHECK(nvs_flash_erase());
    //   ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
    //     /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
    //      * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
    //     esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    // }

    // ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    // wifi_init_sta();


    #if PIN_LED_WS2812
    /* Flash all LEDs during init */
    ledOn(ledRanging);
    ledOn(ledSync);
    ledOn(ledMode);
    #endif

    /* DW3000 needs time to complete its power-on startup (crystal oscillator
     * + internal LDO settling) before IDLE_RC is reachable.  On a warm reset
     * (e.g. triggered by opening the serial monitor) the chip is already past
     * this phase, so init succeeds immediately.  Give it 200 ms on cold boot
     * so the LEDSYNC LED does not stay on indefinitely. */
    vTaskDelay(pdMS_TO_TICKS(200));

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
















