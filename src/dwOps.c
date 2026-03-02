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
#include "dwOps.h"

#include <string.h>
#include <assert.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Hardware pins come from build flags defined in platformio.ini:
 *   PIN_DW_MOSI, PIN_DW_MISO, PIN_DW_SCK  — SPI2 (FSPI)
 *   PIN_DW_CS   — chip-select (manual GPIO, not hardware-managed)
 *   PIN_DW_RST  — active-low reset
 * The IRQ pin (PIN_DW_IRQ) is configured in uwb.c alongside the semaphore.
 */

#define DW_SPI_HOST    SPI2_HOST
#define DW_SPI_SLOW_HZ 2000000    /* 2 MHz  — used during OTP reads  */
#define DW_SPI_FAST_HZ 20000000   /* 20 MHz — normal operation        */
#define MAX_SPI_PACKET 128

/* Two device handles on the same bus at different speeds.
 * CS is driven manually so both handles can share the pin without
 * conflicting with the ESP-IDF CS management logic.              */
static spi_device_handle_t spi_slow;
static spi_device_handle_t spi_fast;
static spi_device_handle_t spi_current;

/* Scratch buffers — always accessed from the single UWB task. */
static uint8_t tx_buf[MAX_SPI_PACKET];
static uint8_t rx_buf[MAX_SPI_PACKET];

/* ------------------------------------------------------------------ */

static void spiWrite(dwDevice_t *dev,
                     const void *header, size_t headerLength,
                     const void *data,   size_t dataLength)
{
    size_t total = headerLength + dataLength;
    assert(total <= MAX_SPI_PACKET);

    memcpy(tx_buf, header, headerLength);
    memcpy(tx_buf + headerLength, data, dataLength);

    gpio_set_level(PIN_DW_CS, 0);
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(spi_current, &t);
    gpio_set_level(PIN_DW_CS, 1);
}

static void spiRead(dwDevice_t *dev,
                    const void *header, size_t headerLength,
                    void       *data,   size_t dataLength)
{
    size_t total = headerLength + dataLength;
    assert(total <= MAX_SPI_PACKET);

    memset(tx_buf, 0, total);
    memcpy(tx_buf, header, headerLength);

    gpio_set_level(PIN_DW_CS, 0);
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    spi_device_polling_transmit(spi_current, &t);
    gpio_set_level(PIN_DW_CS, 1);

    memcpy(data, rx_buf + headerLength, dataLength);
}

static void spiSetSpeed(dwDevice_t *dev, dwSpiSpeed_t speed)
{
    spi_current = (speed == dwSpiSpeedLow) ? spi_slow : spi_fast;
}

static void resetDw(dwDevice_t *dev)
{
    gpio_set_level(PIN_DW_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(PIN_DW_RST, 1);
}

static void delayms(dwDevice_t *dev, unsigned int delay)
{
    vTaskDelay(pdMS_TO_TICKS(delay));
}

/* ------------------------------------------------------------------ */

void dwOpsInit(dwDevice_t *device)
{
    /* CS and RST as plain GPIO outputs (CS starts deasserted) */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_DW_CS) | (1ULL << PIN_DW_RST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);
    gpio_set_level(PIN_DW_CS,  1);
    gpio_set_level(PIN_DW_RST, 1);

    /* Initialise SPI2 bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = PIN_DW_MOSI,
        .miso_io_num     = PIN_DW_MISO,
        .sclk_io_num     = PIN_DW_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = MAX_SPI_PACKET,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(DW_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* Slow handle — spics_io_num = -1 so we drive CS manually */
    spi_device_interface_config_t slow_cfg = {
        .clock_speed_hz = DW_SPI_SLOW_HZ,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(DW_SPI_HOST, &slow_cfg, &spi_slow));

    /* Fast handle */
    spi_device_interface_config_t fast_cfg = {
        .clock_speed_hz = DW_SPI_FAST_HZ,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(DW_SPI_HOST, &fast_cfg, &spi_fast));

    spi_current = spi_slow;  /* Start slow for DW1000 bring-up */
}




dwOps_t dwOps = {
    .spiRead     = spiRead,
    .spiWrite    = spiWrite,
    .spiSetSpeed = spiSetSpeed,
    .delayms     = delayms,
    .reset       = resetDw,
};
