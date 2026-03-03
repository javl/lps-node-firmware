/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware — ESP32-S3 / DW3000 port.
 *
 * src/dwOps.c — Hardware platform layer
 *
 * This file serves two purposes:
 *
 *  1. dwOpsInit(): initialise the ESP32-S3 SPI2 bus and all DW-related GPIOs
 *     (CS, RST, WAKEUP).  The IRQ GPIO is handled in uwb.c.
 *
 *  2. Platform functions called by the Qorvo DW3000 decadriver
 *     (deca_device.c):
 *       writetospi()         — SPI write (header + body)
 *       readfromspi()        — SPI write-then-read
 *       deca_sleep()         — millisecond delay
 *       wakeup_device_with_io() — pulse WAKEUP / CS to wake from deep sleep
 *       decamutexon/off()    — short critical-section guards
 *
 *  The DW3000 driver is vtable-free: it calls these as ordinary C functions
 *  rather than through the dwOps_t struct used by the old DW1000 driver.
 *  dwOps is retained as an empty sentinel so compilation units that reference
 *  it (e.g. dwOps.h extern) still link.
 *
 * Pin assignments come from build_flags in platformio.ini:
 *   PIN_DW_MOSI, PIN_DW_MISO, PIN_DW_SCK  — SPI2 (FSPI)
 *   PIN_DW_CS                              — chip-select (manual GPIO)
 *   PIN_DW_RST                             — reset (driven low then released)
 *   PIN_DW_WAKEUP                          — WAKEUP pad on DWM3000 module
 *   PIN_DW_IRQ                             — IRQ (configured in uwb.c)
 */

#include "dwOps.h"

#include <string.h>
#include <assert.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* DW3000 decadriver platform headers */
#include "deca_device_api.h"
#include "deca_spi.h"

/* ------------------------------------------------------------------ */

#define DW_SPI_HOST     SPI2_HOST
/* DW3000 requires <= 7 MHz during reset / initialise (dwt_initialise).
 * Normal operation supports up to 38 MHz; 20 MHz is a conservative safe
 * limit compatible with most PCB layouts.                              */
#define DW_SPI_SLOW_HZ  2000000    /*  2 MHz — reset + dwt_initialise  */
#define DW_SPI_FAST_HZ  20000000   /* 20 MHz — normal operation        */
/* Maximum single SPI transaction.  DW3000 register reads are at most a
 * 2-byte header + up to ~128 bytes payload.                           */
#define MAX_SPI_PACKET  256

/* Two device handles on the same bus at different speeds.
 * CS is driven manually (spics_io_num = -1) so both handles can coexist
 * on the bus without conflicting with the ESP-IDF CS management logic. */
static spi_device_handle_t spi_slow;
static spi_device_handle_t spi_fast;
static spi_device_handle_t spi_current;

/* Scratch DMA-capable buffers — only accessed from the single UWB task. */
static uint8_t tx_buf[MAX_SPI_PACKET];
static uint8_t rx_buf[MAX_SPI_PACKET];

/* Critical-section mutex used by decamutexon / decamutexoff (see below). */
static portMUX_TYPE spi_mux = portMUX_INITIALIZER_UNLOCKED;

/* ====================================================================
 * Speed-switching helpers (called from uwb.c around dwt_initialise)
 * ==================================================================== */

void dwSpiSetSpeedSlow(void) { spi_current = spi_slow; }
void dwSpiSetSpeedFast(void) { spi_current = spi_fast; }

/* ====================================================================
 * dwOpsInit — SPI bus + GPIO initialisation
 *
 * Must be called once before any dwt_* or shim function.
 * ==================================================================== */

void dwOpsInit(dwDevice_t *device)
{
    (void)device;   /* DW3000 driver is handle-free; parameter kept for
                     * source compatibility with the old DW1000 call site */

    /* --- CS and WAKEUP as outputs; RST starts as input (high-Z) ------- */
    gpio_config_t out_cfg = {
        #if PIN_DW_WAKEUP
        .pin_bit_mask = (1ULL << PIN_DW_CS) | (1ULL << PIN_DW_WAKEUP),
        #else
        .pin_bit_mask = (1ULL << PIN_DW_CS),
        #endif
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);
    gpio_set_level(PIN_DW_CS,     1);   /* deassert CS (active-low) */
    #if PIN_DW_WAKEUP
    gpio_set_level(PIN_DW_WAKEUP, 0);   /* WAKEUP idle-low           */
    #endif

    /* RST released (high-Z) so the DWM3000's internal pull-up controls it. */
    #ifdef PIN_DW_RST
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << PIN_DW_RST),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_cfg);
    #endif

    /* --- SPI2 bus -------------------------------------------------------- */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = PIN_DW_MOSI,
        .miso_io_num     = PIN_DW_MISO,
        .sclk_io_num     = PIN_DW_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = MAX_SPI_PACKET,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(DW_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* Slow device handle — used during reset and dwt_initialise */
    spi_device_interface_config_t slow_cfg = {
        .clock_speed_hz = DW_SPI_SLOW_HZ,
        .mode           = 0,            /* CPOL=0, CPHA=0 */
        .spics_io_num   = -1,           /* CS managed manually        */
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(DW_SPI_HOST, &slow_cfg, &spi_slow));

    /* Fast device handle — switched to after dwt_initialise succeeds */
    spi_device_interface_config_t fast_cfg = {
        .clock_speed_hz = DW_SPI_FAST_HZ,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(DW_SPI_HOST, &fast_cfg, &spi_fast));

    /* Start at slow speed; uwb.c calls dwSpiSetSpeedFast() after init. */
    spi_current = spi_slow;
}

/* ====================================================================
 * DW3000 decadriver platform functions
 *
 * These are called directly by deca_device.c; they must be defined as
 * ordinary (non-static) C functions with the exact signatures below.
 * ==================================================================== */

/* writetospi — write header then body to DW3000 over SPI. */
int writetospi(uint16_t        headerLength,
               const uint8_t  *headerBuffer,
               uint16_t        bodyLength,
               const uint8_t  *bodyBuffer)
{
    size_t total = (size_t)headerLength + (size_t)bodyLength;
    assert(total <= MAX_SPI_PACKET);

    memcpy(tx_buf,                  headerBuffer, headerLength);
    memcpy(tx_buf + headerLength,   bodyBuffer,   bodyLength);

    gpio_set_level(PIN_DW_CS, 0);
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(spi_current, &t);
    gpio_set_level(PIN_DW_CS, 1);

    return DWT_SUCCESS;
}

/* readfromspi — send header then clock in readlength bytes from DW3000. */
int readfromspi(uint16_t        headerLength,
                const uint8_t  *headerBuffer,
                uint16_t        readlength,
                uint8_t        *readBuffer)
{
    size_t total = (size_t)headerLength + (size_t)readlength;
    assert(total <= MAX_SPI_PACKET);

    memset(tx_buf, 0, total);
    memcpy(tx_buf, headerBuffer, headerLength);

    gpio_set_level(PIN_DW_CS, 0);
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    spi_device_polling_transmit(spi_current, &t);
    gpio_set_level(PIN_DW_CS, 1);

    memcpy(readBuffer, rx_buf + headerLength, readlength);

    return DWT_SUCCESS;
}

/* deca_sleep — millisecond blocking delay for use inside the decadriver. */
void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(pdMS_TO_TICKS(time_ms));
}

/* deca_usleep — microsecond busy-wait for short delays inside the decadriver.
 * Used by dwt_configure() and dwt_run_pgfcal() for sub-millisecond settling.
 * esp_rom_delay_us() is a busy-wait loop that works before the FreeRTOS
 * scheduler is running and with interrupts disabled.                       */
void deca_usleep(unsigned long time_us)
{
    esp_rom_delay_us((uint32_t)time_us);
}

/* writetospiwithcrc — SPI write + CRC byte (SPI CRC mode).
 * We do not enable SPI CRC in this firmware, but deca_device.c still
 * references this symbol unconditionally, so provide an implementation.
 * It behaves identically to writetospi but appends the crc8 byte.        */
int writetospiwithcrc(uint16_t        headerLength,
                      const uint8_t  *headerBuffer,
                      uint16_t        bodyLength,
                      const uint8_t  *bodyBuffer,
                      uint8_t         crc8)
{
    size_t total = (size_t)headerLength + (size_t)bodyLength + 1; /* +1 CRC */
    assert(total <= MAX_SPI_PACKET);

    memcpy(tx_buf,                  headerBuffer, headerLength);
    memcpy(tx_buf + headerLength,   bodyBuffer,   bodyLength);
    tx_buf[headerLength + bodyLength] = crc8;

    gpio_set_level(PIN_DW_CS, 0);
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };
    spi_device_polling_transmit(spi_current, &t);
    gpio_set_level(PIN_DW_CS, 1);

    return DWT_SUCCESS;
}

/* wakeup_device_with_io — assert WAKEUP line to wake DWM3000 from
 * DEEPSLEEP.  Per the DW3000 datasheet the WAKEUP pin must be held high
 * for ≥ 500 µs; we use 1 ms to be safe.                               */
void wakeup_device_with_io(void)
{
    #if PIN_DW_WAKEUP
    gpio_set_level(PIN_DW_WAKEUP, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(PIN_DW_WAKEUP, 0);
    #endif

    /* Allow time for the DW3000 SPI interface to become ready (~3 ms) */
    vTaskDelay(pdMS_TO_TICKS(3));
}

/* decamutexon / decamutexoff — short critical-section guard used inside
 * deca_device.c to protect multi-byte SPI accesses from concurrent ISR
 * activity.  On ESP32 we use a spinlock-based portMUX.                */
decaIrqStatus_t decamutexon(void)
{
    portENTER_CRITICAL(&spi_mux);
    return (decaIrqStatus_t)1;
}

void decamutexoff(decaIrqStatus_t s)
{
    (void)s;
    portEXIT_CRITICAL(&spi_mux);
}

/* ====================================================================
 * Legacy dwOps vtable
 *
 * DW3000 driver does not use the vtable; it is kept here as an empty
 * sentinel so that any code referencing the extern dwOps symbol links.
 * ==================================================================== */
dwOps_t dwOps = {
    .spiRead     = NULL,
    .spiWrite    = NULL,
    .spiSetSpeed = NULL,
    .delayms     = NULL,
    .reset       = NULL,
};
