/*
 * deca_spi.h — ESP32-S3 platform SPI declarations for the Qorvo DW3000 driver
 *
 * The DW3000 decadriver (deca_device.c) calls the four functions declared
 * here whenever it needs to access the chip over SPI or needs to sleep.
 * The implementations live in src/dwOps.c alongside the ESP-IDF SPI bus
 * initialisation code.
 *
 * Function naming and signatures match the Qorvo SDK convention so that
 * deca_device.c can be used unmodified.
 */

#ifndef __DECA_SPI_H__
#define __DECA_SPI_H__

#include <stdint.h>
#include "deca_device_api.h"

/* Platform mutex token type (returned by decamutexon, consumed by decamutexoff). */
typedef uint32_t decaIrqStatus_t;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * writetospi()
 *
 * Write headerLength bytes from headerBuffer followed by bodyLength bytes
 * from bodyBuffer to the DW3000 SPI interface.
 *
 * Returns DWT_SUCCESS (0) on success, DWT_ERROR (-1) on failure.
 */
int writetospi(uint16_t        headerLength,
               const uint8_t  *headerBuffer,
               uint16_t        bodyLength,
               const uint8_t  *bodyBuffer);

/*
 * readfromspi()
 *
 * Write headerLength bytes from headerBuffer then read readlength bytes into
 * readBuffer from the DW3000 SPI interface.
 *
 * Returns DWT_SUCCESS (0) on success, DWT_ERROR (-1) on failure.
 */
int readfromspi(uint16_t        headerLength,
                const uint8_t  *headerBuffer,
                uint16_t        readlength,
                uint8_t        *readBuffer);

/*
 * deca_sleep()
 *
 * Block for the requested number of milliseconds.
 * Uses FreeRTOS vTaskDelay() internally.
 */
void deca_sleep(unsigned int time_ms);

/*
 * wakeup_device_with_io()
 *
 * Assert the WAKEUP line (or assert CS) to wake the DW3000 from DEEPSLEEP.
 * The actual GPIO toggling is in src/dwOps.c.
 */
void wakeup_device_with_io(void);

/*
 * decamutexon() / decamutexoff()
 *
 * Short critical-section wrappers used inside the decadriver to protect
 * multi-byte SPI operations against concurrent ISR access.
 */
decaIrqStatus_t decamutexon(void);
void            decamutexoff(decaIrqStatus_t s);

#ifdef __cplusplus
}
#endif

#endif /* __DECA_SPI_H__ */
