#ifndef __DECA_SPI_H__
#define __DECA_SPI_H__

/*
 * Platform declarations for Qorvo DW3000 decadriver.
 * Implementations live in src/dwOps.c.
 */
#include <stdint.h>
#include "deca_device_api.h"

/* Platform mutex token type. */
typedef uint32_t decaIrqStatus_t;

int  writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
                uint16_t bodyLength,   const uint8_t *bodyBuffer);
int  readfromspi(uint16_t headerLength, const uint8_t *headerBuffer,
                 uint16_t readlength,   uint8_t *readBuffer);
void deca_sleep(unsigned int time_ms);
void wakeup_device_with_io(void);
decaIrqStatus_t decamutexon(void);
void            decamutexoff(decaIrqStatus_t s);

#endif
