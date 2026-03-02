#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <stdbool.h>
#include <stddef.h>

/* eeprom.c is not compiled in the ESP32 port.
 * Configuration is stored in ESP32 NVS via cfg.c using compile-time
 * build flags. This header is kept for reference only. */

bool eepromRead(int address, void* data, size_t length);
bool eepromWrite(int address, void* data, size_t length);
bool eepromTest();

#endif
