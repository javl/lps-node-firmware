/*
 * libdw1000.h — redirect shim
 *
 * This file shadows the original lib/libdw1000/inc/libdw1000.h so that all
 * existing algorithm source files that do:
 *
 *   #include "libdw1000.h"
 *
 * transparently receive the DW3000-compatible shim API without any
 * per-file edits.
 *
 * lib/libdw1000 is excluded from the build via  lib_ignore = libdw1000
 * in platformio.ini; only lib/libdw3000 is compiled.
 */

#ifndef __LIBDW1000_H__
#define __LIBDW1000_H__

#include "libdw3000_shim.h"

#endif /* __LIBDW1000_H__ */
