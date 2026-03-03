# Main application component for ESP-IDF.
# Only the ported source files are listed here — STM32-specific files
# (eeprom.c, lps25h.c, usb*, f0/, production_test.c, …) are omitted.

idf_component_register(
    SRCS
        # Application
        "main.c"
        "cfg.c"
        "dwOps.c"
        "uwb.c"
        "led.c"
        "button.c"
        "lpp.c"
        "mac.c"
        # UWB algorithms
        "uwb_twr_anchor.c"
        "uwb_twr_tag.c"
        "uwb_tdoa_anchor2.c"
        "uwb_tdoa_anchor3.c"
        "uwb_tdoa_tag.c"
        "uwb_sniffer.c"
        # libdw3000 shim + Qorvo decadriver
        "../lib/libdw3000/src/libdw3000_shim.c"
        "../lib/libdw3000/decadriver/deca_device.c"
    INCLUDE_DIRS
        # Project headers
        "../inc"
        # libdw3000 shim + decadriver headers
        "../lib/libdw3000/inc"
        "../lib/libdw3000/decadriver"
    REQUIRES
        esp_driver_rmt
        esp_wifi
)
