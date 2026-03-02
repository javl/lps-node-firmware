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
        "uwb_sniffer.c"
        # libdw1000 (vendored, included directly)
        "../lib/libdw1000/src/libdw1000.c"
        "../lib/libdw1000/src/libdw1000Spi.c"
    INCLUDE_DIRS
        # Project headers
        "../inc"
        # libdw1000 headers
        "../lib/libdw1000/inc"
    REQUIRES
        esp_driver_rmt
)
