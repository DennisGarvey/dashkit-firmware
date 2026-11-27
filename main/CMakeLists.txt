idf_component_register(
    SRCS
        "main.c"
        "led.c"
        "can/mcp251xfd.c"
        "can/mcp251xfd_classic_rx.c"
        "can/can_manager.c"
        "capture/usb_capture.c"
    INCLUDE_DIRS "." "can" "capture"
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_driver_usb_serial_jtag
        esp_hw_support
        esp_timer
        app_update
)
