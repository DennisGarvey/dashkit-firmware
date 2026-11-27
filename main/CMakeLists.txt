set(TX_SRCS "")
if(CONFIG_DASHKIT_USB_TX)
    list(APPEND TX_SRCS "can/mcp251xfd_classic_rx.c")
endif()

idf_component_register(
    SRCS
        "main.c"
        "led.c"
        "can/mcp251xfd.c"
        ${TX_SRCS}
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
