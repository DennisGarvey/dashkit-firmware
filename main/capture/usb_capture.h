#pragma once

#include "can_interface.h"

void usb_capture_install_log_sink(void);
esp_err_t usb_capture_init(void);
void usb_capture_submit(const can_tagged_frame_t *frame);
void usb_capture_account_overflows(uint8_t bus_id, uint32_t count);
void usb_capture_account_spi(uint8_t bus_id, uint32_t clock_bits);
