#pragma once

#include "can_interface.h"

esp_err_t usb_capture_init(void);
void usb_capture_enable_tx(void);
void usb_capture_submit(const can_tagged_frame_t *frame);
void usb_capture_overflow(uint8_t bus_id);
