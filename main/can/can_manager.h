#pragma once

#include "can_interface.h"
#include "esp_err.h"

#define CAN_MANAGER_MAX_INTERFACES  4

// Initialize the CAN manager (starts USB capture)
esp_err_t can_manager_init(void);

// Register a CAN interface with the manager. Takes ownership of lifecycle.
esp_err_t can_manager_add_interface(can_interface_t *iface);

// Initialize and start all registered interfaces
esp_err_t can_manager_start(void);

// Stop all interfaces
esp_err_t can_manager_stop(void);

#ifdef CONFIG_DASHKIT_USB_TX
// Transmit a frame on the interface with the given bus_id.
// Returns ESP_ERR_NOT_FOUND if no interface matches bus_id.
esp_err_t can_manager_send(uint8_t bus_id, const can_frame_t *frame);
esp_err_t can_manager_poll_tx(uint8_t bus_id, bool abort, can_tx_diagnostics_t *diagnostics);
#endif
