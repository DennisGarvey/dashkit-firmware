#include "can_manager.h"
#include "usb_capture.h"
#include "esp_log.h"

static const char *TAG = "can_mgr";

static can_interface_t *s_interfaces[CAN_MANAGER_MAX_INTERFACES];
static int              s_iface_count = 0;

// Callback invoked by CAN drivers from their RX tasks
static void on_frame_received(const can_tagged_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    usb_capture_submit(frame);
}

esp_err_t can_manager_init(void)
{
    return usb_capture_init();
}

esp_err_t can_manager_add_interface(can_interface_t *iface)
{
    if (s_iface_count >= CAN_MANAGER_MAX_INTERFACES) {
        return ESP_ERR_NO_MEM;
    }
    iface->set_rx_callback(iface, on_frame_received, NULL);
    s_interfaces[s_iface_count++] = iface;
    ESP_LOGI(TAG, "Added interface bus_id=%d (total: %d)", iface->bus_id, s_iface_count);
    return ESP_OK;
}

esp_err_t can_manager_start(void)
{
    for (int i = 0; i < s_iface_count; i++) {
        esp_err_t err = s_interfaces[i]->init(s_interfaces[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Init failed for bus %d: %s", s_interfaces[i]->bus_id, esp_err_to_name(err));
            return err;
        }
        err = s_interfaces[i]->start(s_interfaces[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Start failed for bus %d: %s", s_interfaces[i]->bus_id, esp_err_to_name(err));
            return err;
        }
    }
#ifdef CONFIG_DASHKIT_USB_TX
    usb_capture_enable_tx();
#endif
    return ESP_OK;
}

esp_err_t can_manager_stop(void)
{
    for (int i = 0; i < s_iface_count; i++) {
        s_interfaces[i]->stop(s_interfaces[i]);
    }
    return ESP_OK;
}

#ifdef CONFIG_DASHKIT_USB_TX
esp_err_t can_manager_send(uint8_t bus_id, const can_frame_t *frame)
{
    if (!frame) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < s_iface_count; i++) {
        if (s_interfaces[i]->bus_id == bus_id) {
            if (!s_interfaces[i]->send) return ESP_ERR_NOT_SUPPORTED;
            return s_interfaces[i]->send(s_interfaces[i], frame);
        }
    }
    ESP_LOGW(TAG, "send: no interface for bus_id=%d", bus_id);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t can_manager_poll_tx(uint8_t bus_id, bool abort, can_tx_diagnostics_t *diagnostics)
{
    for (int i = 0; i < s_iface_count; i++) {
        can_interface_t *iface = s_interfaces[i];
        if (iface->bus_id == bus_id && iface->poll_tx) return iface->poll_tx(iface, abort, diagnostics);
    }
    return ESP_ERR_NOT_FOUND;
}
#endif
