#include "board.h"
#include "led.h"
#include "can_interface.h"
#include "can_manager.h"
#include "mcp251xfd.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"


#if !defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) || (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE != 1)
#error "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE must be enabled (a crashing OTA image would brick the device)"
#endif

static const char *TAG = "main";

// Commit the running image after it has survived a stable boot window (OTA
// rollback: PENDING_VERIFY -> VALID, so the next reboot does not roll back).
#define MARK_VALID_DELAY_MS  10000

static void mark_app_valid_cb(void *arg)
{
    (void)arg;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mark app valid failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "app confirmed valid (OTA rollback cleared)");
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "DashKit firmware starting... (reset reason=%d)",
             (int)esp_reset_reason());

    ESP_ERROR_CHECK(led_init());
    led_set_color(LED_COLOR_BLUE);

    ESP_ERROR_CHECK(can_manager_init());


    // CAN interface 0 (MCP2518FD)
    mcp251xfd_config_t can0_cfg = {
        .spi_host     = CAN0_SPI_HOST,
        .pin_mosi     = CAN0_PIN_MOSI,
        .pin_sclk     = CAN0_PIN_SCLK,
        .pin_miso     = CAN0_PIN_MISO,
        .pin_cs       = CAN0_PIN_CS,
        .pin_int      = CAN0_PIN_INT,
        .spi_clock_hz = CAN0_SPI_CLOCK_HZ,
        .osc_freq_hz  = CAN0_OSC_FREQ_HZ,
        .bitrate      = CAN_NOMINAL_BITRATE,
        .bitrate_data = CAN_FD_DATA_BITRATE,
        .bus_id       = 0,
    };
    can_interface_t *can0 = mcp251xfd_create(&can0_cfg);
    if (!can0) {
        ESP_LOGE(TAG, "Failed to create CAN0 interface");
        led_set_color(LED_COLOR_RED);
        return;
    }
    ESP_ERROR_CHECK(can_manager_add_interface(can0));

    // CAN interface 1 (MCP2518FD, shares SPI2 bus with CAN0)
    mcp251xfd_config_t can1_cfg = {
        .spi_host     = CAN1_SPI_HOST,
        .pin_mosi     = CAN1_PIN_MOSI,
        .pin_sclk     = CAN1_PIN_SCLK,
        .pin_miso     = CAN1_PIN_MISO,
        .pin_cs       = CAN1_PIN_CS,
        .pin_int      = CAN1_PIN_INT,
        .spi_clock_hz = CAN1_SPI_CLOCK_HZ,
        .osc_freq_hz  = CAN1_OSC_FREQ_HZ,
        .bitrate      = CAN_NOMINAL_BITRATE,
        .bitrate_data = CAN_FD_DATA_BITRATE,
        .bus_id       = 1,
    };
    can_interface_t *can1 = mcp251xfd_create(&can1_cfg);
    if (!can1) {
        ESP_LOGE(TAG, "Failed to create CAN1 interface");
        led_set_color(LED_COLOR_RED);
        return;
    }
    ESP_ERROR_CHECK(can_manager_add_interface(can1));

    // Start CAN
    esp_err_t err = can_manager_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN manager start failed");
        led_set_color(LED_COLOR_RED);
        return;
    }


    ESP_LOGI(TAG, "DashKit firmware ready");

    // Commit this image once it has survived the boot window (OTA rollback).
    const esp_timer_create_args_t mv_args = {
        .callback = mark_app_valid_cb,
        .name = "ota_mark_valid",
    };
    esp_timer_handle_t mv_timer;
    ESP_ERROR_CHECK(esp_timer_create(&mv_args, &mv_timer));
    ESP_ERROR_CHECK(esp_timer_start_once(mv_timer, MARK_VALID_DELAY_MS * 1000ULL));

    led_set_color(LED_COLOR_GREEN);
}
