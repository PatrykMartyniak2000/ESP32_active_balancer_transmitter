#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_defs.h"
#include "esp_adc/adc_oneshot.h"

#define SPP_TAG "ESP32_SPP_ACCEPTOR"
#define SPP_SERVER_NAME "ESP32_SPP_SERVER"

#define ADC_CHANNEL_COUNT 10 // Number of ADC channels to measure

static uint32_t spp_handle = 0; // Handle for the SPP connection

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static const adc_channel_t adc_channels[ADC_CHANNEL_COUNT] = {
    ADC_CHANNEL_0, // GPIO4
    ADC_CHANNEL_1, // GPIO0
    ADC_CHANNEL_2, // GPIO2
    ADC_CHANNEL_3, // GPIO15
    ADC_CHANNEL_4, // GPIO13
    ADC_CHANNEL_5, // GPIO12
    ADC_CHANNEL_6, // GPIO14
    ADC_CHANNEL_7, // GPIO27
    ADC_CHANNEL_8, // GPIO25
    ADC_CHANNEL_9  // GPIO26
};

static adc_oneshot_unit_handle_t adc_handle;

// Function to initialize the ADC Oneshot Driver
void init_adc_oneshot()
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        adc_oneshot_chan_cfg_t channel_config = {
            .bitwidth = ADC_BITWIDTH_12, // 12-bit resolution
            .atten = ADC_ATTEN_DB_12, // 0-3.3V range
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adc_channels[i], &channel_config));
    }
}

// Function to measure raw ADC values from 10 channels
void measure_adc_channels(int *adc_raw_values)
{
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        esp_err_t ret = adc_oneshot_read(adc_handle, adc_channels[i], &adc_raw_values[i]);
        if (ret == ESP_OK) {
            ESP_LOGI(SPP_TAG, "Channel %d (GPIO%d): %d", i, adc_channels[i] + 4, adc_raw_values[i]);
        } else {
            ESP_LOGE(SPP_TAG, "Failed to read ADC channel %d: %s", i, esp_err_to_name(ret));
            adc_raw_values[i] = -1; // Indicate an error for this channel
        }
    }
}

// SPP callback function to handle Bluetooth events
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_gap_set_device_name(SPP_SERVER_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        spp_handle = param->srv_open.handle; // Store the connection handle

        // Measure ADC values and send them to the connected device
        if (spp_handle != 0) {
            int adc_raw_values[ADC_CHANNEL_COUNT];
            measure_adc_channels(adc_raw_values);

            // Send the ADC values as a comma-separated string
            char data_to_send[256];
            int offset = 0;
            for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
                offset += snprintf(data_to_send + offset, sizeof(data_to_send) - offset, "%d,", adc_raw_values[i]);
            }
            data_to_send[offset - 1] = '\0'; // Replace the last comma with a null terminator
            esp_spp_write(spp_handle, strlen(data_to_send), (uint8_t *)data_to_send);
        }
        break;

    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%" PRIu32,
                 param->data_ind.len, param->data_ind.handle);

        // Process received data
        char received_data[128];
        memcpy(received_data, param->data_ind.data, param->data_ind.len);
        received_data[param->data_ind.len] = '\0'; // Null-terminate the string
        ESP_LOGI(SPP_TAG, "Received data: %s", received_data);

        // Echo the received data back to the sender
        if (spp_handle != 0) {
            esp_spp_write(spp_handle, param->data_ind.len, param->data_ind.data);
        }
        break;

    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;

    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong=%d", param->cong.cong);
        break;

    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize ADC Oneshot Driver
    init_adc_oneshot();

    // Release BLE memory if not needed
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to release BLE memory: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize the Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to initialize controller: %s", esp_err_to_name(ret));
        return;
    }

    // Enable the Bluetooth controller in Classic BT mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to enable controller: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    // Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    // Register the SPP callback
    ret = esp_spp_register_callback(spp_callback);
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to register SPP callback: %s", esp_err_to_name(ret));
        return;
    }

    // Enhanced SPP initialization
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,           // Use callback mode
        .enable_l2cap_ertm = true,        // Enable L2CAP ERTM
        .tx_buffer_size = ESP_SPP_MAX_TX_BUFFER_SIZE, // Set TX buffer size
    };

    ret = esp_spp_enhanced_init(&spp_cfg);
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to initialize SPP: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(SPP_TAG, "Bluetooth initialized successfully");

    // Main application loop
    while (true) {
        if (spp_handle != 0) {
            // Measure ADC values and send them periodically
            int adc_raw_values[ADC_CHANNEL_COUNT];
            measure_adc_channels(adc_raw_values);

            // Send the ADC values as a comma-separated string
            char data_to_send[256];
            int offset = 0;
            for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
                offset += snprintf(data_to_send + offset, sizeof(data_to_send) - offset, "%d,", adc_raw_values[i]);
            }
            data_to_send[offset - 1] = '\0'; // Replace the last comma with a null terminator
            esp_spp_write(spp_handle, strlen(data_to_send), (uint8_t *)data_to_send);
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds
    }
}
