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
#include <esp_adc/adc_oneshot.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_defs.h"

#include "time.h"
#include "sys/time.h"

// ADC variables
static int adc_raw_data[10];

uint8_t adc_channels[10] = {
    ADC_CHANNEL_0,
    ADC_CHANNEL_1,
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_5,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7,
    ADC_CHANNEL_8,
    ADC_CHANNEL_9,
};

esp_err_t ret;

adc_oneshot_unit_handle_t adc_oneshot_unit_handle;

// ADC config

adc_oneshot_unit_init_cfg_t adc_oneshot_unit_init_cfg = {
    .unit_id = ADC_UNIT_2,
    .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};

adc_oneshot_chan_cfg_t adc_oneshot_chan_cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_12,
};

// BLUETOOTH variables
#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

static uint32_t spp_handle = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        esp_bt_gap_set_device_name(SPP_SERVER_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%" PRIu32,
        param->data_ind.len, param->data_ind.handle);
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong=%d", param->cong.cong);
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        spp_handle = param->srv_open.handle;
        break;
    default:
        break;
    }
}

void app_main(void)
{
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = ESP_SPP_MAX_TX_BUFFER_SIZE,
    };
    // Initialize SPP
    ret = esp_spp_enhanced_init(&bt_spp_cfg);
    if (ret) {
        ESP_LOGE(SPP_TAG, "Failed to initialize SPP: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(SPP_TAG, "Bluetooth initialized successfully");

    // ADC initialization and configuration
    ret = adc_oneshot_new_unit(&adc_oneshot_unit_init_cfg, &adc_oneshot_unit_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(SPP_TAG, "Failed to initialize ADC unit: %s\n", esp_err_to_name(ret));
        return;
    }

    for (uint8_t i = 0; i < sizeof(adc_channels); i++) {
        ret = adc_oneshot_config_channel(adc_oneshot_unit_handle, adc_channels[i], &adc_oneshot_chan_cfg);
        if (ret != ESP_OK) {
            ESP_LOGI(SPP_TAG, "Failed to configure ADC channel %d: %s\n", adc_channels[i], esp_err_to_name(ret));
            return;
        }
    }

    while (true) {
        for (uint8_t i = 0; i < sizeof(adc_channels); i++) {
            ret = adc_oneshot_read(adc_oneshot_unit_handle, adc_channels[i], &adc_raw_data[i]);
            if (ret == ESP_OK) {
                printf("ADC[%d]: %d\n", adc_channels[i], adc_raw_data[i]);
                char data[20];
                snprintf(data, sizeof(data), "ADC[%d]: %d\n", adc_channels[i], adc_raw_data[i]);
                if (spp_handle != 0) {
                    esp_spp_write(spp_handle, strlen(data), (uint8_t *)data);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}
