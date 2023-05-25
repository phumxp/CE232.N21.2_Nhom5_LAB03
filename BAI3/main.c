#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#define COMPANY_IDENTIFIER_LENGTH 2
static const char *DEVICE_NAME = "NHA HANG";
static const char *NOTIFICATION = "Khuyen mai 50%";
static const uint8_t COMPANY_IDENTIFIER[COMPANY_IDENTIFIER_LENGTH] = {0x4C, 0x00};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .include_name = true,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
uint8_t *createManufacturerData(uint16_t *manufacturer_len,
                                const uint8_t *data, uint16_t data_len,
                                const uint8_t *companyIdentifier, uint16_t companyIdentifier_len);

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));
    uint8_t *manufacturer_data = createManufacturerData(&adv_data.manufacturer_len,
                                                        (const uint8_t *)NOTIFICATION,
                                                        strlen(NOTIFICATION),
                                                        COMPANY_IDENTIFIER,
                                                        COMPANY_IDENTIFIER_LENGTH);
    adv_data.p_manufacturer_data = manufacturer_data;
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    free(manufacturer_data);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        printf("ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT\n");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        printf("ESP_GAP_BLE_ADV_START_COMPLETE_EVT\n");
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
            printf("Advertising started\n\n");
        else
            printf("Unable to start advertising process, error code %d\n\n", param->scan_start_cmpl.status);
        break;

    default:
        break;
    }
}

uint8_t *createManufacturerData(uint16_t *manufacturer_len,
                                const uint8_t *data, uint16_t data_len,
                                const uint8_t *companyIdentifier, uint16_t companyIdentifier_len)
{
    *manufacturer_len = data_len + companyIdentifier_len;
    uint8_t *manufacturer_data = (uint8_t *)malloc((*manufacturer_len) * sizeof(uint8_t));
    memcpy(manufacturer_data, companyIdentifier, companyIdentifier_len);
    memcpy(manufacturer_data + companyIdentifier_len, data, data_len);
    return manufacturer_data;
}
