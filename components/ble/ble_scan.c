#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"

#include "ble_scan.h"

ble_scan_result_t ble_scan_result = {
    .scan_records_num = 0};

static const char *TAG = "ble_scan";

//====================
#define ENDIAN_CHANGE_U16(x) ((((x) & 0xFF00) >> 8) + (((x) & 0xFF) << 8))

typedef struct
{
    uint8_t flags[3];
    uint8_t length;
    uint8_t type;
    uint16_t company_id;
    uint16_t beacon_type;
} __attribute__((packed)) esp_ble_ibeacon_head_t;

typedef struct
{
    uint8_t proximity_uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t measured_power;
} __attribute__((packed)) esp_ble_ibeacon_vendor_t;

typedef struct
{
    esp_ble_ibeacon_head_t ibeacon_head;
    esp_ble_ibeacon_vendor_t ibeacon_vendor;
} __attribute__((packed)) esp_ble_ibeacon_t;

/* Constant part of iBeacon data */
esp_ble_ibeacon_head_t ibeacon_common_head = {
    .flags = {0x02, 0x01, 0x06},
    .length = 0x1A,
    .type = 0xFF,
    .company_id = 0x004C,
    .beacon_type = 0x1502};

bool esp_ble_is_ibeacon_packet(uint8_t *adv_data, uint8_t adv_data_len)
{
    bool result = false;

    if ((adv_data != NULL) && (adv_data_len == 0x1E))
    {
        if (!memcmp(adv_data, (uint8_t *)&ibeacon_common_head, sizeof(ibeacon_common_head)))
        {
            result = true;
        }
    }

    return result;
}
//====================

void esp_ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    // ESP_LOGI(TAG, "-------------esp_ble_gap_cb event: %d", event);
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            // ESP_LOGI(TAG, "++++++++++++++ ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
            esp_ble_gap_start_scanning(BLE_SCAN_TIME_SEC);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to set scan parameters");
        }
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // ESP_LOGI(TAG, "++++++++++++++ ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Scan start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // ESP_LOGI(TAG, "++++++++++++++ ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Adv start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        // ESP_LOGI(TAG, "++++++++++++++ ESP_GAP_BLE_SCAN_RESULT_EVT %d", param->scan_rst.search_evt);
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
        {
            if (esp_ble_is_ibeacon_packet(param->scan_rst.ble_adv, param->scan_rst.adv_data_len))
            {
                ESP_LOGI(TAG, "----------iBeacon Found----------");
            }

            if (ble_scan_result.scan_records_num < (MAX_SCAN_RECORDS - 1))
            {
                int n = ble_scan_result.scan_records_num;
                memcpy(ble_scan_result.scan_record[n].bda, param->scan_rst.bda, sizeof(ble_scan_result.scan_record[n].bda));
                ble_scan_result.scan_record[n].dev_type = param->scan_rst.dev_type;
                ble_scan_result.scan_record[n].ble_addr_type = param->scan_rst.ble_addr_type;
                ble_scan_result.scan_record[n].rssi = param->scan_rst.rssi;
                memcpy(ble_scan_result.scan_record[n].ble_adv, param->scan_rst.ble_adv, sizeof(ble_scan_result.scan_record[n].ble_adv));
                ble_scan_result.scan_records_num++;
            }
            else
            {
                ESP_LOGE(TAG, "Too many devices");
            }
            ESP_LOGI(TAG, ">> [" ESP_BD_ADDR_STR "] RSSI: %d devType:%d ble_addr_type:%d",
                     ESP_BD_ADDR_HEX(param->scan_rst.bda),
                     param->scan_rst.rssi, param->scan_rst.dev_type, param->scan_rst.ble_addr_type);
        }
        else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
        {
            ESP_LOGI(TAG, "-- scan complete");
        }
        break;
    default:
        break;
    }
}

//=======================================================

static void bin2hex(uint8_t *bin, char *xStr, int len)
{
    static uint8_t xlat[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

    for (int i = 0; i < len; i++)
    {
        *(xStr++) = xlat[(*bin) >> 4];
        *(xStr++) = xlat[(*bin) & 0x0F];
        ++bin;
    }
    *(xStr) = 0;
}

static uint8_t char2byte(char b)
{
    if (b >= 0x61)
        b -= 0x20; /* uppercase -> lowercase */
    if (b >= 0x41)
        b -= 0x37; /* alpha */
    else
        b -= 0x30; /* digit */

    return b;
}

int hex2bin(char *xStr, uint8_t *bin, int len)
{
    uint8_t b;
    int flag = 0, i = 0;

    while (i < len) //(len--)
    {
        if ((b = char2byte(*xStr++)) > 15) // is non hex symbol ?
        {
            return i;
        }

        if (flag) // low tetrada
        {
            *bin <<= 4;
            *bin += b;
            bin++;
        }
        else
        {
            *bin = b;
        }

        i++;
        flag ^= 1; // invert flag
    }
    return i;
}

char *scan_record_to_str(ble_scan_data_t *p, char *str, int max_sz, bool is_for_html)
{
    if (is_for_html)
    {
        snprintf(str, max_sz, "<b>adr:[%02X%02X%02X%02X%02X%02X] RSSI:%d (%d/%d)</b> ", ESP_BD_ADDR_HEX(p->bda), p->rssi, p->dev_type, p->ble_addr_type);
    }
    else
    {
        snprintf(str, max_sz, "[%02X%02X%02X%02X%02X%02X] RSSI:%d ", ESP_BD_ADDR_HEX(p->bda), p->rssi);
    }
    for (int n = 0; p->ble_adv[n] != 0 && n < sizeof(p->ble_adv);)
    {
        int l = strlen(str);
        uint8_t data_len = p->ble_adv[n];
        uint8_t data_type = p->ble_adv[n + 1];
        snprintf(str + l, max_sz - l, " %0x(%u)=", data_type, data_len);
        l = strlen(str);
        if (data_type == ESP_BLE_AD_TYPE_NAME_CMPL)
        {
            if ((l + data_len + 3) < max_sz)
            {
                str[l++] = '\'';
                memcpy(str + l, p->ble_adv + n + 2, data_len - 1);
                l += data_len - 1;
                str[l++] = '\'';
                str[l++] = 0;
            }
        }
        else
        {
            if (l + data_len * 2 + 1 > max_sz)
            {
                ESP_LOGE(TAG, "Too short string size=[%d] for scan_record_to_str", max_sz);
                return str;
            }
            bin2hex(p->ble_adv + n + 1, str + l, data_len);
        }
        n += 1 + data_len;
    }
    return str;
}

void do_ble_scan()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_ble_scan_params_t ble_scan_params = {
        /*
            .scan_type              = BLE_SCAN_TYPE_PASSIVE,
            .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval          = 0x06E1,
            .scan_window            = 0x06E0,
            .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE
        */
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
        .scan_interval = 30, // ms
        .scan_window = 20,   // ms
    };
    ble_scan_result.scan_records_num = 0;
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&ble_scan_params));
}

void init_ble_scan()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bluedroid_cfg));

    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_ble_gap_cb));
}
