#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "gps.h"
#include "sim800l.h"
#include "gyro.h"
#include "ble_scan.h"

#include "state_vals.h"

static const char *TAG = "config";

DRAM_ATTR config_guard_t config_guard;

current_state_vals_t current_state_vals = {
    .loop_state = 0,
    .ble_scan_finish_time = 0,
    .ble_permited_num = 0,
    .is_web_host_runnung = false,
};

bool do_save_config = false;

//========================================

#define MAX_JSON_STRING_FOR_PHONES (MAX_ENABLED_PHONES_NUM * MAX_PHONE_SZ + MAX_ENABLED_PHONES_NUM)
char *config_to_json()
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "timezone", cJSON_CreateString(config_guard.timezone));
    cJSON_AddNumberToObject(root, "do_hang_up", config_guard.do_hang_up);
    cJSON_AddNumberToObject(root, "alarm_sms_period", config_guard.alarm_sms_period);
    cJSON_AddNumberToObject(root, "web_host_active_period", config_guard.web_host_active_period);
    cJSON_AddNumberToObject(root, "stop_alrm_sms_send", config_guard.stop_alrm_sms_send);
    cJSON_AddNumberToObject(root, "treshold_low_frq", config_guard.treshold_low_frq);
    cJSON_AddNumberToObject(root, "treshold_hight_frq", config_guard.treshold_hight_frq);
    cJSON_AddNumberToObject(root, "treshold_low_volt", config_guard.treshold_low_volt);
    cJSON_AddNumberToObject(root, "treshold_low_volt_step", config_guard.treshold_low_volt_step);

    char *str_tmp = malloc(MAX_JSON_STRING_FOR_PHONES + 6);
    str_tmp[0] = 0;
    for (int i = 0; i < config_guard.alarm_phones_num; i++)
    {
        if (i != 0)
        {
            strcat(str_tmp, ",");
        }
        int l = strlen(str_tmp);
        strncpy(str_tmp + l, config_guard.alarm_phones[i], MAX_JSON_STRING_FOR_PHONES - l);
    }
    cJSON_AddItemToObject(root, "alarm_phones", cJSON_CreateString(str_tmp));
    free(str_tmp);

    cJSON_AddItemToObject(root, "enabled_phones", cJSON_CreateString(config_guard.enabled_phones));
    cJSON_AddItemToObject(root, "ble_enabled", cJSON_CreateString(config_guard.ble_enabled));
    char *str = cJSON_Print(root);
    cJSON_Delete(root);
    return str; // !!!   free(str);
}

#define MAX_JSON_STRING_FOR_BLE_LIST (4096)
static char *fill_BLE_list(char *str_tmp, ble_scan_data_t *ble_scan_data, int num, bool is_for_html)
{
    str_tmp[0] = 0;
    for (int i = 0; i < num; i++)
    {
        if (i != 0)
        {
            strcat(str_tmp, is_for_html ? "<BR>" : "\n");
        }
        int l = strlen(str_tmp);
        scan_record_to_str(&ble_scan_data[i], str_tmp + l, MAX_JSON_STRING_FOR_BLE_LIST - l, is_for_html);
    }
    return str_tmp;
}

char *ble_state()
{
    char *str_tmp = malloc(MAX_JSON_STRING_FOR_BLE_LIST * 2);
    strcpy(str_tmp, "ok:\n");
    fill_BLE_list(str_tmp + strlen(str_tmp), &current_state_vals.ble_permited[0], current_state_vals.ble_permited_num, false);
    strcat(str_tmp, "\nall:\n");
    fill_BLE_list(str_tmp + strlen(str_tmp), &ble_scan_result.scan_record[0], ble_scan_result.scan_records_num, false);
    return str_tmp;
}

char *json_state()
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "gsm_sim_in_slot", gsm_state.sim_in_slot);
    cJSON_AddNumberToObject(root, "gsm_signal_level", gsm_state.signal_level);
    cJSON_AddNumberToObject(root, "gsm_voltage", gsm_state.voltage);
    cJSON_AddStringToObject(root, "sms_phone", gsm_state.sms_phone_in);
    cJSON_AddStringToObject(root, "sms_body", gsm_state.sms_body_in);

    cJSON_AddNumberToObject(root, "gyro_low_frq", gyroscope_state.max_dt_between_frame);
    cJSON_AddNumberToObject(root, "gyro_hight_frq", gyroscope_state.max_dt_in_frame);

    char *str_tmp = malloc(MAX_JSON_STRING_FOR_BLE_LIST + 6);
    cJSON_AddItemToObject(root, "ble_permited", cJSON_CreateString(fill_BLE_list(str_tmp, &current_state_vals.ble_permited[0], current_state_vals.ble_permited_num, true)));
    cJSON_AddItemToObject(root, "ble_detected", cJSON_CreateString(fill_BLE_list(str_tmp, &ble_scan_result.scan_record[0], ble_scan_result.scan_records_num, true)));
    free(str_tmp);

    char *str = cJSON_Print(root);
    cJSON_Delete(root);
    return str; // !!!   free(str);
}
//========================================

void parse_config_json(char *json)
{
    cJSON *root = cJSON_Parse(json);
    cJSON *item = cJSON_GetObjectItem(root, "timezone");
    if (item->valuestring != NULL)
    {
        strncpy(config_guard.timezone, item->valuestring, sizeof(config_guard.timezone) - 1);
    }
    config_guard.do_hang_up = cJSON_GetObjectItem(root, "do_hang_up")->valueint;
    config_guard.alarm_sms_period = cJSON_GetObjectItem(root, "alarm_sms_period")->valueint;
    if (config_guard.alarm_sms_period < (BLE_SCAN_TIME_SEC + 2))
    {
        ESP_LOGI(TAG, "config_guard.alarm_sms_period %d -> %d", config_guard.alarm_sms_period, BLE_SCAN_TIME_SEC + 2);
        config_guard.alarm_sms_period = BLE_SCAN_TIME_SEC + 2;
    }

    config_guard.web_host_active_period = cJSON_GetObjectItem(root, "web_host_active_period")->valueint;
    config_guard.stop_alrm_sms_send = cJSON_GetObjectItem(root, "stop_alrm_sms_send")->valueint;
    config_guard.treshold_low_frq = cJSON_GetObjectItem(root, "treshold_low_frq")->valueint;
    config_guard.treshold_hight_frq = cJSON_GetObjectItem(root, "treshold_hight_frq")->valueint;
    config_guard.treshold_low_volt = cJSON_GetObjectItem(root, "treshold_low_volt")->valueint;
    config_guard.treshold_low_volt_step = cJSON_GetObjectItem(root, "treshold_low_volt_step")->valueint;

    item = cJSON_GetObjectItem(root, "alarm_phones");
    config_guard.alarm_phones_num = 0;
    memset(config_guard.alarm_phones, 0, sizeof(config_guard.alarm_phones));
    if (item->valuestring != NULL)
    {
        for (int i = 0, j = 0; config_guard.alarm_phones_num < MAX_ENABLED_PHONES_NUM; i++)
        {
            char c = item->valuestring[i];
            if (c == '+' || (c >= '0' && c <= '9'))
            {
                if (j < (MAX_PHONE_SZ - 1))
                {
                    config_guard.alarm_phones[config_guard.alarm_phones_num][j++] = c;
                }
            }
            else if (c == 0)
            {
                if (config_guard.alarm_phones[config_guard.alarm_phones_num][0] != 0)
                {
                    config_guard.alarm_phones_num++;
                }
                break;
            }
            else
            {
                if (config_guard.alarm_phones[config_guard.alarm_phones_num][0] != 0)
                {
                    config_guard.alarm_phones_num++;
                    j = 0;
                }
            }
        }
    }

    item = cJSON_GetObjectItem(root, "enabled_phones");
    if (item->valuestring == NULL)
    {
        config_guard.enabled_phones[0] = 0;
    }
    else
    {
        strncpy(config_guard.enabled_phones, item->valuestring, sizeof(config_guard.enabled_phones) - 1);
        config_guard.enabled_phones[sizeof(config_guard.enabled_phones) - 1] = 0;
    }

    item = cJSON_GetObjectItem(root, "ble_enabled");
    if (item->valuestring == NULL)
    {
        config_guard.ble_enabled[0] = 0;
    }
    else
    {
        strncpy(config_guard.ble_enabled, item->valuestring, sizeof(config_guard.ble_enabled) - 1);
        config_guard.ble_enabled[sizeof(config_guard.ble_enabled) - 1] = 0;
    }
}

//========================================================
static bool mem_in_mem(const uint8_t *data, size_t data_sz, uint8_t *sub_data, size_t sub_data_sz)
{
    int delta = data_sz - sub_data_sz;
    for (int i = 0; i <= delta; i++)
    {
        if (!memcmp(data + i, sub_data, sub_data_sz))
        {
            return true;
        }
    }
    return false;
}

bool has_ble_permit()
{
    current_state_vals.ble_permited_num = 0;
    for (int rec_num = 0; rec_num < ble_scan_result.scan_records_num; rec_num++)
    {
        char *xStr = config_guard.ble_enabled;
        int l = strlen(config_guard.ble_enabled);
        for (int ofs = 0; ofs < l;)
        {
            uint8_t bin[MAX_BLE_INFO_SZ / 2 + 1];
            int sz = hex2bin(xStr + ofs, bin, l);
            int b_sz = sz / 2;
            if (mem_in_mem(ble_scan_result.scan_record[rec_num].bda, sizeof(ble_scan_result.scan_record[rec_num].bda), bin, b_sz) ||
                mem_in_mem(ble_scan_result.scan_record[rec_num].ble_adv, sizeof(ble_scan_result.scan_record[rec_num].ble_adv), bin, b_sz))
            {
                memcpy(&current_state_vals.ble_permited[current_state_vals.ble_permited_num++], &ble_scan_result.scan_record[rec_num], sizeof(ble_scan_data_t));
                break;
            }
            ofs += sz + 1;
        }
    }
    return current_state_vals.ble_permited_num > 0;
}

//=====================================
bool is_phone_not_permited(char *phone)
{
    if (is_set_send_sms_flag())
    {
        ESP_LOGI(TAG, "Busy to send SMS. ignore received SMS");
        return true;
    }
    if (strstr(config_guard.enabled_phones, phone) != NULL)
    {
        return false;
    }
    ESP_LOGI(TAG, "phone not permited");
    return true;
}
