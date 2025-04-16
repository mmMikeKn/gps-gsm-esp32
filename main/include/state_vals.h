#ifndef _config_VAL_H
#define _config_VAL_H

#include "esp_types.h"
#include "driver/gpio.h"

#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "cJSON.h"

#define GPIO_ACC_ON GPIO_NUM_25

#define VOLTAGE_BAT_ADC2_CHANNEL (ADC_CHANNEL_7)   // GPIO_NUM_27
#define VOLTAGE_BAT_ADC_ATTEN_DB ADC_ATTEN_DB_0 // 500kOm/50kOm 3*LiFePo4 < 700 mV

#define GPIO_LED GPIO_NUM_2
#define GPIO_ADD_LED GPIO_NUM_26 // POWER LED -> D25. hardware pcb soldering

#define NVS_STORAGE_NAME_CONFIG "GFG_GUARD"
#define NVS_REC_NAME_CONFIG "cfg"
#define NVS_REC_NAME_VOLTAGE "voltage_last"

#define REBOOT_PERIOD (20 * 60 * 1000L * 1000L)

#define MAX_ENABLED_PHONES_NUM (5)
#define MAX_PHONE_SZ (20)
#define MAX_BLE_DEV (5)
#define MAX_BLE_INFO_SZ (512)

typedef struct
{
    char timezone[20];
    uint16_t alarm_sms_period;
    uint16_t web_host_active_period;
    uint16_t treshold_low_frq, treshold_hight_frq;
    uint16_t treshold_low_volt, treshold_low_volt_step;

    char alarm_phones[MAX_ENABLED_PHONES_NUM][MAX_PHONE_SZ];
    int8_t alarm_phones_num;
    char enabled_phones[MAX_ENABLED_PHONES_NUM * MAX_PHONE_SZ + MAX_ENABLED_PHONES_NUM];
    char ble_enabled[MAX_BLE_INFO_SZ];
    bool stop_alrm_sms_send;
    bool do_hang_up;
    int8_t magic_val;
} config_guard_t;

typedef enum
{
    BLE_AUTHORIZED = (1 << 0),
    GYROSCOPE_SIGNAL = (1 << 1),
    BLE_SCAN_STATE_ON_ALARM = (1 << 2),
    ACC_WAS_ON = (1 << 3),
    ACC_WAS_OFF = (1 << 4),
} loop_states_t;

typedef struct
{
    uint32_t loop_state;
    int64_t ble_scan_finish_time, last_alarm_sms_time, last_web_active_time;
    uint8_t ble_permited_num;
    ble_scan_data_t ble_permited[MAX_BLE_DEV];
    bool is_web_host_runnung;
#ifdef VOLTAGE_BAT_ADC2_CHANNEL
    int bat_voltage;
#endif
} current_state_vals_t;

extern config_guard_t config_guard;
extern int16_t last_voltage_treshold;
extern current_state_vals_t current_state_vals;
extern bool do_save_config;

// extern bool read_configs_from_nvs();
// extern esp_err_t save_configs_to_nvs();
extern char *config_to_json();
extern void parse_config_json(char *json);
extern char *json_state();
extern char *ble_state();

extern bool has_ble_permit();
extern bool is_phone_not_permited(char *phone);
extern bool is_acc_on();

#endif