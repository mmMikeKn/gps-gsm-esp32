#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_pm.h"
// #include "mdns.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "sms_cmd.h"
#include "web_host.h"

static const char *TAG = "main";

#define IS_LOOP_STATE(x) ((current_state_vals.loop_state & x) != 0)
#define SET_LOOP_STATE(x) current_state_vals.loop_state |= x
#define RESET_LOOP_STATE(x) current_state_vals.loop_state &= ~x

int16_t last_voltage_treshold = -1;

static void build_alarm_sms(char *hd)
{
    strcpy(gsm_state.sms_body_out, " Alarm (");
    strcat(gsm_state.sms_body_out, hd);
    strcat(gsm_state.sms_body_out, ")\n");
    int l = strlen(gsm_state.sms_body_out);
    fill_current_state_in_sms(gsm_state.sms_body_out + l, sizeof(gsm_state.sms_body_out) - 1 - l);
    ESP_LOGI(TAG, "SMS prepared: %s", gsm_state.sms_body_out);
}

static void send_alarm_sms()
{
    if (config_guard.stop_alrm_sms_send)
    {
        ESP_LOGI(TAG, "DISABLED. alarm (n:%d) sms: %s", config_guard.alarm_phones_num, gsm_state.sms_body_out);
        return;
    }
    ESP_LOGI(TAG, "going to send alarm (n:%d) sms: %s", config_guard.alarm_phones_num, gsm_state.sms_body_out);
    char *tmp = malloc(sizeof(gsm_state.sms_body_out));
    memset(tmp, 0, sizeof(gsm_state.sms_body_out));
    strncpy(tmp, gsm_state.sms_body_out, sizeof(gsm_state.sms_body_out) - 1);
    for (size_t i = 0; i < config_guard.alarm_phones_num; i++)
    {
        strncpy(gsm_state.sms_phone_out, config_guard.alarm_phones[i], sizeof(gsm_state.sms_phone_out) - 1);
        strcpy(gsm_state.sms_body_out, tmp);
        set_send_sms_flag();
        ESP_LOGI(TAG, "going to send alarm to (%s)", gsm_state.sms_phone_out);
        for (int j = 0; j < 5 && !NO_OUT_SMS(); j++)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (!NO_OUT_SMS())
            {
                ESP_LOGI(TAG, "wait alarm sending alarm sms to (%s)", gsm_state.sms_phone_out);
            }
        }
    }
    gsm_state.sms_phone_out[0] = 0;
    free(tmp);
}

static void start_alarm_ble_scan()
{
    gpio_set_level(GPIO_LED, 1);
    SET_LOOP_STATE(BLE_SCAN_STATE_ON_ALARM);
    if (current_state_vals.ble_scan_finish_time < esp_timer_get_time())
    {
        current_state_vals.ble_scan_finish_time = esp_timer_get_time() + (BLE_SCAN_TIME_SEC + 1) * 1000L * 1000L; // in microseconds since boot
        do_ble_scan();
    }
    else
    {
        ESP_LOGI(TAG, "start_alarm_ble_scan. continue scan");
    }
}

//========================================
#define MAGIC_CONFIG_BYTE (42)

static void set_def_config()
{
    strcpy(config_guard.timezone, "UTC");
    config_guard.alarm_sms_period = BLE_SCAN_TIME_SEC * 2; // mast be > BLE_SCAN_TIME_SEC
    config_guard.web_host_active_period = 120;
    config_guard.treshold_low_frq = 50;
    config_guard.treshold_hight_frq = 150;
#ifdef VOLTAGE_BAT_ADC2_CHANNEL
    config_guard.treshold_low_volt = 6200; // LiFePo4 2*3.1
#else
    config_guard.treshold_low_volt = 3500; // Lion 3.5v
#endif
    config_guard.treshold_low_volt_step = 100;
    config_guard.alarm_phones_num = 0;
    config_guard.stop_alrm_sms_send = false;
    config_guard.magic_val = MAGIC_CONFIG_BYTE;
}

bool read_configs_from_nvs()
{
    nvs_handle_t handle;
    esp_err_t err, err2;

    ESP_LOGI(TAG, "Going to load config from " NVS_STORAGE_NAME_CONFIG);
    if ((err = nvs_open(NVS_STORAGE_NAME_CONFIG, NVS_READONLY, &handle)) != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open '" NVS_STORAGE_NAME_CONFIG "' for read error %s", esp_err_to_name(err));
        set_def_config();
        return false;
    }
    size_t required_size = sizeof(config_guard);
    err = nvs_get_blob(handle, NVS_REC_NAME_CONFIG, &config_guard, &required_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_get_blob '" NVS_STORAGE_NAME_CONFIG "'  error %s", esp_err_to_name(err));
        set_def_config();
    }
    else if (config_guard.magic_val != MAGIC_CONFIG_BYTE)
    {
        ESP_LOGE(TAG, "nvs_get_blob '" NVS_STORAGE_NAME_CONFIG "'  old data");
        set_def_config();
    }
    last_voltage_treshold = config_guard.treshold_low_volt;
    if ((err2 = nvs_get_i16(handle, NVS_REC_NAME_VOLTAGE, &last_voltage_treshold)) != ESP_OK)
    {
        ESP_LOGE(TAG, "last_voltage_treshold. nvs_get_i16 error %s", esp_err_to_name(err2));
    }
    else
    {
        ESP_LOGI(TAG, "saved last_voltage_treshold: %d", last_voltage_treshold);
    }
    nvs_close(handle);

    return err == ESP_OK;
}

static esp_err_t save_configs_to_nvs()
{
    nvs_handle_t handle;
    esp_err_t err;
    char *json = config_to_json();
    ESP_LOGI(TAG, "going to save config: %s", json);
    free(json);

    ESP_LOGI(TAG, "going to nvs_open '" NVS_STORAGE_NAME_CONFIG "'");
    if ((err = nvs_open(NVS_STORAGE_NAME_CONFIG, NVS_READWRITE, &handle)) != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open '" NVS_STORAGE_NAME_CONFIG "' for save error %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "going to nvs_set_blob '" NVS_REC_NAME_CONFIG "' ");
    if ((err = nvs_set_blob(handle, NVS_REC_NAME_CONFIG, &config_guard, sizeof(config_guard))) != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_set_blob  error %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "going to nvs_close '" NVS_REC_NAME_CONFIG "' ");
    nvs_close(handle);
    return ESP_OK;
}

static esp_err_t save_voltage_to_nvs()
{
    nvs_handle_t handle;
    esp_err_t err;
    ESP_LOGI(TAG, "going to save last_voltage_treshold: %d", last_voltage_treshold);

    ESP_LOGI(TAG, "going to nvs_open '" NVS_STORAGE_NAME_CONFIG "'");
    if ((err = nvs_open(NVS_STORAGE_NAME_CONFIG, NVS_READWRITE, &handle)) != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open for save error %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "going to nvs_set_i16");
    if ((err = nvs_set_i16(handle, NVS_REC_NAME_VOLTAGE, last_voltage_treshold)) != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_set_i16 error %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "going to nvs_close '" NVS_REC_NAME_CONFIG "' ");
    nvs_close(handle);
    return ESP_OK;
}

//------------------------------------------------------------------------------------------
static const gpio_config_t gpio_acc_cfg = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_INPUT,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .pull_up_en = GPIO_PULLUP_ENABLE, // for optocoupler
    .pin_bit_mask = (1ULL << GPIO_ACC_ON)};

static const gpio_config_t gpio_led_cfg = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
#ifdef GPIO_ADD_LED
    .pin_bit_mask = ((1ULL << GPIO_LED) | (1ULL << GPIO_ADD_LED))
#else
    .pin_bit_mask = (1ULL << GPIO_LED)
#endif
};

bool is_acc_on()
{
    return gpio_get_level(GPIO_ACC_ON) == 0; // optocoupler inversion
}

void app_main(void)
{
    gpio_config(&gpio_led_cfg);
    gpio_set_level(GPIO_LED, 1);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
            .max_freq_mhz = 160,
            .min_freq_mhz = 40,
            .light_sleep_enable = false
    };
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );
#endif // CONFIG_PM_ENABLE


    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    /* mDNS resolved url too slow. Direct access to the url http://192.194.4.1:80 is more fast
    // ./main/component.yml dependencies:  espressif/mdns: "^1.0.3"
    mdns_init();
    mdns_hostname_set("mm");
    mdns_instance_name_set("mm server");

    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}};
    ESP_ERROR_CHECK(mdns_service_add("MM-WebServer", "_http", "_tcp", 80, serviceTxtData, sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
    */
    read_configs_from_nvs();
    char *json = config_to_json();
    ESP_LOGI(TAG, "config: %s", json);
    free(json);
    //=============

    start_sim800l();
    start_gps();
    init_ble_scan();
    start_gyro();
#ifdef VOLTAGE_BAT_ADC2_CHANNEL
    adc_oneshot_unit_handle_t adc2_handle = NULL;
    adc_oneshot_unit_init_cfg_t init_adc_oneshot_chan = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_chan_cfg_t adc_oneshot_chan_cfg = {
        .atten = VOLTAGE_BAT_ADC_ATTEN_DB,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_adc_oneshot_chan, &adc2_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, VOLTAGE_BAT_ADC2_CHANNEL, &adc_oneshot_chan_cfg));
#endif

    gpio_config(&gpio_acc_cfg);
    if (!is_acc_on())
    {
        SET_LOOP_STATE(ACC_WAS_OFF);
    }
    gpio_set_level(GPIO_LED, 0);
    ESP_LOGI(TAG, "Going to enter in to endless loop");

    while (true)
    {
        //=====================================================
        int bat_voltage = 0;
#ifdef VOLTAGE_BAT_ADC2_CHANNEL
        bat_voltage = current_state_vals.bat_voltage; // mV
#else
        bat_voltage = gsm_state.voltage; // mV
#endif
        //        ESP_LOGI(TAG, "bat_voltage: %d last_voltage_treshold: %d NO_OUT_SMS:%d", bat_voltage, last_voltage_treshold, NO_OUT_SMS());
        if (bat_voltage != 0 && NO_OUT_SMS()) // recieved voltage form SIM8000L (direct from LiIon 3.7) or ADC from 2*Lifepo2 && ignore if another sms going to send
        {
            if (bat_voltage > (last_voltage_treshold + config_guard.treshold_low_volt_step) && bat_voltage < config_guard.treshold_low_volt)
            {
                last_voltage_treshold = bat_voltage - config_guard.treshold_low_volt_step;
                if (last_voltage_treshold > config_guard.treshold_low_volt)
                {
                    last_voltage_treshold = config_guard.treshold_low_volt;
                }
                ESP_LOGI(TAG, "charged. bat_voltage: %d. set last_voltage_treshold: %d", bat_voltage, last_voltage_treshold);
                save_voltage_to_nvs();
            }
            else if (!is_acc_on() // ignore if charge
                     && bat_voltage < last_voltage_treshold)
            {
                last_voltage_treshold = bat_voltage - config_guard.treshold_low_volt_step;
                if (save_voltage_to_nvs() == ESP_OK)
                {
                    build_alarm_sms("POWER");
                    send_alarm_sms();
                }
            }
        }
        //=====================================================
        if (is_acc_on())
        {
            if (IS_LOOP_STATE(ACC_WAS_OFF))
            {
                // depress contact bounce
                bool is_acc_on_val = true;
                for (int i = 0; i < 10 && is_acc_on_val; i++)
                {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    is_acc_on_val = is_acc_on();
                }
                if (is_acc_on_val)
                {
                    SET_LOOP_STATE(ACC_WAS_ON);
                    RESET_LOOP_STATE(ACC_WAS_OFF);
                    build_alarm_sms("ACC ON");
                    run_web_host();
                    start_alarm_ble_scan();
                }
                else
                {
                    ESP_LOGE(TAG, "ACC contact bounce");
                }
            }
        }
        else
        {
            SET_LOOP_STATE(ACC_WAS_OFF);
            if (IS_LOOP_STATE(ACC_WAS_ON))
            {
                ESP_LOGI(TAG, "ACC OFF");
                if (!IS_LOOP_STATE(BLE_AUTHORIZED))
                {
                    build_alarm_sms("ACC OFF");
                    start_alarm_ble_scan();
                }
            }
            RESET_LOOP_STATE(ACC_WAS_ON);
            RESET_LOOP_STATE(BLE_AUTHORIZED);
        }
        //=====================================================
        if (gsm_state.has_sms)
        {
#ifdef GPIO_ADD_LED
            gpio_set_level(GPIO_ADD_LED, 1);
#endif
            gpio_set_level(GPIO_LED, 1);
            handle_cmd_sms();
#ifdef GPIO_ADD_LED
            gpio_set_level(GPIO_ADD_LED, 0);
#endif
            gpio_set_level(GPIO_LED, 0);
            continue;
        }
        //=====================================================
        if (gsm_state.is_ring && !gsm_state.do_hang_up && !gsm_state.do_answer_incoming_call)
        {
            gpio_set_level(GPIO_LED, 1);
            gsm_state.is_ring = false;
            if (is_phone_not_permited(gsm_state.sms_phone_in))
            {
                gsm_state.do_hang_up = true;
                continue;
            }

            if (config_guard.do_hang_up)
            {
                gsm_state.do_hang_up = true;
                vTaskDelay(3000 / portTICK_PERIOD_MS);
                fill_current_state_in_sms(gsm_state.sms_body_out, sizeof(gsm_state.sms_body_out) - 1);
                send_echo_sms(gsm_state.ring_phone);
                vTaskDelay(3000 / portTICK_PERIOD_MS);
            }
            else
            {
                gsm_state.do_answer_incoming_call = true;
            }
            gpio_set_level(GPIO_LED, 0);
            continue;
        }
        //=====================================================
        if (!is_acc_on() && !IS_LOOP_STATE(BLE_SCAN_STATE_ON_ALARM) && !IS_LOOP_STATE(BLE_AUTHORIZED) &&
            current_state_vals.last_alarm_sms_time < esp_timer_get_time() &&
            gyroscope_state.ready_for_check &&
            (gyroscope_state.max_dt_between_frame > config_guard.treshold_low_frq && gyroscope_state.max_dt_in_frame > config_guard.treshold_hight_frq))
        {
            build_alarm_sms("GYRO");
            start_alarm_ble_scan();
            current_state_vals.last_alarm_sms_time = esp_timer_get_time() + config_guard.alarm_sms_period * 1000L * 1000L;
            ESP_LOGI(TAG, "next gyro check after %d sec", config_guard.alarm_sms_period);
        }
        //=====================================================
        if (IS_LOOP_STATE(BLE_SCAN_STATE_ON_ALARM))
        {
            if (has_ble_permit())
            {
                gpio_set_level(GPIO_LED, 0);
                RESET_LOOP_STATE(BLE_SCAN_STATE_ON_ALARM);
                if (is_acc_on())
                {
                    SET_LOOP_STATE(BLE_AUTHORIZED);
                }
                char *s = json_state();
                ESP_LOGI(TAG, "BLE permited: %s", s);
                free(s);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
            else
            {
                if (current_state_vals.ble_scan_finish_time < esp_timer_get_time())
                {
                    char *s = json_state();
                    ESP_LOGI(TAG, "NO BLE permits: %s", s);
                    free(s);
                    gpio_set_level(GPIO_LED, 0);
                    RESET_LOOP_STATE(BLE_SCAN_STATE_ON_ALARM);
                    send_alarm_sms();
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                }
                else
                {
                    ESP_LOGI(TAG, "No BLE permit detected");
                }
            }
        }
        //=====================================================
#ifdef VOLTAGE_BAT_ADC2_CHANNEL
        static int adc_raw_voltage;
        static int sum_v = 0;
        static int cnt_v = 0;
        // 2240 => 595mV (6.57V) k=3.4
        ret = adc_oneshot_read(adc2_handle, VOLTAGE_BAT_ADC2_CHANNEL, &adc_raw_voltage);
        if (ret != 0)
        {
            if (ret != ESP_ERR_TIMEOUT) // ESP_ERR_TIMEOUT if wifi is on
            {
                ESP_LOGI(TAG, "adc_oneshot_read ret: %X", ret);
            }
        }
        else
        {
            sum_v += adc_raw_voltage;
            cnt_v++;
            // ESP_LOGI(TAG, "ADC2 Raw Data: %d. over:%d Bat : %d mV", adc_raw_voltage, (int)(sum_v / cnt_v), (int)((float)sum_v / cnt_v / 0.34f));
            if (cnt_v > 16)
            {
                current_state_vals.bat_voltage = (int)((float)sum_v / cnt_v / 0.341f);
                ESP_LOGI(TAG, "Bat : %d mV (%d)", current_state_vals.bat_voltage, current_state_vals.bat_voltage / 2);
                sum_v = 0;
                cnt_v = 0;
            }
        }
#endif
        //=====================================================
#ifdef GPIO_ADD_LED
        vTaskDelay(1500 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_ADD_LED, 1);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_ADD_LED, 0);
#else
        vTaskDelay(1500 / portTICK_PERIOD_MS);
#endif
        //=====================================================
        if (esp_timer_get_time() > REBOOT_PERIOD)
        {
            ESP_LOGI(TAG, "restart by timer. period %ld", REBOOT_PERIOD); // if unpedictable programm deadlock and so on.
            esp_restart();
        }
        if (do_save_config)
        {
            do_save_config = false;
            save_configs_to_nvs();
        }
        /*
                if (gyroscope_state.ready_for_check)
                {
                    if (gyroscope_state.max_dt_between_frame > config_guard.treshold_low_frq && gyroscope_state.max_dt_in_frame > config_guard.treshold_hight_frq)
                    {
                        ESP_LOGE(TAG, "!!!!!!!!!! %d/%d/%d", gyroscope_state.max_dt_between_frame, gyroscope_state.max_dt_in_frame, gyroscope_state.ready_for_check);
                    } else {
                        ESP_LOGI(TAG, "-------- %d/%d/%d", gyroscope_state.max_dt_between_frame, gyroscope_state.max_dt_in_frame, gyroscope_state.ready_for_check);
                    }
                }
        */
    }
}
