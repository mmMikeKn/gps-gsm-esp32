#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "sms_cmd.h"

static const char *TAG = "sms_cmd";

#define GPS_HREF_FMT "http://maps.google.com/maps?q=%.5f,%.5f\n%02d.%02d.%02d %02d:%02d:%02d(%s)\n%dkm/h "

static void fill_href_in_sms_body(char *str, gps_crd_t *gps, int max_sz)
{
    struct tm tm = {0};
    tm.tm_year = gps->date.year + 100;
    tm.tm_mon = gps->date.month;
    tm.tm_mday = gps->date.day;
    tm.tm_hour = gps->tim.hour;
    tm.tm_min = gps->tim.minute;
    tm.tm_sec = gps->tim.second;
    setenv("TZ", "UTC", 1);
    tzset();
    time_t t = mktime(&tm);
    setenv("TZ", config_guard.timezone, 1);
    struct tm *out_tm = localtime(&t);

    snprintf(str, max_sz, GPS_HREF_FMT "sat:%d "
#ifdef VOLTAGE_BAT_ADC2_CHANNEL
                                       "%0.2f/%0.2fV"
#else
                                       "%0.2fV"
#endif
                                       " ACC:%d G:%d/%d/%d %s:%d",
             gps->latitude, gps->longitude,
             // gps->date.day, gps->date.month, gps->date.year,             gps->tim.hour, gps->tim.minute, gps->tim.second,
             out_tm->tm_mday, out_tm->tm_mon, out_tm->tm_year - 100, out_tm->tm_hour, out_tm->tm_min, out_tm->tm_sec,
             config_guard.timezone,
             (int)(gps->speed * 60 / 1000),
             gps_state.sats_in_view,
             (float)gsm_state.voltage / 1000.0f,
#ifdef VOLTAGE_BAT_ADC2_CHANNEL
             (float)current_state_vals.bat_voltage / 1000.0f,
#endif
             is_acc_on(),
             gyroscope_state.max_dt_between_frame, gyroscope_state.max_dt_in_frame, gyroscope_state.ready_for_check,
             gsm_state.provider, gsm_state.signal_level);
}

char *fill_current_state_in_sms(char *str, int max_sz)
{
    gps_crd_t *gps = &gps_state.cur_crd;
    if (gps_state.valid)
    {
        fill_href_in_sms_body(str, gps, max_sz);
    }
    else
    {
        int lastNum = gps_state.last_valid_crd_num;
        if (lastNum == 0)
        {
            strcpy(str, "no crd&history ");
            int n = strlen(str);
            fill_href_in_sms_body(str + n, gps, max_sz - n);
        }
        else
        {
            strcpy(str, "last crd: ");
            int n = strlen(str);

            fill_href_in_sms_body(str + n, &gps_state.last_valid_crd[lastNum - 1], max_sz - n);
        }
    }
    return str;
}

void send_echo_sms(char *phone)
{
    strncpy(gsm_state.sms_phone_out, phone, sizeof(gsm_state.sms_phone_out) - 1);
    ESP_LOGI(TAG, "send echo (%s) sms: %s", gsm_state.sms_phone_out, gsm_state.sms_body_out);
    set_send_sms_flag();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "send echo. end of vTaskDelay");
}

static char *sms_last_crd_list(char *str, int max_sz)
{
    int lastNum = gps_state.last_valid_crd_num;
    if (lastNum == 0)
    {
        gps_crd_t *gps = &gps_state.cur_crd;
        snprintf(str, max_sz, "No crd. sat:%d. %02d.%02d.%02d %02d:%02d:%02d",
                 gps_state.sats_in_view,
                 gps->date.day, gps->date.month, gps->date.year,
                 gps->tim.hour, gps->tim.minute, gps->tim.second);
    }
    else
    {
        str[0] = 0;
        for (int i = 0; i < lastNum; i++)
        {
            int n = strlen(str);
            if (i != 0 && n < max_sz)
            {
                str[n++] = '\n';
                str[n] = 0;
            }
            snprintf(str + n, max_sz - n, "[%d]: ", i);
            n = strlen(str);
            gps_crd_t *gps = &gps_state.last_valid_crd[lastNum - 1];
            snprintf(str + n, max_sz - n, GPS_HREF_FMT,
                     gps->latitude, gps->longitude,
                     gps->date.day, gps->date.month, gps->date.year,
                     gps->tim.hour, gps->tim.minute, gps->tim.second, "UTC",
                     (int)(gps->speed * 60 / 1000));
        }
    }
    return str;
}

static bool parse_sms_arg(int n, uint16_t *val)
{
    char tmp[6];
    size_t p = 0, cnt = -1;
    bool is_last_not_digit = true;
    for (size_t i = 0; gsm_state.sms_body_in[i] != 0 && i < sizeof(gsm_state.sms_body_in); i++)
    {
        char c = gsm_state.sms_body_in[i];
        if (c >= '0' && c <= '9')
        {
            if (is_last_not_digit)
            {
                cnt++;
            }
            if (cnt == n && p < sizeof(tmp))
            {
                tmp[p++] = c;
            }
            is_last_not_digit = false;
        }
        else
        {
            is_last_not_digit = true;
            if (cnt == n)
            {
                break;
            }
        }
    }
    if (p > 0)
    {
        tmp[p] = 0;
        *val = atoi(tmp);
        ESP_LOGI(TAG, "Parsed val=%d n:%d, from:'%s'/'%s'", *val, n, tmp, gsm_state.sms_body_in);
        return true;
    }
    return false;
}

static void json_to_sms_msg(char *json)
{
    for (int i = 0; json[i] != 0; i++)
    {
        if (json[i] == '\t')
        {
            json[i] = ' ';
        }
    }
}

static void add_config_in_sms()
{
    char *json = config_to_json();
    json_to_sms_msg(json);
    strcpy(gsm_state.sms_body_out, gsm_state.sms_body_in);
    strcat(gsm_state.sms_body_out, ":");
    int l = strlen(gsm_state.sms_body_out);
    strncpy(gsm_state.sms_body_out + l, json, sizeof(gsm_state.sms_body_out) - 1 - l);
    free(json);
}

static const char *help_str = "I[nfo]\n"
                              "S[tate]\n"
                              "T[rack]\n"
                              "C[onf]\n"
                              "G[yro]<n>,<n>\n"
                              "V[oltage]<n>,<n>\n"
                              "A[larm]<0/1>\n"
                              "B[le]\n"
                              "E[nableBle] <hex1,hex2,..>\n"
                              "H[ang up]0,1\n"
                              "R[oute]0,1\n"
                              "Reboot";

void handle_cmd_sms()
{
    gsm_state.has_sms = false;
    if (is_phone_not_permited(gsm_state.sms_phone_in))
    {
        if (config_guard.alarm_phones_num > 0 && config_guard.do_route_unknown_sms)
        {
            snprintf(gsm_state.sms_body_out, sizeof(gsm_state.sms_body_out) - 1, "route [%s]: ", gsm_state.sms_phone_in);
            int l = strlen(gsm_state.sms_body_out); // TODO if long sms..
            strncat(gsm_state.sms_body_out, gsm_state.sms_body_in, sizeof(gsm_state.sms_body_out) - 1 - l);
            send_echo_sms(config_guard.alarm_phones[0]);
        }
        return;
    }
    uint16_t val;
    switch (tolower(gsm_state.sms_body_in[0]))
    {
    case 'r': // r[eboot]
        if (strcmp(gsm_state.sms_body_in + 1, "eboot") == 0)
        {
            strcpy(gsm_state.sms_body_out, "reboot");
            send_echo_sms(gsm_state.sms_phone_in);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            esp_restart();
            return;
        }
        else if (parse_sms_arg(0, &val))
        {
            config_guard.do_route_unknown_sms = val != 0;
            do_save_config = true;
            add_config_in_sms();
        }
        break;
    case 'a': // Alarm sms on|off
        if (parse_sms_arg(0, &val))
        {
            config_guard.stop_alrm_sms_send = val == 0;
            do_save_config = true;
            add_config_in_sms();
        }
        break;
    case 'g': // gyroscope alarm levels
        if (!parse_sms_arg(0, &config_guard.treshold_low_frq) || !parse_sms_arg(1, &config_guard.treshold_hight_frq))
        {
            strcpy(gsm_state.sms_body_out, "Wrong attributes in gyroscope alarm levels sms\n");
            strcat(gsm_state.sms_body_out, help_str);
        }
        else
        {
            do_save_config = true;
            add_config_in_sms();
        }
        break;
    case 'v': // voltage alarm levels
        if (!parse_sms_arg(0, &config_guard.treshold_low_volt) || !parse_sms_arg(1, &config_guard.treshold_low_volt_step))
        {
            strcpy(gsm_state.sms_body_out, "Wrong attributes in voltage alarm levels sms\n");
            strcat(gsm_state.sms_body_out, help_str);
        }
        else
        {
            do_save_config = true;
            add_config_in_sms();
        }
        break;
    case 'c': // C[onf]
        sprintf(gsm_state.sms_body_out, "vt:%d mV ", last_voltage_treshold);
        add_config_in_sms();
        break;
    case 's': // S[ate]
        char *json = json_state();
        json_to_sms_msg(json);
        snprintf(gsm_state.sms_body_out, sizeof(gsm_state.sms_body_out) - 1, "state: %s", json);
        free(json);
        break;
    case 'i': // I[nfo]
        fill_current_state_in_sms(gsm_state.sms_body_out, sizeof(gsm_state.sms_body_out) - 1);
        break;
    case 't': // T[rack]
        sms_last_crd_list(gsm_state.sms_body_out, sizeof(gsm_state.sms_body_out) - 1);
        break;
    case 'b': // B[le]
        char *s = ble_state();
        strncpy(gsm_state.sms_body_out, s, sizeof(gsm_state.sms_body_out) - 1);
        free(s);
        if (current_state_vals.ble_scan_finish_time < esp_timer_get_time())
        {
            current_state_vals.ble_scan_finish_time = esp_timer_get_time() + (BLE_SCAN_TIME_SEC + 1) * 1000L * 1000L; // in microseconds since boot
            do_ble_scan();
        }
        else
        {
            ESP_LOGI(TAG, "continue ble scan");
        }
        break;
    case 'h':
        if (parse_sms_arg(0, &val))
        {
            config_guard.do_hang_up = val == 1;
            do_save_config = true;
            add_config_in_sms();
        }
        else
        {
            strcpy(gsm_state.sms_body_out, "Wrong attributes (config_guard.do_hang_up) in the sms\n");
            strcat(gsm_state.sms_body_out, help_str);
        }
        break;
    case 'e':
        char *ptr_arg = strchr(gsm_state.sms_body_in, ' ');
        if (ptr_arg != NULL)
        {
            strncpy(config_guard.ble_enabled, ptr_arg + 1, sizeof(config_guard.ble_enabled) - 1);
            do_save_config = true;
            add_config_in_sms();
        }
        else
        {
            strcpy(gsm_state.sms_body_out, "Wrong attributes (config_guard.ble_enabled) in the sms\n");
            strcat(gsm_state.sms_body_out, help_str);
        }
        break;
    default:
        strcpy(gsm_state.sms_body_out, help_str);
    }

    send_echo_sms(gsm_state.sms_phone_in);
}
