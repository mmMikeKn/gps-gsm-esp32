#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include <esp_adc/adc_oneshot.h>

#include "gyro.h"

gyroscope_state_t gyroscope_state;

#define GYRO_CHANELS_NUM (3)
#define GYRO_FRAME_NUM (256)
#define GYRO_FRAME_SIZE (512)

const static DRAM_ATTR adc_channel_t channel[GYRO_CHANELS_NUM] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_4}; // X,Y,Z

static DRAM_ATTR adc_continuous_handle_t adc_driver_handle;

typedef struct
{
    uint16_t over_val_in_frame, max_dt;
    uint64_t sum_in_frame, num_in_frame;
} gyro_chanel_data_t;

typedef struct
{
    gyro_chanel_data_t gyro_chanel_data[GYRO_FRAME_NUM]; // cycle bufffer
} adc_data_t;

static DRAM_ATTR adc_data_t adc_data[GYRO_CHANELS_NUM];

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    static DRAM_ATTR int cur_frame = 0;
    static DRAM_ATTR bool is_all_frames_filled = false;
    
    cur_frame++;
    if (cur_frame >= GYRO_FRAME_NUM)
    {
        cur_frame = 0;
        is_all_frames_filled = true;
    }

    for (int i = 0; i < GYRO_CHANELS_NUM; i++)
    { // prepare struct for filling
        adc_data_t *a = &adc_data[i];
        a->gyro_chanel_data[cur_frame].sum_in_frame = 0;
        a->gyro_chanel_data[cur_frame].num_in_frame = a->gyro_chanel_data[cur_frame].max_dt = a->gyro_chanel_data[cur_frame].over_val_in_frame = 0;
    }

    //======================================
    for (int i = 0; i < edata->size; i += SOC_ADC_DIGI_RESULT_BYTES)
    {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&edata->conv_frame_buffer[i];
        uint16_t ch = p->type1.channel;
        for (int n = 0; n < GYRO_CHANELS_NUM; n++)
        {
            if (channel[n] == ch)
            {
                adc_data_t *a = &adc_data[n];
                a->gyro_chanel_data[cur_frame].sum_in_frame += p->type1.data;
                a->gyro_chanel_data[cur_frame].num_in_frame++;
                break;
            }
        }
    }

    for (int i = 0; i < GYRO_CHANELS_NUM; i++)
    {
        adc_data_t *a = &adc_data[i];
        a->gyro_chanel_data[cur_frame].over_val_in_frame = a->gyro_chanel_data[cur_frame].sum_in_frame / a->gyro_chanel_data[cur_frame].num_in_frame;
    }

    for (int i = 0; i < edata->size; i += SOC_ADC_DIGI_RESULT_BYTES)
    {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&edata->conv_frame_buffer[i];
        uint16_t ch = p->type1.channel;
        for (int n = 0; n < GYRO_CHANELS_NUM; n++)
        {
            if (channel[n] == ch)
            {
                adc_data_t *a = &adc_data[n];
                uint16_t over = a->gyro_chanel_data[cur_frame].over_val_in_frame;
                uint16_t dt = (over > p->type1.data) ? over - p->type1.data : p->type1.data - over;
                if (dt > a->gyro_chanel_data[cur_frame].max_dt)
                {
                    a->gyro_chanel_data[cur_frame].max_dt = dt;
                }
                break;
            }
        }
    }

    if (is_all_frames_filled)
    {
        gyroscope_state.max_dt_between_frame = gyroscope_state.max_dt_in_frame = 0;
        for (int i = 0; i < GYRO_CHANELS_NUM; i++)
        {
            adc_data_t *a = &adc_data[i];
            uint64_t sum = 0;
            for (int j = 0; j < GYRO_FRAME_NUM; j++)
            {
                sum += a->gyro_chanel_data[j].over_val_in_frame;
                if (a->gyro_chanel_data[j].max_dt > gyroscope_state.max_dt_in_frame)
                {
                    gyroscope_state.max_dt_in_frame = a->gyro_chanel_data[j].max_dt;
                }
            }
            uint16_t over_val_in_frames = sum / GYRO_FRAME_NUM;
            for (int j = 0; j < GYRO_FRAME_NUM; j++)
            {
                uint16_t dt = over_val_in_frames > a->gyro_chanel_data[j].over_val_in_frame ? over_val_in_frames - a->gyro_chanel_data[j].over_val_in_frame : a->gyro_chanel_data[j].over_val_in_frame - over_val_in_frames;
                if (dt > gyroscope_state.max_dt_between_frame)
                {
                    gyroscope_state.max_dt_between_frame = dt;
                }
            }
        }
        gyroscope_state.ready_for_check = true;
        is_all_frames_filled = false;
    }

    return true;
}

/*
adc_oneshot_unit_handle_t adc1_handle;
static void gyro_task2(void *arg)
{
    while (true)
    {
        vTaskDelay(1500 / portTICK_PERIOD_MS);
        int adc_raw[3];
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw[0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_raw[1]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[2]));
        ESP_LOGI(TAG, "val: %x %x %x", adc_raw[0], adc_raw[1], adc_raw[2]);
    }
}

void startGyro2(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    ESP_LOGI(TAG, "gyro_task start");
    xTaskCreate(gyro_task2, "gyro_task2", 2048 * 2, NULL, 11, NULL);
}
*/

void start_gyro(void)
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = GYRO_FRAME_SIZE * 2,
        .conv_frame_size = GYRO_FRAME_SIZE,
        .flags.flush_pool = 1,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_driver_handle));

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    adc_continuous_config_t dig_cfg = {
        .pattern_num = GYRO_CHANELS_NUM,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = 80000,                // SOC_ADC_SAMPLE_FREQ_THRES_LOW           (20*1000)
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,    // ADC_CONV_SINGLE_UNIT_1 = 1,  ///< Only use ADC1 for conversion
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, // < See `adc_digi_output_data_t.type1`
    };

    for (int i = 0; i < GYRO_CHANELS_NUM; i++)
    {
        adc_pattern[i].atten = ADC_ATTEN_DB_12; // No input attenuation, ADC can measure up to approx.
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    ESP_ERROR_CHECK(adc_continuous_config(adc_driver_handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    memset(&adc_data, 0, sizeof(adc_data));

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_driver_handle, &cbs, NULL));

    ESP_ERROR_CHECK(adc_continuous_start(adc_driver_handle));
    ESP_LOGI("GYRO BOARD", "ADC gyroscope started");
}
