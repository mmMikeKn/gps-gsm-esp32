#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "sim800l.h"

gsm_state_t gsm_state;

static const char *TAG = "sim800l";

bool _do_send_sms = false;
int _sms_body_out_ofs = 0;
static bool _recvd_ok_str = false;

#define SMS_STEP_IDLE (0)
#define SMS_STEP_RCV_SMS_BODY (1)

static int _rcv_sms_step = SMS_STEP_IDLE;
static QueueHandle_t uart_queue;

#define MAX_STR_SIZE (512)

static bool starts_with(const char *prefix, const char *str)
{
    size_t l_prefix = strlen(prefix), l_str = strlen(str);
    return l_str < l_prefix ? false : memcmp(prefix, str, l_prefix) == 0;
}

static bool parse_answer_payload(char *str, int ppos, char *out, int max_out_sz)
{
    out[0] = 0;
    char *p = strstr(str, ": ");
    if (p == NULL)
    {
        ESP_LOGI(TAG, "No ': ' in the '%s'", str);
        return false;
    }

    max_out_sz--;
    int curPpos = 0, outIndx = 0;
    bool is_in_quotes_char = false;
    for (p += 2; *p != 0 && curPpos <= ppos; p++)
    {
        if (*p == '"')
        {
            is_in_quotes_char = !is_in_quotes_char;
            continue; // ignore '"'
        }
        if (*p == ',' && !is_in_quotes_char)
        {
            curPpos++;
            continue;
        }
        if (curPpos == ppos)
        {
            if (outIndx >= max_out_sz)
            {
                ESP_LOGI(TAG, "too long value (%d) in the '%s' for ppos=%d", max_out_sz, str, ppos);
                return false;
            }
            out[outIndx++] = *p;
        }
    }
    if (outIndx == 0)
    {
        ESP_LOGI(TAG, "No ppos=%d in the '%s'", ppos, str);
        return false;
    }
    out[outIndx] = 0;
    return true;
}

static bool parse_answer_payload_int(char *str, int ppos, int *out_val)
{
    char tmp[8];
    if (parse_answer_payload(str, ppos, tmp, sizeof(tmp)))
    {
        *out_val = atoi(tmp);
        return true;
    }
    return false;
}

static void parse_answer(char *answer)
{
    if (strstr("OK", answer) != NULL)
    {
        ///        ESP_LOGI(TAG, "rcv 'OK'");
        _recvd_ok_str = true;
        return;
    }
    if (starts_with(gsm_state.last_at_cmd, answer))
    {
        //        ESP_LOGI(TAG, "rcv cmd echo");
        return;
    }
    //    ESP_LOGI(TAG, "gsm_state.last_at_cmd: '%s'", gsm_state.last_at_cmd);
    ESP_LOGI(TAG, "<=='%s'", answer);
    //==========
    if (_rcv_sms_step == SMS_STEP_RCV_SMS_BODY)
    {
        strncpy(gsm_state.sms_body_in, answer, sizeof(gsm_state.sms_body_in) - 1);
        _rcv_sms_step = SMS_STEP_IDLE;
        gsm_state.has_sms = true;
        ESP_LOGI(TAG, "gsm_state.sms_body_in: '%s'", gsm_state.sms_body_in);
    }
    else if (starts_with("+COPS:", answer))
    { // provider
        if (parse_answer_payload(answer, 2, gsm_state.provider, sizeof(gsm_state.provider)))
        {
            ESP_LOGI(TAG, "gsm_state.provider: '%s'", gsm_state.provider);
        }
    }
    else if (starts_with("+CBC:", answer))
    {
        if (parse_answer_payload_int(answer, 2, &gsm_state.voltage))
        {
            ESP_LOGI(TAG, "gsm_state.voltage: '%d'", gsm_state.voltage);
        }
    }
    else if (starts_with("+CSQ:", answer))
    {
        if (parse_answer_payload_int(answer, 0, &gsm_state.signal_level))
        {
            ESP_LOGI(TAG, "gsm_state.signal_level: '%d'", gsm_state.signal_level);
        }
    }
    else if (starts_with("+CSMINS:", answer))
    {
        if (parse_answer_payload_int(answer, 1, &gsm_state.sim_in_slot))
        {
            ESP_LOGI(TAG, "gsm_state.sim_in_slot: '%d'", gsm_state.sim_in_slot);
        }
    }
    else if (starts_with("+CLIP:", answer))
    {
        if (parse_answer_payload(answer, 0, gsm_state.ring_phone, sizeof(gsm_state.ring_phone)))
        {
            ESP_LOGI(TAG, "gsm_state.ring_phone: '%s'", gsm_state.ring_phone);
        }
    }
    else if (starts_with("RING", answer))
    {
        if (!gsm_state.do_hang_up && !gsm_state.do_answer_incoming_call)
        {
            gsm_state.is_ring = true;
            ESP_LOGI(TAG, "gsm_state.is_ring: '%d'", gsm_state.is_ring);
        }
        else
        {
            ESP_LOGI(TAG, "Ignode ring. ring handled %d/%d", gsm_state.do_hang_up, gsm_state.do_answer_incoming_call);
        }
    }
    else if (starts_with("+CMT:", answer))
    {
        if (parse_answer_payload(answer, 0, gsm_state.sms_phone_in, sizeof(gsm_state.sms_phone_in)))
        {
            ESP_LOGI(TAG, "gsm_state.sms_phone_in: '%s'", gsm_state.sms_phone_in);
        }
        _rcv_sms_step = SMS_STEP_RCV_SMS_BODY;
    }
    else if (starts_with("+CMGL:", answer))
    {
        ESP_LOGI(TAG, "SMS Messages from Preferred Store: '%s'", answer);
    }
}

static void rcv_task(void *arg)
{
    uart_event_t event;
    uint8_t *str = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    /*
        while (true)
        {
            const int rxBytes = uart_read_bytes(SIM800L_UART_NUM, str, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
            if (rxBytes > 0) {
                str[rxBytes] = 0;
                ESP_LOGI(TAG, "Read %d bytes", rxBytes);
                ESP_LOG_BUFFER_HEXDUMP(TAG, str, rxBytes, ESP_LOG_INFO);
            }
        }
    */
    while (true)
    {
        if (xQueueReceive(uart_queue, &event, pdMS_TO_TICKS(50)))
        {
            switch (event.type)
            {
            case UART_DATA:
                /*
                                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                                int len = uart_read_bytes(SIM800L_UART_NUM, str, event.size, portMAX_DELAY);
                                ESP_LOG_BUFFER_HEXDUMP(TAG, str, len, ESP_LOG_INFO);
                */
                break;
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "HW FIFO Overflow");
                uart_flush(SIM800L_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "Ring Buffer Full");
                uart_flush(SIM800L_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_BREAK:
                ESP_LOGW(TAG, "Rx Break");
                uart_flush(SIM800L_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_PARITY_ERR:
                ESP_LOGE(TAG, "Parity Error");
                uart_flush(SIM800L_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_FRAME_ERR:
                ESP_LOGE(TAG, "Frame Error");
                uart_flush(SIM800L_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_PATTERN_DET:
            {
                size_t buffered_size;
                uart_get_buffered_data_len(SIM800L_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(SIM800L_UART_NUM);
                //                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos < 0)
                {
                    uart_flush_input(SIM800L_UART_NUM);
                }
                else
                {
                    int len = uart_read_bytes(SIM800L_UART_NUM, str, pos, 100 / portTICK_PERIOD_MS);
                    str[len] = 0;
                    //      ESP_LOG_BUFFER_HEXDUMP(TAG, str, len, ESP_LOG_INFO);
                    while (str[0] == '\r' || str[0] == '\n')
                    {
                        for (int i = 0; i <= len; i++)
                        {
                            str[i] = str[i + 1];
                        }
                        len--;
                    }
                    while (len > 0 && (str[len - 1] == '\r' || str[len - 1] == '\n'))
                    {
                        str[--len] = 0;
                    }
                    if (len > 0)
                    {
                        parse_answer((char *)str);
                    }
                }
            }
            break;
            default:
                ESP_LOGW(TAG, "unknown uart event type: %d", event.type);
                break;
            }
        }
    }
}

// static SemaphoreHandle_t _cmd_send_mutex;

static void sendCmd(const char *cmd)
{
    //    xSemaphoreTake(_cmd_send_mutex, portMAX_DELAY);
    int cmdLen = strlen(cmd);
    if (cmdLen > (sizeof(gsm_state.last_at_cmd) - 1))
    {
        cmdLen = sizeof(gsm_state.last_at_cmd) - 1;
    }
    strncpy(gsm_state.last_at_cmd, cmd, cmdLen);
    gsm_state.last_at_cmd[cmdLen] = 0;
    gsm_state.is_ok_last_at_cmd = false;

    _recvd_ok_str = false;

#ifdef SIM800L_DTR_GPIO
    gpio_set_level(SIM800L_DTR_GPIO, 0);  // wake up
    vTaskDelay(150 / portTICK_PERIOD_MS); //
#endif

    uart_write_bytes(SIM800L_UART_NUM, cmd, cmdLen);
    uart_write_bytes(SIM800L_UART_NUM, "\n", 1);
    ESP_LOGI(TAG, "==> '%s'", cmd);
    // ESP_LOG_BUFFER_HEXDUMP(TAG, cmd, cmdLen, ESP_LOG_INFO);
    for (int i = 0; i < 10; i++)
    {
        if (_recvd_ok_str)
        {
            gsm_state.is_ok_last_at_cmd = true;
            break;
        }
        else
        {
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
    }
#ifdef SIM800L_DTR_GPIO
    gpio_set_level(SIM800L_DTR_GPIO, 1);
#endif
    //    xSemaphoreGive(_cmd_send_mutex);
}

static void sim800l_task(void *arg)
{
    char *cmd = (char *)malloc(20 + sizeof(gsm_state.sms_phone_out) + sizeof(gsm_state.sms_body_out));
    while (true)
    {
        sendCmd("AT+CBC");
        sendCmd("AT+CSQ");
        sendCmd("AT+COPS?");
        for (int i = 0; i < 240; i++) // 500*(2*120)
        {
            // ESP_LOGI(TAG, "sim800l_task cycle: %d %d", _rcv_sms_step, gsm_state.do_send_sms);
            if (_do_send_sms)
            {
                int l = strlen(gsm_state.sms_body_out + _sms_body_out_ofs);
                char c = 0;
                if (l > MAX_SMS_BODY_SZ)
                {
                    c = gsm_state.sms_body_out[_sms_body_out_ofs + MAX_SMS_BODY_SZ];
                    gsm_state.sms_body_out[_sms_body_out_ofs + MAX_SMS_BODY_SZ] = 0;
                }
                sprintf(cmd, "AT+CMGS=\"%s\"\n\%s\x1A", gsm_state.sms_phone_out, gsm_state.sms_body_out + _sms_body_out_ofs);
                // ESP_LOGI(TAG, "going to send sms command:\n%s", cmd);
                sendCmd(cmd);
                if (l > MAX_SMS_BODY_SZ)
                {
                    gsm_state.sms_body_out[_sms_body_out_ofs + MAX_SMS_BODY_SZ] = c;
                    _sms_body_out_ofs += MAX_SMS_BODY_SZ;
                    ESP_LOGI(TAG, "delay for send part of the full messge");
                    vTaskDelay(3000 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "end of delay.");
                }
                else
                {
                    _do_send_sms = false;
                    gsm_state.sms_body_out[0] = 0;
                    _sms_body_out_ofs = 0;
                    ESP_LOGI(TAG, "delay for send messge");
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "end of delay.");
                }
            }
            else if (gsm_state.do_hang_up || gsm_state.do_answer_incoming_call)
            {
                gsm_state.is_ring = false;
                if (gsm_state.do_answer_incoming_call)
                {
                    gsm_state.do_answer_incoming_call = false;
                    sendCmd("ATA");
                }
                else
                {
                    gsm_state.do_hang_up = false;
                    sendCmd("ATH");
                }
            }

            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

void set_send_sms_flag()
{
    _do_send_sms = true;
    _sms_body_out_ofs = 0;
}

bool is_set_send_sms_flag()
{
    return _do_send_sms;
}

void start_sim800l()
{
#ifdef SIM800L_DTR_GPIO
    static const gpio_config_t gpio_DTR_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SIM800L_DTR_GPIO)};
#endif

    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    ESP_LOGI(TAG, "going to call uart_driver_install");
    ESP_ERROR_CHECK(uart_driver_install(SIM800L_UART_NUM, RX_BUF_SIZE, TX_BUF_SIZE, 20, &uart_queue, 0));
    //    ESP_ERROR_CHECK(uart_driver_install(SIM800L_UART_NUM, RX_BUF_SIZE, TX_BUF_SIZE, 50, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SIM800L_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SIM800L_UART_NUM, SIM800L_TXD, SIM800L_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(SIM800L_UART_NUM, '\n', 1, 5, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(SIM800L_UART_NUM, 50));
    uart_flush(SIM800L_UART_NUM);
#ifdef SIM800L_DTR_GPIO
    gpio_config(&gpio_DTR_cfg);
#endif
    //    _cmd_send_mutex = xSemaphoreCreateMutex();
    memset(&gsm_state, 0, sizeof(gsm_state));
    xTaskCreate(rcv_task, "sim800l_rcv", 4096, NULL, 11, NULL);
    //--------------
    sendCmd("AT");
    sendCmd("AT");
    sendCmd("AT&F");
    sendCmd("AT+CMGDA=6");        // delete all sms from storage
    sendCmd("AT+CSMINS?");        // SIM Inserted Status Reporting
    sendCmd("AT+CLIP=1");         // Calling Line Identification Presentation. 1 Enable +CLIP notification.
    sendCmd("AT+CMGF=1");         // Select SMS Message Format. 1 Text mode
    sendCmd("AT+CNMI=1,2,0,0,0"); // New SMS Message Indications. SMS-DELIVERs (except class 2) are routed directly to  the TE using unsolicited result code: +CMT

#ifdef SIM800L_DTR_GPIO
    gpio_set_level(SIM800L_DTR_GPIO, 1);
    sendCmd("AT+CSCLK=1"); // sleep mode
#endif

    xTaskCreate(sim800l_task, "sim800l_loop", 4096, NULL, 11, NULL);
    ESP_LOGI(TAG, "SIM800L started");
}
