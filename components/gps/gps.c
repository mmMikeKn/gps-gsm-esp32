#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "gps.h"

static const char *TAG = "neo-6m";
static const char *GPS_TAG = "gps-parser";
static QueueHandle_t uart_queue;

//========= IDF parser example ========
#define NMEA_MAX_STATEMENT_ITEM_LENGTH (16)

gps_state_t gps_state = {0};

static long to_sec_simple(gps_crd_t *t)
{
    return ((((t->date.year * 365 + t->date.month) * 30 + t->date.day) * 24 + t->tim.hour) * 60 + t->tim.minute) * 60 + t->tim.second;
}

static void save_last_valid_crd()
{
    if (gps_state.valid)
    {
        if (gps_state.last_valid_crd_num == 0 ||
            to_sec_simple(&gps_state.last_valid_crd[gps_state.last_valid_crd_num - 1]) - to_sec_simple(&gps_state.cur_crd) > 60 * 5L)
        {
            if (gps_state.last_valid_crd_num >= GPS_LAST_VALID_MAX)
            {
                memcpy(gps_state.last_valid_crd, &gps_state.last_valid_crd[1], (GPS_LAST_VALID_MAX - 1) * sizeof(gps_crd_t));
                gps_state.last_valid_crd_num--;
            }
            memcpy(&gps_state.last_valid_crd[gps_state.last_valid_crd_num], &gps_state.cur_crd, sizeof(gps_crd_t));
            gps_state.last_valid_crd_num++;
        }
    }
}

/**
 * @brief GPS parser library runtime structure
 *
 */
typedef struct
{
    uint8_t item_pos;                                /*!< Current position in item */
    uint8_t item_num;                                /*!< Current item number */
    uint8_t asterisk;                                /*!< Asterisk detected flag */
    uint8_t crc;                                     /*!< Calculated CRC value */
    uint8_t parsed_statement;                        /*!< OR'd of statements that have been parsed */
    uint8_t sat_num;                                 /*!< Satellite number */
    uint8_t sat_count;                               /*!< Satellite count */
    uint8_t cur_statement;                           /*!< Current statement ID */
    uint32_t all_statements;                         /*!< All statements mask */
    char item_str[NMEA_MAX_STATEMENT_ITEM_LENGTH];   /*!< Current item */
    uint8_t buffer[NMEA_PARSER_RUNTIME_BUFFER_SIZE]; /*!< Runtime buffer */
} esp_gps_t;

/**
 * @brief parse latitude or longitude
 *              format of latitude in NMEA is ddmm.sss and longitude is dddmm.sss
 * @param esp_gps esp_gps_t type object
 * @return float Latitude or Longitude value (unit: degree)
 */
static float parse_lat_long(esp_gps_t *esp_gps)
{
    float ll = strtof(esp_gps->item_str, NULL);
    int deg = ((int)ll) / 100;
    float min = ll - (deg * 100);
    ll = deg + min / 60.0f;
    return ll;
}

/**
 * @brief Converter two continuous numeric character into a uint8_t number
 *
 * @param digit_char numeric character
 * @return uint8_t result of converting
 */
static inline uint8_t convert_two_digit2number(const char *digit_char)
{
    return 10 * (digit_char[0] - '0') + (digit_char[1] - '0');
}

/**
 * @brief Parse UTC time in GPS statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_utc_time(esp_gps_t *esp_gps)
{
    gps_state.cur_crd.tim.hour = convert_two_digit2number(esp_gps->item_str + 0);
    gps_state.cur_crd.tim.minute = convert_two_digit2number(esp_gps->item_str + 2);
    gps_state.cur_crd.tim.second = convert_two_digit2number(esp_gps->item_str + 4);
    if (esp_gps->item_str[6] == '.')
    {
        uint16_t tmp = 0;
        uint8_t i = 7;
        while (esp_gps->item_str[i])
        {
            tmp = 10 * tmp + esp_gps->item_str[i] - '0';
            i++;
        }
        gps_state.cur_crd.tim.thousand = tmp;
    }
}

#if CONFIG_NMEA_STATEMENT_GGA
/**
 * @brief Parse GGA statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gga(esp_gps_t *esp_gps)
{
    /* Process GGA statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Process UTC time */
        parse_utc_time(esp_gps);
        break;
    case 2: /* Latitude */
        gps_state.cur_crd.latitude = parse_lat_long(esp_gps);
        break;
    case 3: /* Latitude north(1)/south(-1) information */
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
        {
            gps_state.cur_crd.latitude *= -1;
        }
        break;
    case 4: /* Longitude */
        gps_state.cur_crd.longitude = parse_lat_long(esp_gps);
        break;
    case 5: /* Longitude east(1)/west(-1) information */
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
        {
            gps_state.cur_crd.longitude *= -1;
        }
        break;
    case 6: /* Fix status */
        gps_state.fix = (gps_fix_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 7: /* Satellites in use */
        gps_state.sats_in_use = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 8: /* HDOP */
        gps_state.dop_h = strtof(esp_gps->item_str, NULL);
        break;
    case 9: /* Altitude */
        gps_state.cur_crd.altitude = strtof(esp_gps->item_str, NULL);
        break;
    case 11: /* Altitude above ellipsoid */
        gps_state.cur_crd.altitude += strtof(esp_gps->item_str, NULL);
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_GSA
/**
 * @brief Parse GSA statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gsa(esp_gps_t *esp_gps)
{
    /* Process GSA statement */
    switch (esp_gps->item_num)
    {
    case 2: /* Process fix mode */
        gps_state.fix_mode = (gps_fix_mode_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 15: /* Process PDOP */
        gps_state.dop_p = strtof(esp_gps->item_str, NULL);
        break;
    case 16: /* Process HDOP */
        gps_state.dop_h = strtof(esp_gps->item_str, NULL);
        break;
    case 17: /* Process VDOP */
        gps_state.dop_v = strtof(esp_gps->item_str, NULL);
        break;
    default:
        /* Parse satellite IDs */
        if (esp_gps->item_num >= 3 && esp_gps->item_num <= 14)
        {
            gps_state.sats_id_in_use[esp_gps->item_num - 3] = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        }
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_GSV
/**
 * @brief Parse GSV statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gsv(esp_gps_t *esp_gps)
{
    /* Process GSV statement */
    switch (esp_gps->item_num)
    {
    case 1: /* total GSV numbers */
        esp_gps->sat_count = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 2: /* Current GSV statement number */
        esp_gps->sat_num = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    case 3: /* Process satellites in view */
        gps_state.sats_in_view = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
        break;
    default:
        if (esp_gps->item_num >= 4 && esp_gps->item_num <= 19)
        {
            uint8_t item_num = esp_gps->item_num - 4; /* Normalize item number from 4-19 to 0-15 */
            uint8_t index;
            uint32_t value;
            index = 4 * (esp_gps->sat_num - 1) + item_num / 4; /* Get array index */
            if (index < GPS_MAX_SATELLITES_IN_VIEW)
            {
                value = strtol(esp_gps->item_str, NULL, 10);
                switch (item_num % 4)
                {
                case 0:
                    gps_state.sats_desc_in_view[index].num = (uint8_t)value;
                    break;
                case 1:
                    gps_state.sats_desc_in_view[index].elevation = (uint8_t)value;
                    break;
                case 2:
                    gps_state.sats_desc_in_view[index].azimuth = (uint16_t)value;
                    break;
                case 3:
                    gps_state.sats_desc_in_view[index].snr = (uint8_t)value;
                    break;
                default:
                    break;
                }
            }
        }
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_RMC
/**
 * @brief Parse RMC statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_rmc(esp_gps_t *esp_gps)
{
    /* Process GPRMC statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Process UTC time */
        parse_utc_time(esp_gps);
        break;
    case 2: /* Process valid status */
        gps_state.valid = (esp_gps->item_str[0] == 'A');
        break;
    case 3: /* Latitude */
        gps_state.cur_crd.latitude = parse_lat_long(esp_gps);
        break;
    case 4: /* Latitude north(1)/south(-1) information */
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
        {
            gps_state.cur_crd.latitude *= -1;
        }
        break;
    case 5: /* Longitude */
        gps_state.cur_crd.longitude = parse_lat_long(esp_gps);
        break;
    case 6: /* Longitude east(1)/west(-1) information */
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
        {
            gps_state.cur_crd.longitude *= -1;
        }
        break;
    case 7: /* Process ground speed in unit m/s */
        gps_state.cur_crd.speed = strtof(esp_gps->item_str, NULL) * 1.852;
        break;
    case 8: /* Process true course over ground */
        gps_state.cog = strtof(esp_gps->item_str, NULL);
        break;
    case 9: /* Process date */
        gps_state.cur_crd.date.day = convert_two_digit2number(esp_gps->item_str + 0);
        gps_state.cur_crd.date.month = convert_two_digit2number(esp_gps->item_str + 2);
        gps_state.cur_crd.date.year = convert_two_digit2number(esp_gps->item_str + 4);
        break;
    case 10: /* Process magnetic variation */
        gps_state.variation = strtof(esp_gps->item_str, NULL);
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_GLL
/**
 * @brief Parse GLL statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gll(esp_gps_t *esp_gps)
{
    /* Process GPGLL statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Latitude */
        gps_state.cur_crd.latitude = parse_lat_long(esp_gps);
        break;
    case 2: /* Latitude north(1)/south(-1) information */
        if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's')
        {
            gps_state.cur_crd.latitude *= -1;
        }
        break;
    case 3: /* Longitude */
        gps_state.cur_crd.longitude = parse_lat_long(esp_gps);
        break;
    case 4: /* Longitude east(1)/west(-1) information */
        if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w')
        {
            gps_state.cur_crd.longitude *= -1;
        }
        break;
    case 5: /* Process UTC time */
        parse_utc_time(esp_gps);
        break;
    case 6: /* Process valid status */
        gps_state.valid = (esp_gps->item_str[0] == 'A');
        break;
    default:
        break;
    }
}
#endif

#if CONFIG_NMEA_STATEMENT_VTG
/**
 * @brief Parse VTG statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_vtg(esp_gps_t *esp_gps)
{
    /* Process GPVGT statement */
    switch (esp_gps->item_num)
    {
    case 1: /* Process true course over ground */
        gps_state.cog = strtof(esp_gps->item_str, NULL);
        break;
    case 3: /* Process magnetic variation */
        gps_state.variation = strtof(esp_gps->item_str, NULL);
        break;
    case 5:                                                                /* Process ground speed in unit m/s */
        gps_state.cur_crd.speed = strtof(esp_gps->item_str, NULL) * 1.852; // knots to m/s
        break;
    case 7:                                                              /* Process ground speed in unit m/s */
        gps_state.cur_crd.speed = strtof(esp_gps->item_str, NULL) / 3.6; // km/h to m/s
        break;
    default:
        break;
    }
}
#endif

/**
 * @brief Parse received item
 *
 * @param esp_gps esp_gps_t type object
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t parse_item(esp_gps_t *esp_gps)
{
    esp_err_t err = ESP_OK;
    /* start of a statement */
    if (esp_gps->item_num == 0 && esp_gps->item_str[0] == '$')
    {
        if (0)
        {
        }
#if CONFIG_NMEA_STATEMENT_GGA
        else if (strstr(esp_gps->item_str, "GGA"))
        {
            esp_gps->cur_statement = STATEMENT_GGA;
        }
#endif
#if CONFIG_NMEA_STATEMENT_GSA
        else if (strstr(esp_gps->item_str, "GSA"))
        {
            esp_gps->cur_statement = STATEMENT_GSA;
        }
#endif
#if CONFIG_NMEA_STATEMENT_RMC
        else if (strstr(esp_gps->item_str, "RMC"))
        {
            esp_gps->cur_statement = STATEMENT_RMC;
        }
#endif
#if CONFIG_NMEA_STATEMENT_GSV
        else if (strstr(esp_gps->item_str, "GSV"))
        {
            esp_gps->cur_statement = STATEMENT_GSV;
        }
#endif
#if CONFIG_NMEA_STATEMENT_GLL
        else if (strstr(esp_gps->item_str, "GLL"))
        {
            esp_gps->cur_statement = STATEMENT_GLL;
        }
#endif
#if CONFIG_NMEA_STATEMENT_VTG
        else if (strstr(esp_gps->item_str, "VTG"))
        {
            esp_gps->cur_statement = STATEMENT_VTG;
        }
#endif
        else
        {
            esp_gps->cur_statement = STATEMENT_UNKNOWN;
        }
        goto out;
    }
    /* Parse each item, depend on the type of the statement */
    if (esp_gps->cur_statement == STATEMENT_UNKNOWN)
    {
        goto out;
    }
#if CONFIG_NMEA_STATEMENT_GGA
    else if (esp_gps->cur_statement == STATEMENT_GGA)
    {
        parse_gga(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_GSA
    else if (esp_gps->cur_statement == STATEMENT_GSA)
    {
        parse_gsa(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_GSV
    else if (esp_gps->cur_statement == STATEMENT_GSV)
    {
        parse_gsv(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_RMC
    else if (esp_gps->cur_statement == STATEMENT_RMC)
    {
        parse_rmc(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_GLL
    else if (esp_gps->cur_statement == STATEMENT_GLL)
    {
        parse_gll(esp_gps);
    }
#endif
#if CONFIG_NMEA_STATEMENT_VTG
    else if (esp_gps->cur_statement == STATEMENT_VTG)
    {
        parse_vtg(esp_gps);
    }
#endif
    else
    {
        err = ESP_FAIL;
    }
out:
    return err;
}

/**
 * @brief Parse NMEA statements from GPS receiver
 *
 * @param esp_gps esp_gps_t type object
 * @param len number of bytes to decode
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t gps_decode(esp_gps_t *esp_gps, size_t len)
{
    const uint8_t *d = esp_gps->buffer;
    while (*d)
    {
        /* Start of a statement */
        if (*d == '$')
        {
            /* Reset runtime information */
            esp_gps->asterisk = 0;
            esp_gps->item_num = 0;
            esp_gps->item_pos = 0;
            esp_gps->cur_statement = 0;
            esp_gps->crc = 0;
            esp_gps->sat_count = 0;
            esp_gps->sat_num = 0;
            /* Add character to item */
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        }
        /* Detect item separator character */
        else if (*d == ',')
        {
            /* Parse current item */
            parse_item(esp_gps);
            /* Add character to CRC computation */
            esp_gps->crc ^= (uint8_t)(*d);
            /* Start with next item */
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        }
        /* End of CRC computation */
        else if (*d == '*')
        {
            /* Parse current item */
            parse_item(esp_gps);
            /* Asterisk detected */
            esp_gps->asterisk = 1;
            /* Start with next item */
            esp_gps->item_pos = 0;
            esp_gps->item_str[0] = '\0';
            esp_gps->item_num++;
        }
        /* End of statement */
        else if (*d == '\r')
        {
            /* Convert received CRC from string (hex) to number */
            uint8_t crc = (uint8_t)strtol(esp_gps->item_str, NULL, 16);
            /* CRC passed */
            if (esp_gps->crc == crc)
            {
                switch (esp_gps->cur_statement)
                {
#if CONFIG_NMEA_STATEMENT_GGA
                case STATEMENT_GGA:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GGA;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_GSA
                case STATEMENT_GSA:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GSA;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_RMC
                case STATEMENT_RMC:
                    esp_gps->parsed_statement |= 1 << STATEMENT_RMC;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_GSV
                case STATEMENT_GSV:
                    if (esp_gps->sat_num == esp_gps->sat_count)
                    {
                        esp_gps->parsed_statement |= 1 << STATEMENT_GSV;
                    }
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_GLL
                case STATEMENT_GLL:
                    esp_gps->parsed_statement |= 1 << STATEMENT_GLL;
                    break;
#endif
#if CONFIG_NMEA_STATEMENT_VTG
                case STATEMENT_VTG:
                    esp_gps->parsed_statement |= 1 << STATEMENT_VTG;
                    break;
#endif
                default:
                    break;
                }
                /* Check if all statements have been parsed */
                if (((esp_gps->parsed_statement) & esp_gps->all_statements) == esp_gps->all_statements)
                {
                    esp_gps->parsed_statement = 0;
                    // TODO Send signal to notify that GPS information has been updated */
                }
            }
            else
            {
                ESP_LOGD(GPS_TAG, "CRC Error for statement:%s", esp_gps->buffer);
            }
            if (esp_gps->cur_statement == STATEMENT_UNKNOWN)
            {
                // TODO Send signal to notify that one unknown statement has been met */
            }
        }
        /* Other non-space character */
        else
        {
            if (!(esp_gps->asterisk))
            {
                /* Add to CRC */
                esp_gps->crc ^= (uint8_t)(*d);
            }
            /* Add character to item */
            esp_gps->item_str[esp_gps->item_pos++] = *d;
            esp_gps->item_str[esp_gps->item_pos] = '\0';
        }
        /* Process next character */
        d++;
    }
    save_last_valid_crd();
    return ESP_OK;
}

//======================================================================
static void rcv_task(void *arg)
{
    esp_gps_t *esp_gps = (esp_gps_t *)malloc(sizeof(esp_gps_t));

    uart_event_t event;

    while (true)
    {
        if (xQueueReceive(uart_queue, &event, pdMS_TO_TICKS(200)))
        {
            switch (event.type)
            {
            case UART_DATA:
                //                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                //                int len = uart_read_bytes(GPS_UART_NUM, str, event.size, portMAX_DELAY);
                //                ESP_LOG_BUFFER_HEXDUMP(TAG, str, len, ESP_LOG_INFO);
                break;
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "HW FIFO Overflow");
                uart_flush(GPS_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "Ring Buffer Full");
                break;
            case UART_BREAK:
                ESP_LOGW(TAG, "Rx Break");
                uart_flush(GPS_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_PARITY_ERR:
                ESP_LOGE(TAG, "Parity Error");
                uart_flush(GPS_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_FRAME_ERR:
                ESP_LOGE(TAG, "Frame Error");
                uart_flush(GPS_UART_NUM);
                xQueueReset(uart_queue);
                break;
            case UART_PATTERN_DET:
            {
                size_t buffered_size;
                uart_get_buffered_data_len(GPS_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(GPS_UART_NUM);
                // ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1)
                {
                    uart_flush_input(GPS_UART_NUM);
                }
                else
                {
                    int len = uart_read_bytes(GPS_UART_NUM, esp_gps->buffer, pos, 100 / portTICK_PERIOD_MS);
                    esp_gps->buffer[len] = 0;
                    uint8_t pat[2] = {0};
                    uart_read_bytes(GPS_UART_NUM, pat, 1, 100 / portTICK_PERIOD_MS);

                    // ESP_LOGI(TAG, "<== '%s'", esp_gps->buffer);
                    //                     if (esp_gps->buffer[0] != '$')          ESP_LOG_BUFFER_HEXDUMP(TAG, esp_gps->buffer, len, ESP_LOG_INFO);

                    if (gps_decode(esp_gps, len + 1) != ESP_OK)
                    {
                        ESP_LOGW(GPS_TAG, "GPS decode line failed");
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

void start_gps()
{
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);
    esp_log_level_set(GPS_TAG, ESP_LOG_VERBOSE);

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    ESP_LOGI(TAG, "going to call uart_driver_install");
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, RX_GPS_BUF_SIZE, TX_GPS_BUF_SIZE, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(GPS_UART_NUM, '\n', 1, 9, 0, 0));
    ESP_ERROR_CHECK(uart_pattern_queue_reset(GPS_UART_NUM, 16));

    //--------------
    // UNX SyncChars[2]=0xB5,0x62 + CLASS[1] + ID[1] + LENGTH[2] + PAYLOAD[LENGHT] + CK_AB[2]
    // The Class defines the basic subset of the message (0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc)
    // ID Field defines the message that is to follow (CFG-RXM 0x06 0x11 2 Set/Get RXM configuration)
    // payload: 0 U1 - reserved - set to 8
    //          1 U1 - lpMode. 0: Max. performance mode. 1: Power Save Mode (>= FW 6.00 only) 4: Eco mode
    // CK_AB -> range: CLASS...PAYLOAD
    // uint8_t cmd_CK_AB[2] = {0, 0};    for (size_t i = 2; i < sizeof(cmd_power_saving_mode); i++)    {        cmd_CK_AB[0] += cmd_power_saving_mode[i];        cmd_CK_AB[1] += cmd_CK_AB[0];    }

    uint8_t cmd_CFG_RXM[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
    uart_write_bytes(GPS_UART_NUM, cmd_CFG_RXM, sizeof(cmd_CFG_RXM));
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // uint8_t cmd_RF_OFF[]={0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74};
    // uart_write_bytes(GPS_UART_NUM, cmd_RF_OFF, sizeof(cmd_RF_OFF)); 50mA -> 18mA

    uart_flush(GPS_UART_NUM);
    ESP_LOGI(TAG, "GPS started");

    if (xTaskCreate(rcv_task, "gps_rcv_task", 2048, NULL, 12, NULL) != pdTRUE)
    {
        ESP_LOGE(TAG, "NMEA xTaskCreate failed");
    }
}
