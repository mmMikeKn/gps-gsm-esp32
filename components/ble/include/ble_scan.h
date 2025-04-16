#ifndef _BLE_SCAN_H
#define _BLE_SCAN_H

#include "esp_types.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"

#define BLE_SCAN_TIME_SEC (20)
#define MAX_SCAN_RECORDS (20)

typedef struct {
    esp_bd_addr_t bda;                          /*!< Bluetooth device address which has been searched */
    esp_bt_dev_type_t dev_type;                 /*!< Device type */
    esp_ble_addr_type_t ble_addr_type;          /*!< Ble device address type */
    int rssi;                                   /*!< Searched device's RSSI */
    uint8_t  ble_adv[ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX];     /*!< Received EIR */
} ble_scan_data_t;

typedef struct {
    ble_scan_data_t scan_record[MAX_SCAN_RECORDS];
    uint8_t scan_records_num;
} ble_scan_result_t;


//===============================
extern ble_scan_result_t ble_scan_result;
extern void init_ble_scan();
extern void do_ble_scan();
extern char *scan_record_to_str(ble_scan_data_t *p, char *str, int max_sz, bool is_for_html);

extern int hex2bin(char *xStr, uint8_t *bin, int len);

#endif
