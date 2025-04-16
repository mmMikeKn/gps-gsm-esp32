#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "lwip/netif.h"
#include "lwip/prot/ip.h"
#include "lwip/prot/ip4.h"
#include "lwip/prot/tcp.h"
#include "lwip/prot/udp.h"
#include "cJSON.h"
#include "esp_timer.h"

#include "web_host.h"

#include "gps.h"
#include "sim800l.h"
#include "gyro.h"
#include "ble_scan.h"
#include "state_vals.h"

#define SCR_LINE_HD 0
#define SCR_LINE_SSID 1
#define SCR_LINE_PASWD 2
#define SCR_LINE_AP_MAC 3
#define SCR_LINE_STA_MAC 4
#define SCR_LINE_AP_IP 5
#define SCR_LINE_URI 6
#define SCR_LINE_WARN 7

static const char *TAG = "WebHost";

static uint8_t sta_mac[6], ap_mac[6];
static bool is_connected_sta = false;
static httpd_handle_t server = NULL;

//----------------------------------------------------------------
static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "set ip: " IPSTR, IP2STR(&(((ip_event_ap_staipassigned_t *)event_data)->ip)));
    return;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_AP_START:
        break;
    case WIFI_EVENT_AP_STACONNECTED:
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "STA " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        memcpy(sta_mac, event->mac, 6);
        is_connected_sta = true;
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED:
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "STA " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
        is_connected_sta = false;
        break;
    }
    }
}

static esp_err_t get_handler(httpd_req_t *req)
{
    current_state_vals.last_web_active_time = esp_timer_get_time();
    ESP_LOGI(TAG, "get_handler: [%s]", req->uri);

    if (strcmp(req->uri, "/favicon.ico") == 0)
    {
        extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
        extern const unsigned char favicon_ico_end[] asm("_binary_favicon_ico_end");
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_end - favicon_ico_start);
        return ESP_OK;
    }

    if (strcmp(req->uri, "/") == 0 || strcmp(req->uri, "/index.html") == 0)
    {
        extern const unsigned char index_html_start[] asm("_binary_index_html_start");
        extern const unsigned char index_html_end[] asm("_binary_index_html_end");
        ESP_LOGI(TAG, "send binary_index_html sz:%d", index_html_end - index_html_start);
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
        return ESP_OK;
    }

    if (strcmp(req->uri, "/config") == 0)
    {
        httpd_resp_set_type(req, "application/json");
        char *json = config_to_json();
        httpd_resp_send(req, (const char *)json, strlen(json));
        free(json);
        return ESP_OK;
    }

    if (strcmp(req->uri, "/state") == 0)
    {
        httpd_resp_set_type(req, "application/json");
        char *json = json_state();
        httpd_resp_send(req, (const char *)json, strlen(json));
        free(json);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "GET '%s' URI is not available", req->uri);
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "URI is not available");
    return ESP_FAIL;
}

static esp_err_t post_handler(httpd_req_t *req)
{
    current_state_vals.last_web_active_time = esp_timer_get_time();
    ESP_LOGI(TAG, "post_handler: [%s]", req->uri);
    char *json = malloc(req->content_len + 1);
    int ret = httpd_req_recv(req, json, req->content_len);
    if (ret <= 0)
    {
        free(json);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    json[ret] = 0;
    ESP_LOGI(TAG, "post body: '%s'", json);

    if (strcmp(req->uri, "/config") == 0)
    {
        parse_config_json(json);
        free(json);
        do_save_config = true;
        /*
        esp_err_t err = save_configs_to_nvs();
        if (err != ESP_OK)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
            return ESP_FAIL;
        }
        */

        return ESP_OK;
    }

    ESP_LOGE(TAG, "POST '%s' URI is not available", req->uri);
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "URI is not available");
    return ESP_FAIL;
}

//----------------------------------------------------------------
static bool start(const char *ssid, const char *password)
{
    is_connected_sta = false;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    wifi_config_t ap_config;
    memset(&ap_config, 0, sizeof(ap_config));
    strncpy((char *)ap_config.ap.ssid, ssid, sizeof(ap_config.ap.ssid) - 1);
    //    ap_config.ap.channel = 1;
    ap_config.ap.ssid_len = strlen((char *)ap_config.ap.ssid);
    ap_config.ap.max_connection = 4;
    ap_config.ap.beacon_interval = 100;

    if (strlen(password) == 0)
    {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    else
    {
        strcpy((char *)ap_config.ap.password, password);
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_AP, ap_mac));
    ESP_LOGI(TAG, "wifi_init finished. mac=" MACSTR " ssid=[%s] password=[%s]", MAC2STR(ap_mac), ssid, password);
    //    ESP_ERROR_CHECK(esp_wifi_set_channel(ap_config.ap.channel, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "+++++++ starting HTTP server");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_uri_t file_download = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = get_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &file_download);
        httpd_uri_t post_rq = {
            .uri = "/*",
            .method = HTTP_POST,
            .handler = post_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &post_rq);
        ESP_LOGI(TAG, "+++++++ started HTTP server");
        return true;
    }
    return false;
}

static void stop()
{
    ESP_LOGI(TAG, "++++++++ stoping HTTP server");
    if (server != NULL)
    {
        httpd_stop(server);
    }
    server = NULL;
    esp_wifi_set_promiscuous(false);
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &ip_event_handler);
    esp_wifi_stop();
    current_state_vals.is_web_host_runnung = false;
    ESP_LOGI(TAG, "--------- stoped HTTP server");
}

static void web_host(void *arg)
{
    current_state_vals.last_web_active_time = esp_timer_get_time();
    start("ESPH", "0123456789");
    while ((current_state_vals.last_web_active_time + config_guard.web_host_active_period * 1000L * 1000L) > esp_timer_get_time())
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    stop();
    vTaskDelete(NULL);
}

//----------------------------------------------------------------
void run_web_host()
{
    if (current_state_vals.is_web_host_runnung)
    {
        ESP_LOGI(TAG, "Web host already started");
        return;
    }
    current_state_vals.is_web_host_runnung = true;
    xTaskCreate(web_host, "web_host_task", 4096, NULL, 11, NULL);
}
