#include "WIFI.h"

#include <cstring>

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"

#include "esp_log.h"

static const char *TAG = "WIFI";

#define WIFI_AP_DEFAULT_SSID "OpenQS"
#define WIFI_AP_DEFAULT_PWD "00000000"

#define WIFI_AP_IP_ADDR "192.168.99.1"
#define WIFI_AP_NETMASK "255.255.255.0"
#define WIFI_AP_GATEWAY "192.168.99.1"

// #define SOCKET_SERVER_DEFAULT_ADDRESS "192.168.99.1"

#define WIFI_MAXIMUM_RETRY_CONNECTED 3
#define WIFI_SCAN_LIST_MAX_NUM 10

#define WIFI_STA_CONNECTED_BIT BIT0
#define WIFI_STA_FAIL_BIT BIT1
#define WIFI_AP_CONNECTED_BIT BIT2
#define WIFI_AP_FAIL_BIT BIT3

static bool exists = false;
static EventGroupHandle_t s_wifi_event_group;
static esp_netif_t *sta_netif, *ap_netif;
static esp_err_t err;
static int retry_connect_num = 0;
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT)
    {
        if (event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Connected to AP, Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
            xEventGroupSetBits(s_wifi_event_group, WIFI_STA_CONNECTED_BIT);
        }
    }
    else if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
            if (retry_connect_num >= 0)
            {
                if (retry_connect_num < WIFI_MAXIMUM_RETRY_CONNECTED)
                {
                    ESP_LOGI(TAG, "Retry to connect...");
                    retry_connect_num++;
                    esp_wifi_connect();
                }
                else
                {
                    ESP_LOGI(TAG, "Connection failed, too many retries.");
                    retry_connect_num = 0;
                }
            }
            else
            {
                retry_connect_num = 0;
            }
            xEventGroupSetBits(s_wifi_event_group, WIFI_STA_FAIL_BIT);
        }
        else if (event_id == WIFI_EVENT_SCAN_DONE)
        {
            wifi_event_sta_scan_done_t *event = (wifi_event_sta_scan_done_t *)event_data;
            if (event->status == 0)
            {
                uint16_t device_num = WIFI_SCAN_LIST_MAX_NUM;
                wifi_ap_record_t *ap_records = (wifi_ap_record_t *)calloc(1, sizeof(wifi_ap_record_t) * WIFI_SCAN_LIST_MAX_NUM);

                err = esp_wifi_scan_get_ap_records(&device_num, ap_records);
                if (err != ESP_OK)
                    ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records failed (%s)", esp_err_to_name(err));

                if (device_num > 0)
                {
                    ESP_LOGI(TAG, "Scan to %d device", device_num);
                    char mac[18] = {0};
                    for (uint16_t i = 0; i < device_num; i++)
                    {
                        sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x",
                                ap_records[i].bssid[0], ap_records[i].bssid[1],
                                ap_records[i].bssid[2], ap_records[i].bssid[3],
                                ap_records[i].bssid[4], ap_records[i].bssid[5]);
                        ESP_LOGI(TAG, "[%d] SSID:%s, MAC: %s, rssi:%d", i + 1, ap_records[i].ssid, mac, ap_records[i].rssi);
                    }
                }
                else
                    ESP_LOGI(TAG, "Unable to scan device");
                free(ap_records);
                err = esp_wifi_clear_ap_list();
                if (err != ESP_OK)
                    ESP_LOGE(TAG, "esp_wifi_scan_get_ap_records failed (%s)", esp_err_to_name(err));
            }
        }
        else if (event_id == WIFI_EVENT_AP_STACONNECTED)
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_AP_CONNECTED_BIT);
            ESP_LOGI(TAG, "WIFI_EVENT_AP_STACONNECTED");
        }
        else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_AP_FAIL_BIT);
            ESP_LOGI(TAG, "WIFI_EVENT_AP_STADISCONNECTED");
        }
    }
}
Wifi::Wifi()
{
}

Wifi::~Wifi()
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    if (ap_netif)
        esp_netif_destroy_default_wifi(ap_netif);
    if (sta_netif)
        esp_netif_destroy_default_wifi(sta_netif);
    esp_wifi_deinit();
    esp_netif_deinit();
}

void Wifi::init()
{
    if (exists)
        return;
    exists = true;

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    err = esp_netif_init();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_netif_init failed (%s)", esp_err_to_name(err));
    err = esp_event_loop_create_default();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_event_loop_create_default failed (%s)", esp_err_to_name(err));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id, instance_got_ip, instance_scan_done;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_SCAN_DONE, &wifi_event_handler, NULL, &instance_scan_done));

    sta_netif = esp_netif_create_default_wifi_sta();
    ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(sta_netif, DEVICE_NAME);

    // Initialize and start WiFi
    wifi_config_t wifi_sta_config = {}, wifi_ap_config = {};
    wifi_sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_sta_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_sta_config.sta.failure_retry_cnt = 3;

    memcpy(wifi_ap_config.ap.ssid, WIFI_AP_DEFAULT_SSID, sizeof(WIFI_AP_DEFAULT_SSID));
    memcpy(wifi_ap_config.ap.password, WIFI_AP_DEFAULT_PWD, sizeof(WIFI_AP_DEFAULT_PWD));
    wifi_ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_ap_config.ap.max_connection = 1;

    esp_netif_ip_info_t ip_info;
    ip4addr_aton(WIFI_AP_IP_ADDR, (ip4_addr_t *)&ip_info.ip);
    ip4addr_aton(WIFI_AP_NETMASK, (ip4_addr_t *)&ip_info.netmask);
    ip4addr_aton(WIFI_AP_GATEWAY, (ip4_addr_t *)&ip_info.gw);
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_event_group = xEventGroupCreate();
}

void Wifi::scan(bool sync)
{
    if (!exists)
        return;
    err = esp_wifi_scan_start(NULL, sync);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "esp_wifi_scan_start failed (%s)", esp_err_to_name(err));
}

void Wifi::connect(const char *ssid, const char *password, bool sync)
{
    if (!exists)
        return;
    wifi_config_t wifi_sta_config = {};
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_sta_config));

    wifi_ap_record_t ap_info;
    err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err == ESP_OK)
    {
        if (strncmp((char *)ap_info.ssid, ssid, sizeof(ap_info.ssid)) != 0)
        {
            ESP_LOGI(TAG, "Connected to a different AP, Disconnecting...");
            esp_wifi_disconnect();
        }
    }
    strncpy((char *)wifi_sta_config.sta.ssid, ssid, sizeof(wifi_sta_config.sta.ssid));
    strncpy((char *)wifi_sta_config.sta.password, password, sizeof(wifi_sta_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));
    ESP_LOGI(TAG, "Connecting to AP...");
    err = esp_wifi_connect();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Failed to connect to AP: %s", esp_err_to_name(err));
    if (!sync)
        return;
    wait_connect();
}

void Wifi::disconnect()
{
    if (!exists)
        return;
    esp_wifi_disconnect();
}

bool Wifi::wait_connect(uint32_t time_out)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_STA_CONNECTED_BIT | WIFI_STA_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        (time_out == 0) ? portMAX_DELAY : (time_out / portTICK_PERIOD_MS));
    if (bits & WIFI_STA_CONNECTED_BIT)
        ESP_LOGI(TAG, "Connected to ap SSID:%s, Password:%s", WIFI_AP_DEFAULT_SSID, WIFI_AP_DEFAULT_PWD);
    else if (bits & WIFI_STA_FAIL_BIT)
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, Password:%s", WIFI_AP_DEFAULT_SSID, WIFI_AP_DEFAULT_PWD);
    return false;
}
bool Wifi::wait_AP_connect(uint32_t time_out)
{
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_AP_CONNECTED_BIT | WIFI_AP_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        (time_out == 0) ? portMAX_DELAY : (time_out / portTICK_PERIOD_MS));
    if (bits & WIFI_AP_CONNECTED_BIT)
        ESP_LOGI(TAG, "Connected to sta.");
    else if (bits & WIFI_AP_FAIL_BIT)
        ESP_LOGI(TAG, "Failed to connect sta.");
    return false;
}