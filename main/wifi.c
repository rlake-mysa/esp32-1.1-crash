#include <sdkconfig.h>

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>

#define WIFI_CONNECTED_BIT BIT0

static EventGroupHandle_t wifi_event_group;

static void
wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                esp_wifi_connect();
                xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            default:
                break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
            case IP_EVENT_STA_GOT_IP:
                xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            default:
                break;
        }
    }
}

// configure and return station
static esp_netif_t*
wifi_init_sta(void)
{
    esp_netif_t* sta = esp_netif_create_default_wifi_sta();
    assert(sta);

    wifi_config_t wifi_config = {
        .sta = { .ssid = CONFIG_REMOTE_AP_SSID, .password = CONFIG_REMOTE_AP_PASSWORD },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    return sta;
}

// configure and return softAP
static esp_netif_t*
wifi_init_softap(void)
{
    esp_netif_t* ap = esp_netif_create_default_wifi_ap();
    assert(ap);

    wifi_config_t wifi_config = {
        .ap = { .ssid           = CONFIG_AP_SSID,
                .ssid_len       = strlen(CONFIG_AP_SSID),
                .password       = CONFIG_AP_PASSWORD,
                .max_connection = 4,
                .authmode       = WIFI_AUTH_WPA_WPA2_PSK },
    };

    if (strlen(CONFIG_AP_PASSWORD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));

    return ap;
}

void
wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_init_softap();
    wifi_init_sta();
    ESP_ERROR_CHECK(esp_wifi_start());
}