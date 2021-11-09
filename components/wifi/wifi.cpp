#include <cstring>
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "wifi.hpp"

static const char *TAG = "WiFi";

using namespace jhall;

#define MAX_CONNECTION_RETRIES  5
#define WIFI_CONNECTED_BIT  (1<<0)
#define WIFI_FAIL_BIT       (1<<1)

WiFi::WiFi()
    : m_retry_count(0)
    , m_wifi_instance(nullptr)
    , m_ip_instance(nullptr)
    , m_wifi_event_group(xEventGroupCreate())
    , m_ap_ssid("")
    , m_ap_pass("")
    , m_sta_ssid("")
    , m_sta_pass("")
{
    if (m_wifi_event_group == nullptr) {
        ESP_LOGI(TAG, "couldn't create event group");
        abort();    // TODO
    }
}

WiFi::~WiFi() 
{
    vEventGroupDelete(m_wifi_event_group);
}

void WiFi::set_ap_credentials(std::string ssid, std::string pass)
{
    m_ap_ssid = ssid;
    m_ap_pass = pass;
}

void WiFi::set_sta_credentials(std::string ssid, std::string pass)
{
    m_sta_ssid = ssid;
    m_sta_pass = pass;
}

esp_err_t WiFi::start(wifi_mode_t mode)
{
    auto err = init_wifi(mode);
    if (err != ESP_OK) {
        return err;
    }
    err = wait_for_connect();
    if (err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

esp_err_t WiFi::shutdown()
{
    // TODO: Is there more cleanup needed
    return esp_wifi_disconnect();
}

esp_err_t WiFi::init_wifi(wifi_mode_t mode)
{
    // 1. Wi-Fi/LwIP Init Phase
    auto err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_loop_create_default();
     if (err != ESP_OK) {
        return err;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&init_cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        event_handler,
        this,
        &m_wifi_instance
    );
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        event_handler,
        this,
        &m_ip_instance
    );
    if (err != ESP_OK) {
        return err;
    }

    // 2. Wi-Fi Configuration Phase
    err = esp_wifi_set_mode(mode);
    if (err != ESP_OK) {
        return err;
    }

    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        // TODO: save result for deletion
        (void)esp_netif_create_default_wifi_ap();
        err = config_ap();
        if (err != ESP_OK) {
            return err;
        }
    }
    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        // TODO: save result for deletion
        (void)esp_netif_create_default_wifi_sta();
        err = config_sta();
        if (err != ESP_OK) {
            return err;
        }
    }

    // 3. Wi-Fi Start Phase
    err = esp_wifi_start();
     if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t WiFi::config_ap() const 
{
    wifi_config_t cfg = {};
    auto &ap = cfg.ap;
    strlcpy(reinterpret_cast<char*>(ap.ssid), m_ap_ssid.c_str(), sizeof(ap.ssid));
    strlcpy(reinterpret_cast<char*>(ap.password), m_ap_pass.c_str(), sizeof(ap.password));
    ap.max_connection = 1;
    ap.authmode = WIFI_AUTH_WPA2_PSK;
    return esp_wifi_set_config(WIFI_IF_AP, &cfg);
}

esp_err_t WiFi::config_sta() const 
{
     wifi_config_t cfg = {};
    auto &sta = cfg.sta;
    strlcpy(reinterpret_cast<char*>(sta.ssid), m_sta_ssid.c_str(), sizeof(sta.ssid));
    strlcpy(reinterpret_cast<char*>(sta.password), m_sta_pass.c_str(), sizeof(sta.password));
    sta.pmf_cfg.capable = true;
    sta.pmf_cfg.required = false;
    return esp_wifi_set_config(WIFI_IF_STA, &cfg);
}

esp_err_t WiFi::wait_for_connect()
{
    EventBits_t bits = xEventGroupWaitBits(
        m_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "wait: WIFI_CONNECTED_BIT");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "wait: WIFI_FAIL_BIT");
    } else {
        ESP_LOGE(TAG, "unexpected event");
    }
    return ESP_FAIL;
}

void WiFi::event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
) {
    auto self = static_cast<WiFi*>(arg);
    if (event_base == WIFI_EVENT) {
        self->handle_wifi_event(event_id, event_data);
    } else if (event_base == IP_EVENT) {
        self->handle_ip_event(event_id, event_data);
   }
}

esp_err_t WiFi::handle_wifi_event(int32_t event_id, void *event_data)
{
    esp_err_t err = ESP_OK;
    switch (event_id) {
    case WIFI_EVENT_STA_START:
        err = esp_wifi_connect(); // 4. Wi-Fi Connect Phase
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "event: WIFI_EVENT_STA_CONNECTED");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        if (m_retry_count < MAX_CONNECTION_RETRIES) {
            err = esp_wifi_connect(); // 4. Wi-Fi Connect Phase
            m_retry_count++;
            ESP_LOGI(TAG, "event: WIFI_EVENT_STA_DISCONNECTED; retry attempt %d", m_retry_count);
        } else {
            xEventGroupSetBits(m_wifi_event_group, WIFI_FAIL_BIT);
        }
    default:
        break;
    }

    if (err != ESP_OK) {
       ESP_LOGE(TAG, "wifi event: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t WiFi::handle_ip_event(int32_t event_id, void *event_data)
{
    esp_err_t err = ESP_OK;
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP: {
            auto *event = static_cast<ip_event_got_ip_t*>(event_data);
            ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
            m_retry_count = 0;
            xEventGroupSetBits(m_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        break;
    default:
        break;
    }
    if (err != ESP_OK) {
       ESP_LOGE(TAG, "wifi event: %s", esp_err_to_name(err));
    }
    return err;
}