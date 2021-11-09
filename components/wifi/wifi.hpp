#ifndef WIFI_HPP
#define WIFI_HPP

#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event_base.h"

namespace jhall {

class WiFi {
public:
    WiFi();
    ~WiFi();

    void set_ap_credentials(std::string ssid, std::string pass);
    void set_sta_credentials(std::string ssid, std::string pass);

    esp_err_t start(wifi_mode_t mode);
    esp_err_t shutdown();

private:
    esp_err_t init_wifi(wifi_mode_t mode);
    esp_err_t config_ap() const;
    esp_err_t config_sta() const;
    esp_err_t wait_for_connect();
    static void event_handler(
        void *arg, 
        esp_event_base_t event_base,
        int32_t event_id,
        void *event_data
    );
    esp_err_t handle_wifi_event(int32_t event_id, void *event_data);
    esp_err_t handle_ip_event(int32_t event_id, void *event_data);
    
    int m_retry_count;
    esp_event_handler_instance_t m_wifi_instance;
    esp_event_handler_instance_t m_ip_instance;
    EventGroupHandle_t m_wifi_event_group;
    
    std::string m_ap_ssid;
    std::string m_ap_pass;
    std::string m_sta_ssid;
    std::string m_sta_pass;
};

} // namespace jhall

#endif // WIFI_HPP  