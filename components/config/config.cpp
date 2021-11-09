#include "nvs_flash.h"
#include "esp_log.h"

#include "config.hpp"

static const char *TAG = "config";

using namespace jhall;

#define MAX_STRING_SIZE 100 // including nul terminator

Config::Config()
{
    auto err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nvs_open("config", NVS_READWRITE, &m_handle));
}

Config::~Config()
{
    nvs_close(m_handle);
}

esp_err_t Config::erase() const
{
    return nvs_erase_all(m_handle);
}

esp_err_t Config::set(const char *key, std::string const &value) const
{
    if (value.size() + 1 > MAX_STRING_SIZE) {
        return ESP_ERR_NVS_VALUE_TOO_LONG;
    }
    return nvs_set_str(m_handle, key, value.c_str());
}

esp_err_t Config::get(const char *key, std::string &value) const
{
    size_t size = MAX_STRING_SIZE;
    char buf[MAX_STRING_SIZE];
    auto err = nvs_get_str(m_handle, key, buf, &size);
    if (err != ESP_OK) {
        return err;
    }
    value = std::string(buf);
    return ESP_OK;
}

esp_err_t Config::set(const char *key, uint32_t value) const
{
    return nvs_set_u32(m_handle, key, value);
}

esp_err_t Config::get(const char *key, uint32_t *value) const
{
    return nvs_get_u32(m_handle, key, value);
}

esp_err_t Config::commit() const
{
    return nvs_commit(m_handle);
}