#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

#include "nvs.h"

namespace jhall {

class Config {
public:
    Config();

    virtual ~Config();

    esp_err_t erase() const;

    esp_err_t set(const char *key, std::string const &value) const;
    esp_err_t get(const char *key, std::string &value) const;

    esp_err_t set(const char *key, uint32_t value) const;
    esp_err_t get(const char *key, uint32_t *value) const;

    esp_err_t commit() const;

private:
    nvs_handle_t m_handle;

}; // class Config

} // namespace jhall

#endif // CONFIG_HPP