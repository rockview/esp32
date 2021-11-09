#include "http.hpp"

using namespace jhall;

HTTP::HTTP()
    : m_server(nullptr)
{
}

HTTP::~HTTP()
{
    stop();
}

esp_err_t HTTP::start()
{
    if (m_server != nullptr) {
        return ESP_FAIL;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    auto err = httpd_start(&m_server, &cfg);
    if (err != ESP_OK) {
        return err;
    }

    httpd_uri_t get = {
        "/config",      // uri
        HTTP_GET,       // method
        get_handler,    // handler
        nullptr,        // user_ctx
    };
    httpd_register_uri_handler(m_server, &get);

    httpd_uri_t post = {
        "/submit",      // uri
        HTTP_POST,      // method
        post_handler,   // handler
        nullptr,        // user_ctx
    };
    httpd_register_uri_handler(m_server, &post);

    return ESP_OK;
}

void HTTP::stop()
{
    if (m_server != nullptr) {
        httpd_stop(m_server);
    }
}

esp_err_t HTTP::get_handler(httpd_req_t *req)
{
    const char resp[] = "Hello, World!";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t HTTP::post_handler(httpd_req_t *req)
{
    return ESP_OK;
}