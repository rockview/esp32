#ifndef HTTP_HPP
#define HTTP_HPP

#include "esp_http_server.h"

namespace jhall {

class HTTP {
public:
    HTTP();
    ~HTTP();

    esp_err_t start();
    void stop();

private:
    static esp_err_t get_handler(httpd_req_t *req);
    static esp_err_t post_handler(httpd_req_t *req);

    httpd_handle_t m_server;
};

} // namespace jhall

#endif // HTTP_HPP
