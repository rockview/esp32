#ifndef WS2812B_HPP
#define WS2812B_HPP

#include <vector>

#include "driver/rmt.h"

namespace jhall {

class WS2812B {
public:
    WS2812B(rmt_channel_t channel, gpio_num_t led_pin, uint32_t num_pixels);
    virtual ~WS2812B();

    esp_err_t set_pixel(uint32_t pixel, uint8_t r, uint8_t g, uint8_t b);
    esp_err_t show();
    esp_err_t clear();

private:
    static void translator_cb(const void *src, rmt_item32_t *dest, size_t src_size, 
        size_t wanted_num, size_t *translated_size, size_t *item_num);

    rmt_channel_t m_channel;
    std::vector<uint8_t> m_pixels;

}; // class WS2812B

} // namespace jhall

#endif // WS2812B_HPP