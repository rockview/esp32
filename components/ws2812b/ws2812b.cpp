#include <cstring>

#include "ws2812b.hpp"

static const char *TAG = "WS2812B";

using namespace jhall;

#define NUM_COLOR_CHANNELS 3

WS2812B::WS2812B(rmt_channel_t channel, gpio_num_t led_pin, uint32_t num_pixels)
    : m_channel(channel)
{
    m_pixels.resize(num_pixels*NUM_COLOR_CHANNELS, 0);

    rmt_config_t cfg = RMT_DEFAULT_CONFIG_TX(led_pin, m_channel);
    cfg.clk_div = 4;    // 20 MHz (50ns) counter clock

    ESP_ERROR_CHECK(rmt_config(&cfg));
    ESP_ERROR_CHECK(rmt_driver_install(cfg.channel, 0, 0));

    ESP_ERROR_CHECK(rmt_translator_init(RMT_CHANNEL_0, translator_cb));
}
    
WS2812B::~WS2812B()
{
    rmt_driver_uninstall(m_channel);
}

esp_err_t WS2812B::set_pixel(size_t pixel, uint8_t r, uint8_t g, uint8_t b)
{
    size_t index = pixel*NUM_COLOR_CHANNELS;
    if (index >= m_pixels.size()) {
        return ESP_ERR_INVALID_ARG;
    }
    m_pixels[index + 0] = g;
    m_pixels[index + 1] = r;
    m_pixels[index + 2] = b;
    return ESP_OK;
}
    
esp_err_t WS2812B::show()
{
    return rmt_write_sample(m_channel, m_pixels.data(), m_pixels.size(), true);
    // reset?
}

esp_err_t WS2812B::clear()
{
    memset(m_pixels.data(), 0, m_pixels.size());
    return show();
}

void WS2812B::translator_cb(const void *src, rmt_item32_t *dest, size_t src_size, 
    size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    // 80Mhz / 4 = 20MHz; one tick every 0.05us; 7 ticks = 0.35us, 18 ticks = 0.9us
    static const rmt_item32_t zero = {{{7, 1, 18, 0}}};
    static const rmt_item32_t one = {{{18, 1, 7, 0}}};

    auto source = static_cast<const uint8_t *>(src);
    auto sp = source;
    auto ep = sp + src_size;
    auto dp = dest;
    while (sp < ep) {
        auto chan = *sp++;
        for (int bit = 7; bit >= 0; bit--) {
            if ((chan & (1 << bit)) != 0) {
                *dp++ = one;
            } else {
                *dp++ = zero;
            }
        }
        // Assumption: wanted_num is a multiple of 8
        if ((dp - dest) >= wanted_num) {
            break;
        }
    }
    *translated_size = sp - source;
    *item_num = dp - dest;
}