#include "esp32/rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hd44780.hpp"

using namespace jhall;

LCD::LCD(gpio_num_t rs, gpio_num_t e,
                 gpio_num_t d4, gpio_num_t d5, gpio_num_t d6, gpio_num_t d7,
                 uint32_t line_count)
    : m_rs(rs), m_e(e),
      m_d4(d4), m_d5(d5), m_d6(d6), m_d7(d7),
      m_display_state(ON), m_cursor_state(OFF), m_blink_state(OFF)
{
    uint64_t mask = 
        (1ULL<<m_rs) | (1ULL<<m_e) | 
        (1ULL<<m_d4) | (1ULL<<m_d5) | (1ULL<<m_d6) | (1ULL<<m_d7);
    gpio_config_t conf = {
        mask,                   // pin_bit_mask
        GPIO_MODE_OUTPUT,       // mode
        GPIO_PULLUP_DISABLE,    // pull_up_en
        GPIO_PULLDOWN_DISABLE,  // pull_down_en
        GPIO_INTR_DISABLE,      // intr_type
    };
    gpio_config(&conf);

    gpio_set_level(m_rs, 0);
    gpio_set_level(m_e, 0);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Initialize 4-bit mode
    write_nibble(0x3);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    write_nibble(0x3);
    ets_delay_us(100);
    write_nibble(0x3);
    write_nibble(0x2);

    // Set function: 4-bit data, 1/2 lines, and 5x8 font
    write_byte(0, 0x20 | (line_count > 1) << 3);

    // Set entry mode: increment and no shift
    write_byte(0, 0x06);

    // Display on, cursor off, and blink off
    write_control();

    // Clear display and home cursor
    clear();
}

LCD::~LCD() {}

void LCD::clear() const
{
    write_byte(0, 0x01);
    ets_delay_us(2000); // Command settling time > 1.6ms
}

void LCD::set_cursor(uint32_t row, uint32_t col) const
{
    uint32_t value = col;
    switch (row) {
    default:
        break;
    case 1:
        value += 0x40;
        break;
    case 2:
        value += 0x14;
        break;
    case 3:
        value += 0x54;
        break;
    }
    value |= 0x80;
    write_byte(0, value);
}

void LCD::print(char c) const
{
    write_byte(1, c);
}

void LCD::print(const std::string &s) const
{
    for (char const &c : s) {
        write_byte(1, c);
    }
}

void LCD::printf(const char *format, ...) const
{
    char text[81];
    va_list ap;
    va_start(ap, format);
    (void)vsnprintf(text, sizeof(text), format, ap);
    va_end(ap);
    print(text);
}

void LCD::set_display_state(state_t s)
{
    m_display_state = s;
    write_control();
}

void LCD::set_cursor_state(state_t s)
{
    m_cursor_state = s;
    write_control();
}

void LCD::set_blink_state(state_t s)
{
    m_blink_state = s;
    write_control();
}

void LCD::define_char(uint32_t code, const std::array<char, 8> &bitmap) const
{
    // Set CGRAM address
    write_byte(0, 0x40 | ((code & 0x7) << 3));
    for (auto elem : bitmap) {
        write_byte(1, elem & 0x1f);
    }
}

void LCD::write_byte(uint32_t rs, uint32_t b) const
{
    gpio_set_level(m_rs, rs);

    write_nibble(b >> 4);
    write_nibble(b);
}

void LCD::write_nibble(uint32_t n) const
{
    gpio_set_level(m_d4, (n&0x1) != 0);
    gpio_set_level(m_d5, (n&0x2) != 0);
    gpio_set_level(m_d6, (n&0x4) != 0);
    gpio_set_level(m_d7, (n&0x8) != 0);

    // Minimum enable pulse width is 300ns; might need to 
    // check if gpio_set_level() is slow enough to meet this
    gpio_set_level(m_e, 1);
    gpio_set_level(m_e, 0);

    ets_delay_us(40); // Command settling time > 37us   
}

void LCD::write_control() const
{
    uint32_t value = 0x08;
    if (m_display_state == ON) {
        value |= 0x04;
    }
    if (m_cursor_state == ON) {
        value |= 0x02;
    }
    if (m_blink_state == ON) {
        value |= 0x01;
    }
    write_byte(0, value);
}