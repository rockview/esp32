#ifndef HD44780_HPP
#define HD44780_HPP

#include <cstddef>
#include <string>
#include <array>
#include "driver/gpio.h"

namespace jhall {

/**
 * @brief The LCD class supports communication with a HD44780-controlled LCD display.
 */
class LCD {
public:
 
    /**
     * @brief Constructor.
     * @param rs GPIO pin connected to the RS pin of the LCD.
     * @param e GPIO pin connected to the E pin of the LCD.
     * @param d4 GPIO pin connected to the D4 pin of the LCD.
     * @param d5 GPIO pin connected to the D5 pin of the LCD.
     * @param d6 GPIO pin connected to the D6 pin of the LCD.
     * @param d7 GPIO pin connected to the D7 pin of the LCD.
     * @param line_count The number of lines in the LCD.
     */
    LCD(gpio_num_t rs, gpio_num_t e,
        gpio_num_t d4, gpio_num_t d5, gpio_num_t d6, gpio_num_t d7,
        uint32_t line_count = 2);
    /**
     * @brief Destructor.
     */
    ~LCD();

    /**
     * @brief Clear display and return cursor to home position (0, 0).
     */
    void clear() const;

    /**
     * @brief Set cursor position.
     * @param row Row address.
     * @param col Column address.
     */
    void set_cursor(uint32_t row, uint32_t col) const;

    /**
     * @brief Print character at current cursor position.
     * @param c Character to display.
     */
    void print(char c) const;

    /**
     * @brief Print string at current cursor position.
     * @param s String to display.
     */
    void print(std::string const &s) const;

    /**
     * @brief Print formatted string at current cursor position.
     * @param format Format specification and arguments.
     */
    void printf(const char *format...) const;

    /**
     * @brief Display property states.
     */
    enum state_t {
        ON, /*!< ON state. */
        OFF /*!< OFF state. */
    };

    /**
     * @brief Set display state.
     * @param s Display state.
     */
    void set_display_state(state_t s);

    /**
     * @brief Set cursor state.
     * @param s Cursor state.
     */
    void set_cursor_state(state_t s);

    /**
     * @brief Set cursor blink state.
     * @param s Blink state.
     */
    void set_blink_state(state_t s);

    /**
     * @brief Define custom character bitmap.
     * @param code Character code (0-7).
     * @param bitmap Bitmap (8 rows of 5 columns each).
     */
    void define_char(uint32_t code, const std::array<char, 8> &bitmap) const;

private:
    void write_byte(uint32_t rs, uint32_t b) const;
    void write_nibble(uint32_t n) const;
    void write_control() const;

    gpio_num_t m_rs;
    gpio_num_t m_e;
    gpio_num_t m_d4;
    gpio_num_t m_d5;
    gpio_num_t m_d6;
    gpio_num_t m_d7;

    state_t m_display_state;
    state_t m_cursor_state;
    state_t m_blink_state;
};

} // namespace jhall

#endif // HD44780_HPP