#ifndef DS3231_HPP
#define DS3231_HPP

#include <cstddef>
#include <ctime>
#include <memory>
#include "driver/i2c.h"
#include "driver/gpio.h"

namespace jhall {

/**
 * @brief The RTC class supports I2C communications with an DS3231 RTC chip.
 */
class RTC {
public:
    /**
     * @brief Constructor.
     * @param port: The I2C port.
     * @param sda: The gpio pin connected to the SDA line.
     * @param scl: The gpio pin connected to the SCL line.
     */
    RTC(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);

    /**
     * @brief Destructor.
     */
    ~RTC();
    
    /**
     * @brief Set DS3231 time.
     * @param t Number of seconds since the Unix epoch 1970-1-1T00:00:00Z.
     * @param hours_24 Whether the DS3231 stores time in 12 hour (false) or 24
     * hour (true) format.
     * @return 
     *  - ESP_OK: success.
     *  - ESP_ERR_INVALID_ARG: the time could not be converted.
     *  - other: I2C communication failed.
     */
    esp_err_t set_time(time_t t, bool hours_24) const;

    /**
     * @brief Get DS3231 time.
     * @return 
     *  - Number of seconds since the Unix epoch 1970-1-1T00:00:00Z.
     *  - -1: I2C communication failed.
     */
    time_t get_time() const;

    /**
     * @brief Set DS3231 time.
     * @param dt Broken down time since the Unix epoch 1970-1-1T00:00:00Z.
     * @param hours_24 Whether the DS3231 stores time in 12 hour (false) or 24 hour (true) format.
     * @return 
     *  - ESP_OK: success.
     *  - other: I2C communication failed.
     */
    esp_err_t set_date_time(const struct tm &dt, bool hours_24) const;

    /**
     * @brief Get DS3231 time.
     * @param dt Broken down time since the Unix epoch 1970-1-1T00:00:00Z.
     * @return 
     *  - ESP_OK: success.
     *  - other: I2C communication failed.
     */
    esp_err_t get_date_time(struct tm *dt) const;

    /**
     * @brief A representation of the time register contents of the DS3231.
     */
    struct rtc_time_t {
        uint8_t seconds;    /*!< Seconds 00-59*/
        uint8_t minutes;    /*!< Minutes 00-59 */
        uint8_t hours;  /*!< Hours 1-12 (12 hour) or 00-23 (24 hour) */
        uint8_t wday;   /*!< Weekday 1-7 */
        uint8_t day;    /*!< Month day 01-31 */
        uint8_t month;  /*!< Month 01-12 */
        uint8_t year;   /*!< Year 00-99 (beginning in 1900) */
        uint8_t flags;  /*!< Flags */
    };

    /**
     * @brief Flags for rtc_time_t.flags.
     */
    enum rtc_flag_t {
        HOURS_24    = 1<<0, /*!< 24-hour time. */
        PM          = 1<<1, /*!< PM indicator for 12-hour time. */
        CENTURY     = 1<<2, /*!< Add 100 to year. */
    };

    /**
     * @brief Get DS3231 time.
     * @param dt RTC register time since the Unix epoch 1970-1-1T00:00:00Z.
     * @return 
     *  - ESP_OK: success.
     *  - other: I2C communication failed.
     */
    esp_err_t get_date_time(rtc_time_t *dt) const;

    /**
     * @brief DS3231 register addresses.
     */
    enum register_addr_t {
        SECONDS_REG = 0x00, /*!< Seconds register. */
        MINUTES_REG = 0x01, /*!< Minutes register. */
        HOURS_REG = 0x02,   /*!< Hours register. */
        DAY_REG = 0x03, /*!< Day register. */
        DATE_REG = 0x04,    /*!< Date register. */
        MONTH_REG = 0x05,   /*!< Month register. */
        YEAR_REG = 0x06,    /*!< Year register. */
        CONTROL_REG = 0x0e, /*!< Control register. */
        STATUS_REG = 0x0f,  /*!< Status register. */
        AGING_REG = 0x10,   /*!< Aging register. */
        MSB_TEMP_REG = 0x11,    /*!< MSB temperature register. */
        LSB_TEMP_REG = 0x12,    /*!< LSB temperature register. */
        LAST_REG = LSB_TEMP_REG, /*!< Last valid register addr. */
    };

    /**
     * @brief Control register flags.
     */
    enum {
        CONTROL_EOSC_   = (1<<7),   /*!< Enable oscillator. */
        CONTROL_BBSQW   = (1<<6),   /*!< Battery-backed square-wave enable. */
        CONTROL_CONV    = (1<<5),   /*!< Convert temperature. */
        CONTROL_RS2     = (1<<4),   /*!< Rate select bit 2. */
        CONTROL_RS1     = (1<<3),   /*!< Rate select bit 1. */
        CONTROL_INTCN   = (1<<2),   /*!< Interrupt control. */
        CONTROL_A2IE    = (1<<1),   /*!< Alarm 2 interrupt enable. */
        CONTROL_A1IE    = (1<<0),   /*!< Alarm 1 interrupt enable. */
    };

    /**
     * 
     * @brief Status register flags.
     */
    enum {
        STATUS_OSF      = (1<<7), /*!< Oscillator stop flag. */
        STATUS_EN32KHZ  = (1<<3), /*!< Enable 32kHz output. */
        STATUS_BSY      = (1<<2), /*!< Device busy. */
        STATUS_A2E      = (1<<1), /*!< Alarm 2 flag. */
        STATUS_A1E      = (1<<0), /*!< Alarm 1 flag. */
    };
    
    /**
     * @brief Read DS3231 register value.
     * @param reg Register address.
     * @param value Register value.
     * @return
     *  - ESP_OK: success.
     *  - ESP_ERR_INVALID_ARG: invalid register.
     *  - other: I2C communication failed.
     */
    esp_err_t read_register(register_addr_t reg, uint8_t *value) const;

     /**
     * @brief Write DS3231 register value.
     * @param reg Register address.
     * @param value Register value.
     * @return
     *  - ESP_OK: success.
     *  - ESP_ERR_INVALID_ARG: invalid register.
     *  - other: I2C communication failed.
     */   
    esp_err_t write_register(register_addr_t reg, uint8_t value) const;


    /**
     * @brief Read temperature registeres.
     * @param temp Temperature (degress centigrade).
     * @return
     *  - ESP_OK: success.
     *  - other: I2C communication failed.
     */
    esp_err_t read_temp(double *temp) const;

    /**
     * @brief Force temperature conversion.
     * This has the side effect of running the TCXO algorithm to update the capacitance array.
     * @return
     *  - ESP_OK: success.
     *  - other: I@C communication failure.
     */
    esp_err_t force_temp_conv() const;

private:
    esp_err_t read_time(rtc_time_t *rt) const;
    esp_err_t write_time(rtc_time_t &rt) const;

    i2c_port_t m_port;
}; // class RTC

} // namespace jhall

#endif // HD44780_HPP