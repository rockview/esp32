#include <cmath>

#include "esp_log.h"

#include "ds3231.hpp"

static const char *TAG = "RTC";

using namespace jhall;

#define I2C_ADDR    (0x68)
#define I2C_TIMEOUT (100/portTICK_RATE_MS)

// --- Helper functions ---

// timegm() converts a broken-down time to a calendar time without taking
// the local timezone into account when performing the conversion.
// Unfortunately, is not available in ESP-IDF so this version is copied 
// from forum post by Sergey D:
// https://stackoverflow.com/questions/16647819/timegm-cross-platform
// Algorithm: http://howardhinnant.github.io/date_algorithms.html
static int days_from_civil(int y, int m, int d)
{
    y -= m <= 2;
    int era = y / 400;
    int yoe = y - era * 400;                                   // [0, 399]
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;  // [0, 365]
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;           // [0, 146096]
    return era * 146097 + doe - 719468;
}

static time_t timegm(tm const* t)     // It does not modify broken-down time
{
    int year = t->tm_year + 1900;
    int month = t->tm_mon;          // 0-11
    if (month > 11)
    {
        year += month / 12;
        month %= 12;
    }
    else if (month < 0)
    {
        int years_diff = (11 - month) / 12;
        year -= years_diff;
        month += 12 * years_diff;
    }
    int days_since_1970 = days_from_civil(year, month + 1, t->tm_mday);

    return 60 * (60 * (24L * days_since_1970 + t->tm_hour) + t->tm_min) + t->tm_sec;
}

static uint8_t to_bcd(int value)
{
    return ((value/10)<<4) + value%10;
}

static int from_bcd(uint8_t value, uint8_t hi_mask)
{
    return 10*((value&hi_mask)>>4) + (value&0x0f);
}

// --- Class methods ---

RTC::RTC(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
    : m_port(port) 
{
    i2c_config_t conf = {
        I2C_MODE_MASTER,        // mode
        sda,                    // sda_io_num
        scl,                    // scl_io_num
        GPIO_PULLUP_DISABLE,    // sda_pullup_en (breakout board has 10k pullup)
        GPIO_PULLUP_DISABLE,    // scl_pullup_en (breakout board has 10k pullup)
        {
            400000,             // master.clk_speed
        },
        0,                      // clk_flags
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));
}

RTC::~RTC() 
{
    i2c_driver_delete(m_port);
}

esp_err_t RTC::set_time(time_t t, bool hours_24) const 
{
    struct tm dt;
    if (gmtime_r(&t, &dt) == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_date_time(dt, hours_24);
}

time_t RTC::get_time() const 
{
    struct tm dt;
    auto err = get_date_time(&dt);
    if (err != ESP_OK) {
        return -1;
    }
    return timegm(&dt);
}

esp_err_t RTC::set_date_time(const struct tm& tm, bool hours_24) const 
{
    rtc_time_t rt;
    rt.seconds = to_bcd(tm.tm_sec);
    rt.minutes = to_bcd(tm.tm_min);
    if (hours_24) {
        rt.hours = to_bcd(tm.tm_hour);
    } else {
        auto hours = tm.tm_hour;
        auto pm = 0;
        if (hours > 12) {
            hours -= 12;
            pm = 1;
        }
        rt.hours = (1<<6) | (pm<<5) | to_bcd(hours);
    }
    rt.wday = tm.tm_wday + 1;
    rt.day = to_bcd(tm.tm_mday);
    int century = tm.tm_year > 99;
    int year = century? tm.tm_year - 100: tm.tm_year;
    rt.month = (century<<7) | to_bcd(tm.tm_mon + 1);
    rt.year = to_bcd(year);

    return write_time(rt);
}

esp_err_t RTC::get_date_time(struct tm *tm) const 
{
    rtc_time_t rt;
    auto result = read_time(&rt);
    if (result != ESP_OK) {
        return result;
    }

    tm->tm_sec = from_bcd(rt.seconds, 0x70);
    tm->tm_min = from_bcd(rt.minutes, 0x70);
    if (rt.hours&0x40) {
        // 12-hour clock
        tm->tm_hour = rt.hours&0x1f;
        if (rt.hours&0x20) {
            // PM
            tm->tm_hour += 12;
        }
    } else {
        // 24-hour clock
        tm->tm_hour = from_bcd(rt.hours, 0x30);
    }
    tm->tm_wday = rt.wday - 1;
    tm->tm_mday = from_bcd(rt.day, 0x30);
    tm->tm_mon = from_bcd(rt.month, 0x10) - 1;
    tm->tm_year = from_bcd(rt.year, 0xf0);
    if (rt.month&0x80) {
        tm->tm_year += 100;
    }
    tm->tm_wday = 0;
    tm->tm_yday = 0;
    tm->tm_isdst = 0;
    
    ESP_LOGD(TAG, "get_date_time(struct tm): %04d-%02d-%02dT%02d:%02d:%02d", 
        tm->tm_year + 1900, tm->tm_mon, tm->tm_mday,
        tm->tm_hour, tm->tm_min, tm->tm_sec);

    return ESP_OK;
}

esp_err_t RTC::get_date_time(rtc_time_t *rt) const 
{
    auto result = read_time(rt);
    if (result != ESP_OK) {
        return result;
    }

    // Fixup fields
    if (rt->hours&0x40) {
        // 12-hour clock
        if (rt->hours&0x20) {
            rt->flags = PM;
        } else {
            rt->flags = 0;
        }
        rt->hours &= 0x1f;
    } else {
        // 24-hour clock
        rt->flags = HOURS_24;
        rt->hours &= 0x3f;
    }
    if (rt->month&0x80) {
        rt->flags |= CENTURY;
    }
    rt->month &= 0x1f;

    ESP_LOGD(TAG, "get_date_time(rtc_time_t): %d%d/%d%d/%d%d %d%d:%d%d:%d%d%s", 
        rt->year>>4, rt->year&0x0f,
        rt->month>>4, rt->month&0x0f,
        rt->day>>4, rt->day&0x0f,
        rt->hours>>4, rt->hours&0x0f,
        rt->minutes>>4, rt->minutes&0x0f,
        rt->seconds>>4, rt->seconds&0x0f,
        (rt->flags&HOURS_24)? "": ((rt->flags&PM)? "PM": "AM"));

    return ESP_OK;
}

esp_err_t RTC::read_register(register_addr_t reg, uint8_t *value) const 
{
    if ((reg < 0) || (reg > LAST_REG)) {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    auto result = i2c_master_cmd_begin(m_port, cmd, I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);
        
    ESP_LOGD(TAG, "read_register: [%02x]->%02x", reg, *value);

    return result;
}

esp_err_t RTC::write_register(register_addr_t reg, uint8_t value) const 
{
    if ((reg < 0) || (reg > LAST_REG)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "write_register: [%02x]<-%02x", reg, value);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true); 
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    auto result = i2c_master_cmd_begin(m_port, cmd, I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return result;
}

esp_err_t RTC::read_temp(double *temp) const
{
    uint8_t msb;
    uint8_t lsb;
    auto err = read_register(RTC::MSB_TEMP_REG, &msb);
    if (err != ESP_OK) {
        return err;
    }
    err = read_register(RTC::LSB_TEMP_REG, &lsb);
    if (err != ESP_OK) {
        return err;
    }
    *temp = double((int16_t(msb)<<8) | int16_t(lsb)) / (1<<8);
    return ESP_OK;
}


esp_err_t RTC::force_temp_conv() const {
    esp_err_t err;
    for (;;) {
        uint8_t b;
        err = read_register(RTC::STATUS_REG, &b);
        if (err != ESP_OK) {
            break;
        }
        if ((b & STATUS_BSY) != 0) {
            continue;   // Busy wait
        }
        err = read_register(RTC::CONTROL_REG, &b);
        if (err != ESP_OK) {
            break;
        }
        err = write_register(RTC::CONTROL_REG, b | CONTROL_CONV);
        break;
    }
    return err;
}

esp_err_t RTC::read_time(rtc_time_t *rt) const 
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SECONDS_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, reinterpret_cast<uint8_t*>(rt), 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    auto result = i2c_master_cmd_begin(m_port, cmd, I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);

    ESP_LOGD(TAG, "read_time: %02x,%02x,%02x,%02x,%02x,%02x,%02x", 
        rt->seconds, rt->minutes, rt->hours, rt->wday, rt->day, rt->month, rt->year);

    return result;
}

esp_err_t RTC::write_time(rtc_time_t& rt) const 
{
    ESP_LOGD(TAG, "write_time: %02x,%02x,%02x,%02x,%02x,%02x,%02x", 
        rt.seconds, rt.minutes, rt.hours, rt.wday, rt.day, rt.month, rt.year);
         
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SECONDS_REG, true); 
    i2c_master_write(cmd, reinterpret_cast<uint8_t*>(&rt), 7, true);
    i2c_master_stop(cmd);
    auto result = i2c_master_cmd_begin(m_port, cmd, I2C_TIMEOUT);
    i2c_cmd_link_delete(cmd);
    return result;
}

#if 0
static void log_time(struct tm &dt) {
    ESP_LOGD(TAG, "tm_sec:   %d", dt.tm_sec);
    ESP_LOGD(TAG, "tm_min:   %d", dt.tm_min);
    ESP_LOGD(TAG, "tm_hour:  %d", dt.tm_hour);
    ESP_LOGD(TAG, "tm_mday:  %d", dt.tm_mday);
    ESP_LOGD(TAG, "tm_mon:   %d", dt.tm_mon);
    ESP_LOGD(TAG, "tm_year:  %d", dt.tm_year);
    ESP_LOGD(TAG, "tm_wday:  %d", dt.tm_wday);
    ESP_LOGD(TAG, "tm_yday:  %d", dt.tm_yday);
    ESP_LOGD(TAG, "tm_isdst: %d", dt.tm_isdst);
}

/**
 * @brief Compenstate for drift error by adjusting the rate using the aging register.
 * @param drift The error accumulated over the time interval in seconds (-ve: fast, +ve: slow). 
 * NTP timestamp format with implied decimal point between bits 31 and 32.
 * @param interval The time interval over which the drift was measured (milliseconds).
 * 
 * @return
 *  - ESP_OK: success.
 *  - other: I2C communication failed.
 */
esp_err_t compensate_drift(int64_t drift, uint32_t invterval) const;

esp_err_t RTC::compensate_drift(int64_t drift, uint32_t interval) const
{
    double d = double(drift)/(1ULL<<32);
    double ppm = (d * 1000000000.0)/interval;
    ESP_LOGI(TAG, "compensate_drift: drift=%.6fs, interval=%ums, ppm=%.3f", d, interval, ppm);
    
    int32_t comp = std::lround(ppm / 0.1);
    if (comp == 0) {
        ESP_LOGI(TAG, "compensate_drift: no change necessary");
        return ESP_OK;
    }
    // Limit the compensation change to about 1ppm
    if (comp < -10) {
        comp = -10;
    } else if (comp > 10) {
        comp = 10;
    }
 
    uint8_t b;
    auto err = read_register(RTC::AGING_REG, &b);
    if (err != ESP_OK) {
        return err;
    }
    int32_t aging = int8_t(b) + comp;
    if (aging < -128) {
        aging = -128;
    } else if (aging > 127) {
        aging = 127;
    }
    ESP_LOGI(TAG, "compensate_drift: aging update: %+d%+d => %+d", int8_t(b), comp, aging);

    // Write new value to the aging register
    err = write_register(RTC::AGING_REG, uint8_t(aging));
    if (err != ESP_OK) {
        return err;
    }

    // Run TCXO algorithm and compute new capacitance value
    return force_temp_conv();
}
#endif