#ifndef BME280_HPP
#define BME280_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace jhall {

// Bosch code definitions
#define BME280_S32_t int32_t
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t

/**
 * @brief The BME280 class provides access to a BME280 combined temperature,
 * pressure, and humidity (TPH) sensor via the SPI bus.
 */
class BME280 {
public:
    /**
     * @brief Constructor.
     * @param host_id: The SPI controller.
     * @param mosi: The gpio pin connected to the MOSI line.
     * @param miso: The gpio pin connected to the MISO line.
     * @param sclk: The gpio pin connected to the SCLK line.
     * @param cs: The gpio pin connected to the CS line.
     */
    BME280(spi_host_device_t host_id, gpio_num_t mosi, gpio_num_t miso, 
        gpio_num_t sclk, gpio_num_t cs);

    /**
     * @brief Destructor.
     */
    virtual ~BME280();

    /**
     * @brief Start periodically sampling the BME280 sensors.
     * @param interval: The interval between sampling the sensors (seconds).
     */
    void start(uint32_t interval);

    /**
     * @brief Stop sampling the BME280 sensors.
     */
    void stop();

    /**
     * @brief Gets the last temperature sensor reading.
     * @return The temperature (C).
     */
    double get_temperature();

    /**
     * @brief Gets the last pressure sensor reading. If the height above sea
     * level is non-zero the result is adjusted to sea level.
     * @param h: height above sea level (m).
     * @return The pressure (Pa).
     */
    double get_pressure(uint32_t h);

    /**
     * @brief Gets the last humidity sensor reading.
     * @return The humidity (%RH).
     */
    double get_humidity();

private:
    static void sampling_task(void *arg);
    esp_err_t read_register(int reg, uint8_t *value);
    esp_err_t write_register(int reg, uint8_t value);
    esp_err_t read_register_range(int start, int end);
    esp_err_t read_calib_params();
    void log_calib_params();
    void read_sensors();
    BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T);
    BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P);
    BME280_U32_t BME280_compensate_H_int32(BME280_S32_t adc_H);

    TaskHandle_t m_sampling_task_handle;
    spi_host_device_t m_host_id;
    spi_device_handle_t m_dev_handle;
    bool m_free_bus;
    uint32_t m_sampling_delay;    // Time to wait before resampling sensors (secs)

    uint8_t *m_data;    // Data buffer

    SemaphoreHandle_t m_mutex;  // Guards the m_temp, m_press, and m_hum member variables
    BME280_S32_t m_temp;
    BME280_U32_t m_press;
    BME280_U32_t m_hum;

    BME280_S32_t t_fine;     // Internal temperature variable (was global in Bosch code)

    // Sensor calibration parameters
    uint16_t dig_T1;    // CALIB_00_REG
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;     // CALIB_25_REG
    int16_t dig_H2;     // CALIB_26_REG
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;      // CALIB_32_REG

}; // class BME280

} // namespace jhall

#endif // BME280_HPP