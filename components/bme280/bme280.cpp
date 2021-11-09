#include <cmath>

#include "esp_log.h"

#include "bme280.hpp"

static const char *TAG = "BME280";

using namespace jhall;

// SPI bus commands
#define CMD_READ    1
#define CMD_WRITE   0

// Mask out most significant bit of register address
#define MASK_REG_ADDR(r)    ((r) & 0x7fULL)

// Register addresses
enum {
    CALIB_00_REG = 0x88,
    CALIB_25_REG = 0xA1,
    ID_REG = 0xD0,
    RESET_REG = 0xE0,
    CALIB_26_REG = 0xE1,
    CALIB_32_REG = 0xE7,
    CTRL_HUM_REG = 0xF2,
    STATUS_REG = 0xF3,
    CTRL_MEAS_REG = 0xF4,
    CONFIG_REG = 0xF5,
    PRESS_MSB_REG = 0xF7,
    PRESS_LSB_REG = 0xF8,
    PRESS_XLSB_REG = 0xF9,
    TEMP_MSB_REG = 0xFA,
    TEMP_LSB_REG = 0xFB,
    TEMP_XLSB_REG = 0xFC,
    HUM_MSB_REG = 0xFD,
    HUM_LSB_REG = 0xFE,
};

// Chip id; ID_REG[7:0]
#define CHIP_ID     0x60

// Mode; CTRL_MEAS_REG[1:0]
#define MODE_SLEEP  0x00
#define MODE_FORCED 0x01
#define MODE_NORMAL 0x03

// Oversampling; 
// temp:  CTRL_MEAS_REG[7:5]
// press: CTRL_MEAS_REG[4:2]
// hum:   CTRL_HUM[2:0]
#define OVERSAMPLING_SKIP   0x00
#define OVERSAMPLING_1      0x01
#define OVERSAMPLING_2      0x02
#define OVERSAMPLING_4      0x03
#define OVERSAMPLING_8      0x04
#define OVERSAMPLING_16     0x05

// Filter; CONFIG_REG[4:2]
#define FILTER_OFF  0x00
#define FILTER_2    0x01
#define FILTER_4    0x02
#define FILTER_8    0x03
#define FILTER_16   0x04

// Standby in normal mode; CONFIG_REG[7:5]
#define STANDBY_0_5_MS  0x00
#define STANDBY_62_5_MS 0x01
#define STANDBY_125_MS  0x02
#define STANDBY_250_MS  0x03
#define STANDBY_500_MS  0x04
#define STANDBY_1000_MS 0x05
#define STANDBY_10_MS   0x06
#define STANDBY_20_MS   0x07

// Semaphore locking/unlocking helper
class Lock {
public:
    Lock(SemaphoreHandle_t &mutex)
        : m_mutex(mutex)
    {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
    }
    ~Lock()
    {
        xSemaphoreGive(m_mutex);
    }
private:
    SemaphoreHandle_t &m_mutex;
};

BME280::BME280(spi_host_device_t host_id, 
    gpio_num_t mosi, gpio_num_t miso, gpio_num_t sclk, gpio_num_t cs)
    : m_host_id(host_id)
    , m_free_bus(false)
    , m_data(nullptr)
    , m_mutex(xSemaphoreCreateMutex())
{
    if (m_mutex == nullptr) {
        ESP_LOGE(TAG, "cannot create mutex");
        abort();
    }

    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        mosi, // mosi_io_num
        miso, // miso_io_num
        sclk, // sclk_io_num
        -1, // quadwp_io_num
        -1, // quadhd_io_num
        0, // max_transfer_sz
        SPICOMMON_BUSFLAG_MASTER, // flags
        0, // intr_flags
    };
    ESP_ERROR_CHECK(spi_bus_initialize(m_host_id, &bus_cfg, SPI_DMA_CH_AUTO));
    m_free_bus = true;

    // Add device to SPI bus
    spi_device_interface_config_t dev_cfg = {
        1, // command_bits; 0=write, 1=read
        7, // address_bits; low 7 bits of register address
        0, // dummy_bits
        0, // mode
        0, // duty_cycle_pos
        0, // cs_ena_pretrans
        0, // cs_ena_posttrans
        8000000, // clock_speed_hz
        0, // input_delay_ns
        cs, // spics_in_num
        0, // flags
        1, // queue_size
        nullptr,    // pre_cb
        nullptr,    // post_cb
    };
    ESP_ERROR_CHECK(spi_bus_add_device(m_host_id, &dev_cfg, &m_dev_handle));

    // Check chip identity
    uint8_t value;
    ESP_ERROR_CHECK(read_register(ID_REG, &value));
    if (value != CHIP_ID) {
        ESP_LOGE(TAG, "invalid id register=%02x", value);
        abort();
    }

    // Allocate SPI rx buffer
    m_data = static_cast<uint8_t*>(heap_caps_malloc(CALIB_25_REG - CALIB_00_REG + 1, 
        MALLOC_CAP_DMA | MALLOC_CAP_32BIT));

    ESP_ERROR_CHECK(read_calib_params());

    // Configure weather monitoring mode (from datasheet), sampling interval set
    // from start() interval parameter
    ESP_ERROR_CHECK(write_register(CONFIG_REG, 0));
    ESP_ERROR_CHECK(write_register(CTRL_HUM_REG, OVERSAMPLING_1));
    // The final part of the configuration involves setting the CTRL_MEAS
    // register in read_sensors() to bring the device out of sleep state before
    // sampling the sensors
}

BME280::~BME280()
{
    stop();
    if (m_dev_handle != nullptr) {
        spi_bus_remove_device(m_dev_handle);
    }
    if (m_free_bus) {
        spi_bus_free(m_host_id);
    }
    if (m_data != nullptr) {
        heap_caps_free(m_data);
    }
    vSemaphoreDelete(m_mutex);
}

void BME280::start(uint32_t interval)
{
    m_sampling_delay = interval;

    auto result = xTaskCreatePinnedToCore(sampling_task,
        "sampling-task",
        2200,
        this,
        2,
        &m_sampling_task_handle,
        1   // APP CPU
    );
    if (result != pdPASS) {
        abort();
    }
}

// Sampling task handler.
void BME280::sampling_task(void *arg) 
{
	auto self = static_cast<BME280*>(arg);
	for (;;) {
        self->read_sensors();

		// Wait until next sampling time
		vTaskDelay((self->m_sampling_delay * 1000) / portTICK_PERIOD_MS);
    }
}

void BME280::stop()
{
	if (m_sampling_task_handle != nullptr) {
		vTaskDelete(m_sampling_task_handle);
		m_sampling_task_handle = nullptr;
	}
}

// Read device register.
esp_err_t BME280::read_register(int reg, uint8_t *value)
{
    spi_transaction_t t = {
        SPI_TRANS_USE_RXDATA, // flags
        CMD_READ, // cmd
        MASK_REG_ADDR(reg), // addr
        8, // length
        0, // rxlength
        nullptr, // user
        nullptr, // tx_buffer
        nullptr, // rx_buffer
    };
    auto err = spi_device_transmit(m_dev_handle, &t);
    *value = t.rx_data[0];
    ESP_LOGD(TAG, "read_register: [%02x]->%02x", reg, *value);
    return err;
}

// Write device register.
esp_err_t BME280::write_register(int reg, uint8_t value) 
{
    ESP_LOGD(TAG, "write_register: [%02x]<-%02x", reg, value);
    spi_transaction_t t = {
        SPI_TRANS_USE_TXDATA, // flags
        CMD_WRITE, // cmd
        MASK_REG_ADDR(reg), // addr
        8, // length
        0, // rxlength
        nullptr, // user
        nullptr, // tx_buffer
        nullptr, // rx_buffer
    };
    t.tx_data[0] = value;
    return spi_device_transmit(m_dev_handle, &t);
}

// Read contiguous range of device registers into rx buffer.
esp_err_t BME280::read_register_range(int start, int end)
{
   spi_transaction_t t = {
        0, // flags
        CMD_READ, // cmd
        MASK_REG_ADDR(start), // addr
        size_t(end - start + 1)*8, // length
        0, // rxlength
        nullptr, // user
        nullptr, // tx_buffer
        m_data, // rx_buffer
    };
    return spi_device_transmit(m_dev_handle, &t);
}

// Read calibration parameters and save them as member variables.
esp_err_t BME280::read_calib_params()
{
    // Read and save calibration bytes 00-25
    auto err = read_register_range(CALIB_00_REG, CALIB_25_REG);
    if (err != ESP_OK) {
        return err;
    }
    // The ESP32 and BME280 are both little-endian devices so we can cheat...
    dig_T1 = *reinterpret_cast<uint16_t*>(&m_data[0]);  // 0x88 / 0x89
    dig_T2 = *reinterpret_cast<int16_t*>(&m_data[2]);   // 0x8A / 0x8B
    dig_T3 = *reinterpret_cast<int16_t*>(&m_data[4]);   // 0x8C / 0x8D
    dig_P1 = *reinterpret_cast<uint16_t*>(&m_data[6]);  // 0x8E / 0x8F
    dig_P2 = *reinterpret_cast<int16_t*>(&m_data[8]);   // 0x90 / 0x91
    dig_P3 = *reinterpret_cast<int16_t*>(&m_data[10]);  // 0x92 / 0x93
    dig_P4 = *reinterpret_cast<int16_t*>(&m_data[12]);  // 0x94 / 0x95
    dig_P5 = *reinterpret_cast<int16_t*>(&m_data[14]);  // 0x96 / 0x97
    dig_P6 = *reinterpret_cast<int16_t*>(&m_data[16]);  // 0x98 / 0x99
    dig_P7 = *reinterpret_cast<int16_t*>(&m_data[18]);  // 0x9A / 0x9B
    dig_P8 = *reinterpret_cast<int16_t*>(&m_data[20]);  // 0x9C / 0x9D
    dig_P9 = *reinterpret_cast<int16_t*>(&m_data[22]);  // 0x9E / 0x9F
                                                        // 0xA0 (padding)
    dig_H1 = m_data[24];                                // 0xA1

    // Read and save calibration bytes 26-32
    err = read_register_range(CALIB_26_REG, CALIB_32_REG);
    if (err != ESP_OK) {
        return err;
    }
    dig_H2 = *reinterpret_cast<int16_t*>(&m_data[0]);   // 0xE1 / 0xE2
    dig_H3 = m_data[2];                                 // 0xE3
    dig_H4 = (int16_t(m_data[3])<<4) | (m_data[4]&0x0f);// 0xE4 / 0xE5
    dig_H5 = (int16_t(m_data[5])<<4) | (m_data[4]>>4);  // 0xE5 / 0xE6
    dig_H6 = int8_t(m_data[6]);                         // 0xE7

    #if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
		log_calib_params();
    #endif

    return ESP_OK;
}

double BME280::get_temperature()
{
    Lock lock(m_mutex);
    return m_temp / 100.0;
}

// Adjust local pressure p (Pa) at elevation h (m) to sea level. See:
// https://en.wikipedia.org/wiki/Barometric_formula. This formula is used with a
// fixed temperature value of 15 degress Celcius thus matching the "A"
// (Altimeter) field reported by aviation METARs. Thus, the adjusted pressure
// may be checked against METARs for a local airport.
double BME280::get_pressure(uint32_t h)
{  
    const double Tb = 273.15 + 15.0;    // Reference temperature (ICAO) (K)
    const double Lb = -0.00650;         // Laspe rate (ICAO) (K/m)
    const double hb = 0.0;              // Reference height (i.e. sea mean sea level) (m)
    const double R = 8.3144598;         // Universal gas constant (J/(mol.K))
    const double g0 = 9.80665;          // Gravitational acceleration (m/s^2)
    const double M = 0.0289644;         // Molar mass air (Kg/mol)

    Lock lock(m_mutex);
    double p = m_press / double(1<<8);  // Local pressure
    return p/std::pow(((Tb + Lb*(h - hb))/Tb), (-g0*M)/(R*Lb));
}

double BME280::get_humidity()
{
    Lock lock(m_mutex);
    return m_hum / double(1<<10);
}

// Read pressure, temperature, and humidity sensors.
void BME280::read_sensors()
{
    // Reset forced mode to bring device out of sleep state
    ESP_ERROR_CHECK(write_register(CTRL_MEAS_REG, 
        (OVERSAMPLING_1<<5) | (OVERSAMPLING_1<<2) | MODE_FORCED));

    // Read raw ADC data into buffer
    auto err = read_register_range(PRESS_MSB_REG, HUM_LSB_REG);
    {
        Lock lock(m_mutex);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "cannot read sensor");
            m_temp = 0;
            m_press = 0;
            m_hum = 0;
        } else {
            // Construct the raw ADC values
            int32_t up = (int32_t(m_data[0]) << 12) |    // 0xF7
                (int32_t(m_data[1]) << 4) |              // 0xF8
                (int32_t(m_data[2]) >> 4);               // 0xF9
                    
            int32_t ut = (int32_t(m_data[3]) << 12) |    // 0xFA
                (int32_t(m_data[4]) << 4) |              // 0xFB
                (int32_t(m_data[5]) >> 4);               // 0xFC

            int32_t uh = (int32_t(m_data[6]) << 8) |     // 0xFD
                int32_t(m_data[7]);                      // 0xFE

            // Apply compensation
            m_temp = BME280_compensate_T_int32(ut);
            m_press = BME280_compensate_P_int64(up);
            m_hum = BME280_compensate_H_int32(uh);
        }
    }
}

// The following code has been copied verbatim (with the exception of the class
// method qualifiers) from the BME280 Datasheet (code rev.1.1).

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as member variable
BME280_S32_t BME280::BME280_compensate_T_int32(BME280_S32_t adc_T) 
{
    BME280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11; 
    var2 = (((((adc_T>>4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) * ((BME280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine*5+128)>>8; 
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BME280_U32_t BME280::BME280_compensate_P_int64(BME280_S32_t adc_P)
{
    BME280_S64_t var1, var2, p;
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BME280_S64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (BME280_S64_t)dig_P3)>>8) + ((var1 * (BME280_S64_t)dig_P2)<<12); 
    var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)dig_P1)>>33; 
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BME280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25; 
    var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7)<<4); 
    return (BME280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t BME280::BME280_compensate_H_int32(BME280_S32_t adc_H)
{
    BME280_S32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r)) + 
        ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((BME280_S32_t)dig_H3)) >> 11) + 
        ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) * ((BME280_S32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r); 
    return (BME280_U32_t)(v_x1_u32r>>12);
}

#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
void BME280::log_calib_params()
{
    ESP_LOGD(TAG, "dig_T1=%d", dig_T1);
    ESP_LOGD(TAG, "dig_T2=%d", dig_T2);
    ESP_LOGD(TAG, "dig_T3=%d", dig_T3);

    ESP_LOGD(TAG, "dig_P1=%d", dig_P1);
    ESP_LOGD(TAG, "dig_P2=%d", dig_P2);
    ESP_LOGD(TAG, "dig_P3=%d", dig_P3);
    ESP_LOGD(TAG, "dig_P4=%d", dig_P4);
    ESP_LOGD(TAG, "dig_P5=%d", dig_P5);
    ESP_LOGD(TAG, "dig_P6=%d", dig_P6);
    ESP_LOGD(TAG, "dig_P7=%d", dig_P7);
    ESP_LOGD(TAG, "dig_P8=%d", dig_P8);
    ESP_LOGD(TAG, "dig_P9=%d", dig_P9);

    ESP_LOGD(TAG, "dig_H1=%d", dig_H1);
    ESP_LOGD(TAG, "dig_H2=%d", dig_H2);
    ESP_LOGD(TAG, "dig_H3=%d", dig_H3);
    ESP_LOGD(TAG, "dig_H4=%d", dig_H4);
    ESP_LOGD(TAG, "dig_H5=%d", dig_H5);
    ESP_LOGD(TAG, "dig_H6=%d", dig_H6);
}
#endif // LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG

#if 0
    // Set filtering and oversampling configuration (values from datasheet
    // "Indoor Navigation")
    ESP_ERROR_CHECK(write_register(CONFIG_REG, (STANDBY_1000_MS<<5) | (FILTER_16<<2)));
    ESP_ERROR_CHECK(write_register(CTRL_HUM_REG, OVERSAMPLING_1));
    ESP_ERROR_CHECK(write_register(CTRL_MEAS_REG, 
        (OVERSAMPLING_2<<5) | (OVERSAMPLING_16<<2) | MODE_NORMAL));
#endif