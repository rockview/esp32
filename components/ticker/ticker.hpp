#ifndef TICKER_HPP
#define TICKER_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

namespace jhall {

/**
 * @brief The Ticker class provides a callback mechanism that is triggered by
 * GPIO pin state changes.
 *
 * It is intended to be used with a GPIO pin connected to the DS3231 1Hz square
 * wave output, thus providing a callback at one second intervals.
 * */
class Ticker {
public:
    /**
     * @brief Constructor.
     * @param tick_pin The GPIO pin to monitor.
     */
    Ticker(gpio_num_t tick_pin);

    /**
     * @brief Destructor.
     */
    ~Ticker();

    /**
     * @brief Client callback function prototype.
     * @param arg The client argument provided to start().
     */
    typedef void (*tick_cb_t)(void *arg);

    /**
     * @brief Start monitoring the pin for changes.
     * @param cb Client callback function.
     * @param arg Client callback argument.
     */
    void start(tick_cb_t cb, void *arg);

    /**
     * @brief Stop monitoring the pin for changes.
     */
    void stop();

private:
    static void tick_task(void *arg);

    gpio_num_t m_tick_pin;
    tick_cb_t m_tick_cb;
    void *m_tick_arg;
    TaskHandle_t m_tick_task_handle;
};

} // namespace jhall

#endif // TICKER_HPP