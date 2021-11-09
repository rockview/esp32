#include "esp_log.h"

#include "ticker.hpp"

//static const char *TAG = "Ticker";

using namespace jhall;

// --- Helper functions ---

static void tick_isr(void *task_handle) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// --- Class methods

Ticker::Ticker(gpio_num_t tick_pin)
    : m_tick_pin(tick_pin)
    , m_tick_cb(nullptr)
    , m_tick_arg(nullptr)
{
    gpio_config_t conf = {
        (1ULL<<m_tick_pin),     // pin_bit_mask
        GPIO_MODE_INPUT,        // mode
        GPIO_PULLUP_DISABLE,    // pull_up_en
        GPIO_PULLDOWN_DISABLE,  // pull_down_en
        GPIO_INTR_NEGEDGE,      // intr_type
    };
    ESP_ERROR_CHECK(gpio_config(&conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    auto result = xTaskCreatePinnedToCore(tick_task,
        "tick-task",
        2200,
        this,
        3,
        &m_tick_task_handle,
        1   // APP CPU
    );
    if (result != pdPASS) {
        abort();
    }
}

Ticker::~Ticker()
{
    stop();
        // TODO: remove tick_task ??
}

void Ticker::start(tick_cb_t cb, void *arg)
{
    m_tick_cb = cb;
    m_tick_arg = arg;
        
    ESP_ERROR_CHECK(gpio_isr_handler_add(m_tick_pin, tick_isr, m_tick_task_handle));
}

void Ticker::stop()
{
    ESP_ERROR_CHECK(gpio_isr_handler_remove(m_tick_pin));
}

void Ticker::tick_task(void *arg)
{
    auto self = static_cast<Ticker*>(arg);
    for (;;) {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
			continue;	// Zero notification!
		}
        
        if (self->m_tick_cb != nullptr) {
            self->m_tick_cb(self->m_tick_arg);
        }
    }
}