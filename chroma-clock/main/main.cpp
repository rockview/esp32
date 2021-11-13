#include <array>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "hal/spi_types.h"

#include "wifi.hpp"
#include "sntp.hpp"
#include "ticker.hpp"
#include "ds3231.hpp"
#include "hd44780.hpp"
#include "http.hpp"
#include "config.hpp"
#include "ws2812b.hpp"
#include "bme280.hpp"

static const char *TAG = "main";

using namespace jhall;

// Chroma-Clock
//
// Principal of Operation
//
// The clock uses a DS3231 real time clock (RTC) that keeps time with a
// temperature compensated crystal oscillator (TCXO) giving an accuracy better
// than 2ppm. That accuracy may be improved by making adjustments to an aging
// register that adjusts the frequency of crystal oscillation. The RTC is
// configured to generate a 32kHz signal that is used as an external clock
// source for the ESP32 and is much more accurate than the built-in RC
// oscillator. The DS3231 is also configured to generate a 1Hz signal that is
// connected to a GPIO pin of the ESP32 and generates an interrupt on its
// falling edge. This event causes the display to be updated with the system
// time once per second. When the clock first starts, it synchronizes its time
// with an NTP server by updating the system time and the DS3231. Subsequently,
// this synchronization is repeated at fixed intervals to keep the clock's time
// from drifting more than a few milliseconds. The DS3231 and the system keep
// UTC time in the Unix epoch (1970-1-1T00:00:00Z).

// Configuration
//
// The ESP32 needs to be configured to use the 32KHz external clock source from
// the DS3231 for its internal RTC by using the "SDK Configuration Editor":
//
// 1. Set "Timers used for gettimeofday function" to "RTC".
// 2. Set "RTC clock source" to "External 32kHz oscillator at 32K_XN pin".
// 3. Set "Number of cycles for RTC_SLOW_CLK calibration" to "0", i.e. no
//    calibration.
//
// Also, define DNS_TABLE_SIZE as 1 in lwipopts.h so the last cached DNS lookup
// may be flushed by a subsequent lookup.

// FreeRTOS Tasks
//
// There are 3 tasks in this application that run at these priorities on the
// APP CPU: 
//
// - 1 main (suspended after setup)
// - 2 BME280 sampling
// - 3 ticker
// - 4 sntp sync

// ESP-IDF includes an SNTP implementation that I was initially going to use,
// but it is highly configurable using #defines making it difficult to
// understand and also some combinations of these defines failed to compile
// which didn't inspire confidence, so I wrote my own. In any case, writing my
// own implementation gave me a much better understanding of how SNTP works.

// DS3231 RTC
#define SDA_PIN   GPIO_NUM_21
#define SCL_PIN   GPIO_NUM_22
#define TICK_PIN  GPIO_NUM_4   // 1 Hz square wave

// HD44780 LCD
#define RS_PIN  GPIO_NUM_13
#define E_PIN   GPIO_NUM_12
#define D4_PIN  GPIO_NUM_14
#define D5_PIN  GPIO_NUM_27
#define D6_PIN  GPIO_NUM_26
#define D7_PIN  GPIO_NUM_25

// BME280 TPH sensor
#define MISO_PIN  GPIO_NUM_23
#define MOSI_PIN  GPIO_NUM_19
#define SCLK_PIN  GPIO_NUM_18
#define CS_PIN    GPIO_NUM_5

// WS2812B LED strip
#define LED_PIN   GPIO_NUM_18

// Temporary string buffer size
#define BUF_SIZE  17

static std::shared_ptr<Config> cfg;
static std::shared_ptr<RTC> rtc;
static std::shared_ptr<LCD> lcd;
static std::shared_ptr<BME280> tph;

static void init_cfg() {
   cfg = std::make_shared<Config>();
   
   // TODO: remove this when using http form
   cfg->erase();

   std::string tmp;
   switch (cfg->get("ntp-server", tmp)) {
   case ESP_OK:
      break;
   case ESP_ERR_NVS_NOT_FOUND:
      // First boot, write default values
      //cfg->set("ntp-server", "pool.ntp.org");
      //cfg->set("ntp-server", "time.nist.gov");
      //cfg->set("ntp-server", "time.google.com");
      cfg->set("ntp-server", "time.apple.com");
      //cfg->set("poll-interval", 1*60); // 1 minute
      cfg->set("poll-interval", 1*3600); // 1 hour
      //cfg->set("poll-interval", 4*3600); // 4 hours
      //cfg->set("poll-interval", 8*3600); // 8 hours
      //cfg->set("poll-interval", 12*3600); // 12 hours
      //cfg->set("poll-interval", 24*3600); // 24 hours
      cfg->set("max-retry-delay", 10*60); // 10 minutes
      cfg->set("max-sync-error", 5);   // Milliseconds
      cfg->set("timezone", "MST+7");
      cfg->set("date-format", 2);   // 1: European, 2: American, 3: ISO
      cfg->set("temp-unit", 2);     // 1: Centigrade, 2: Farenheit
      cfg->set("press-unit", 2);    // 1: Pa, 2: mb, 3: mmHg, 4: inHg
      cfg->set("altitude", 1701);   // m

      cfg->set("ap-ssid", "ESP32 Clock");
      cfg->set("ap-pass", "time flies");

      // TODO: remove these when using http form
      cfg->set("sta-ssid", "1569 Standing Eagle Dr");
      cfg->set("sta-pass", "The Hall family wireless.");

      cfg->commit();
      break;
   default:
      abort();
   }
}

static void set_timezone() {
   std::string tz;
   ESP_ERROR_CHECK(cfg->get("timezone", tz));
   setenv("TZ", tz.c_str(), 1);
   tzset();
}

static void init_rtc() {
   rtc = std::make_shared<RTC>(I2C_NUM_0, SDA_PIN, SCL_PIN);

   // Initialize RTC to provide 1Hz and 32KHz square wave outputs
   ESP_ERROR_CHECK(rtc->write_register(RTC::CONTROL_REG, 0x00));
   ESP_ERROR_CHECK(rtc->write_register(RTC::STATUS_REG, RTC::STATUS_EN32KHZ));

   // Set aging offset
   // ESP_ERROR_CHECK(rtc->write_register(RTC::AGING_REG, uint8_t(-38)));
   ESP_ERROR_CHECK(rtc->force_temp_conv());

   // Log aging offset
   uint8_t value;
   ESP_ERROR_CHECK(rtc->read_register(RTC::AGING_REG, &value));
   ESP_LOGI(TAG, "aging offset: %d", int8_t(value));

   // Set system time from RTC. Since the RTC time registers only have a precision of 
   // one second this is provisional and will be corrected at the first one second tick
   // which has a precision of 1/32768 secs.
   time_t t = rtc->get_time();
   if (t == -1) {
      abort();
   }
   struct timeval tv = {t, 0};
   settimeofday(&tv, nullptr);
}

static void init_lcd()
{
   lcd = std::make_shared<LCD>(RS_PIN, E_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);
}

static void init_tph()
{
   tph = std::make_shared<BME280>(SPI3_HOST, MISO_PIN, MOSI_PIN, SCLK_PIN, CS_PIN);
   tph->start(13);
}

static void fill_date(char *buf, const struct tm *dt)
{
   uint32_t format;
   cfg->get("date-format", &format);
   switch (format) {
   case 1:  // European
      strftime(buf, BUF_SIZE, "%d-%m-%y", dt);
      break;
   case 2:  // American
      strftime(buf, BUF_SIZE, "%m-%d-%y", dt);
      break;
   case 3:  // ISO
      strftime(buf, BUF_SIZE, "%y-%m-%d", dt);
      break;
   }
}

static void fill_temp(char *buf)
{
   double c = tph->get_temperature();
   uint32_t unit;
   cfg->get("temp-unit", &unit);
   switch (unit) {
   case 1:  // Centigrade
      snprintf(buf, BUF_SIZE, "%.1f\xDF" "C", c);
      break;
   case 2:  // Farenheit
      snprintf(buf, BUF_SIZE, "%.1f\xDF" "F", c*9/5 + 32);
      break;
   }
}

static void fill_press(char *buf)
{
   uint32_t h;
   cfg->get("altitude", &h);
   double p = tph->get_pressure(h);
   uint32_t unit;
   cfg->get("press-unit", &unit);
   switch (unit) {
   case 1:  // Pa
      snprintf(buf, BUF_SIZE, "%.0fPa", p);
      break;
   case 2:  // hPa/mb
      snprintf(buf, BUF_SIZE, "%.1fmb", p/100.0);
      break;
   case 3:  // mmHg
      snprintf(buf, BUF_SIZE, "%.0fmmHg", p*0.00750062);
      break;
   case 4:  // inHg
      snprintf(buf, BUF_SIZE, "%.2finHg", p*0.0002953);
      break;
   }
}

static void fill_hum(char *buf)
{
   snprintf(buf, BUF_SIZE, "%.1f%%RH", tph->get_humidity());
}

// One second tick callback
static void tick_cb(void *arg) {
   // Get UTC system time
   struct timeval tv;
   gettimeofday(&tv, nullptr);

   time_t secs = tv.tv_sec;
   static bool first = true;
   if (first) {
      // Correct provisional time initially set from RTC
      secs++;
      tv = {secs, 0};
      settimeofday(&tv, nullptr);
      first = false;
   } else {
      // Round to nearest second
      if (tv.tv_usec >= 500000) {
         secs++;
      }
   }

   // Convert to local time
   struct tm dt;
   localtime_r(&secs, &dt);

   // Display time
   char buf[BUF_SIZE];
   strftime(buf, sizeof(buf), "%T", &dt);
   lcd->set_cursor(0, 0);
   lcd->print(buf);

   switch (secs % 28) {
      case 0:
         fill_date(buf, &dt);
         break;
      case 7:
         fill_temp(buf);
         break;
      case 14:
         fill_press(buf);
         break;
      case 21:
         fill_hum(buf);
         break;
      default:
         return;
   }

   lcd->set_cursor(1, 0);
   lcd->print("                ");
   lcd->set_cursor(1, 0);
   lcd->print(buf);
}

#if LOG_LOCAL_LEVEL >= ESP_LOG_INFO
// Log time synchronization stats.
static void log_sync(const struct timeval &now, int64_t offset, int64_t roundtrip, std::shared_ptr<RTC> rtc) {
   static time_t t = 0;

   if (t != 0) {
      // Compute time interval for all but first sync
      t = now.tv_sec - t;
   }

   double temp;
   rtc->read_temp(&temp);
   ESP_LOGI(TAG, "interval=%lds, offset=%.6fs, roundtrip=%.6fs, temp=%.2f°C",
      t, double(offset)/(1ULL<<32), double(roundtrip)/(1ULL<<32), temp);

   // Save time of this sync for computing next interval
   t = now.tv_sec;
}
#endif // LOG_LOCAL_LEVEL >= ESP_LOG_INFO

// NTP time sync callback
static void sync_cb(const struct timeval &now, int64_t offset, int64_t roundtrip, void *arg) {
   // Update RTC time registers at the start of next second
   time_t secs = now.tv_sec + 1;
   // I tried using vTaskDelay() here but it introduces too much jitter
   ets_delay_us(1000000 - now.tv_usec);
   auto err = rtc->set_time(secs, true);
   if (err != ESP_OK) {
      ESP_LOGE(TAG, "RTC sync failed: %s", esp_err_to_name(err));
   }

   #if LOG_LOCAL_LEVEL >= ESP_LOG_INFO
      log_sync(now, offset, roundtrip, rtc);
   #endif
}

static std::shared_ptr<SNTP> new_sntp()
{
   auto sntp = std::make_shared<SNTP>();

   std::string server_name;
   ESP_ERROR_CHECK(cfg->get("ntp-server", server_name));
   sntp->set_ntp_server(server_name);

   uint32_t value;
   ESP_ERROR_CHECK(cfg->get("poll-interval", &value));
   sntp->set_poll_interval(value);

	ESP_ERROR_CHECK(cfg->get("max-retry-delay", &value));
   sntp->set_max_retry_delay(value);

 	ESP_ERROR_CHECK(cfg->get("max-sync-error", &value));
   sntp->set_max_sync_error(value);

   sntp->set_sync_cb(sync_cb, nullptr);
   ESP_ERROR_CHECK(sntp->start());

   return sntp;
}

static std::shared_ptr<WiFi> new_wifi()
{
   auto wifi = std::make_shared<WiFi>();

   std::string ap_ssid;
   std::string ap_pass;
   ESP_ERROR_CHECK(cfg->get("ap-ssid", ap_ssid));
   ESP_ERROR_CHECK(cfg->get("ap-pass", ap_pass));
   wifi->set_ap_credentials(ap_ssid, ap_pass);

   std::string sta_ssid;
   std::string sta_pass;
   ESP_ERROR_CHECK(cfg->get("sta-ssid", sta_ssid));
   ESP_ERROR_CHECK(cfg->get("sta-pass", sta_pass));
   wifi->set_sta_credentials(sta_ssid, sta_pass);

   esp_log_level_set("wifi", ESP_LOG_WARN);
   ESP_ERROR_CHECK(wifi->start(WIFI_MODE_APSTA));

   return wifi;
}

static void test_ws2821b()
{
   WS2812B leds(RMT_CHANNEL_0, LED_PIN, 1);
   for (;;) {
      leds.set_pixel(0, 0, 255, 85);
      leds.show();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}

extern "C" void app_main() 
{
   init_cfg();

   set_timezone();

   init_rtc();
   init_lcd();
   init_tph();
 
   Ticker ticker(TICK_PIN);
   ticker.start(tick_cb, nullptr);

   auto wifi = new_wifi();
   auto sntp = new_sntp();
  
   HTTP http;
   http.start();

   //test_ws2821b();

   vTaskSuspend(nullptr);
}