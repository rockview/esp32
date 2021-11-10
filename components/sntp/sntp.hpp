#ifndef SNTP_HPP
#define SNTP_HPP

#include <ctime>
#include <memory>

#include <lwip/ip_addr.h>
#include <lwip/pbuf.h>

namespace jhall {

/**
 * @brief The SNTP class supports time synchronization with an NTP time server.
 * 
 * The NTP time server is polled at fixed intervals and the system time is synchronized with NTP server time.
 */
class SNTP {
public:
    /**
     * @brief Constructor.
     *
     * The class constructor sets the following default values:
     *  - ntp_server: time.apple.com
     *  - poll_interval: 1 hour
     *  - max_retry_delay: 10 minutes 
     * These may be overriden by calling the appropriate setter method.
     */
    SNTP();
    /**
     * @brief Destructor.
     */
    virtual ~SNTP();

    /**
     * @brief Sets the NTP server name.
     * @param server_name The name of an NTP server from which to set the time.
     * Ideally, a pool of time servers that are resolved by DNS lookup on a
     * round-robin basis.
     */
    void set_ntp_server(std::string server_name);

    /**
     * @brief Sets the polling interval.
     * @param poll_interval How often to synchronize with the NTP server (seconds).
     */
    void set_poll_interval(uint32_t poll_interval);

    /**
     * @brief Sets the maximum retry delay.
     * @param max_retry_delay The maximum time to wait to retry a
     * failed synchronization (seconds). 
     */
    void set_max_retry_delay(uint32_t max_retry_delay);

    /**
     * @brief Client callback function prototype.
     * @param now The NTP-synchronized time.
     * @param offset The time offset computed by the time synchronization process for
     * the last sync (seconds). NTP timestamp format (32.32 fixed point).
     * @param roundtrip The roundtrip delay computed by the time sychronization process
     * for the last sync (seconds). NTP timestamp format (32.32 fixed point).
     * @param arg The client argument provided to set_sync_cb().
     */
    typedef void (*sync_cb_t)(const struct timeval &now, int64_t offset, int64_t roundtrip, void *arg);

    /**
     * @brief Set client callback function.
     * @param cb Client callback function.
     * @param arg Client callback argument.
     */
    void set_sync_cb(sync_cb_t cb, void *arg);

    /**
     * @brief Start NTP server synchronization.
     * @return -
     *  - ESP_OK: success.
     *  - ESP_ERR_INVALID_STATE: no server set via add_server().
     */
    esp_err_t start();

    /**
     * @brief Stop NTP server synchronization.
     */
    void stop();

private:
    struct ntp_msg;
    typedef int64_t ntp_t;  // NTP timevalue

    void next_server();
    esp_err_t try_server();
    static void sync_task(void *arg);
    void init_request(struct ntp_msg *msg);
    esp_err_t sync_with_server(struct ntp_msg &msg);
    esp_err_t validate_reply(uint16_t msg_len, const struct ntp_msg &msg,
        ip_addr_t *remote_addr, uint16_t remote_port) const;
    esp_err_t process_reply(const struct ntp_msg &msg) const;
    void ntp_to_unix(ntp_t t, struct timeval &tv) const;
    ntp_t unix_to_ntp(struct timeval &tv) const;
    ntp_t make_ntp_time(uint32_t secs, uint32_t frac) const;
    uint32_t usec_to_uint(uint32_t usec) const;
    uint32_t uint_to_usec(uint32_t frac) const;
    void ntp_to_timeval(ntp_t value, struct timeval &tv) const;
    void sync_success();
    void sync_fail(esp_err_t err);
    void log_msg(const struct ntp_msg &msg) const;
    void ntp_to_str(char *buf, size_t size, ntp_t x) const;
    void log_sync_info(ntp_t t4, ntp_t sync, ntp_t t, ntp_t d) const;

    sync_cb_t m_sync_cb;
    void *m_sync_arg;

    uint32_t m_last_sync_timestamp[2];  // Network byte order
    TaskHandle_t m_sync_task_handle;

    uint32_t m_sync_delay;      // Delay before next sync; reloaded from following two members
    uint32_t m_poll_interval;   // Normal: interval between syncs
    uint32_t m_retry_delay;     // Error: delay before next retry
    uint32_t m_max_retry_delay; // The maximum retry delay before trying another server

    std::string m_server_name;  // Hostname of current server
	ip_addr m_server_addr;      // IP address of current server
    
}; // class SNTP

} // namespace jhall

#endif // SNTP_HPP
