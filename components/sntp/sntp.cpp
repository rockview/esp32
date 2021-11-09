#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/udp.h"
#include "lwip/api.h"

#include "sntp.hpp"

static const char *TAG = "SNTP";

using namespace jhall;

// Not all NTP timeservers perform equally well. The main problem appears to be
// asymmetric delays for the out and back legs of the packet exchange with the
// server. I implemented a simple heuristic to somewhat mitigate this issue but
// it's not an easy problem to solve with SNTP. The heuristic rejects all server
// roundtrips that exceed 100ms and forces a retry. That seems to work well in
// central Arizona but may not be optimal for other locations. A general
// observation is that the nearer the server, the better the performance,
// presumably because it involves fewer network hops that can bog down. A good
// NTP server can synchronize a client to within +/- 5ms. It is tempting to pick
// a server that performs well and stick with it, but that doesn't seem
// reasonable for an application that is expected to run for years unattended. A
// better approach is to pick a DNS name that maps to a pool of time servers
// that are resolved by DNS lookup on a round-robin basis. If the current server
// fails, and retries do not resolve the issue, another DNS lookup will yield a
// new server to try.
//
// The synchronization code is very careful to check for network errors and
// performs all the packet and message validation checks described in RFC 4330
// (12 checks total). If a synchronization fails for any reason it is retried
// with exponential backoff until either it succeeds or a retry delay threshold
// is exceeded, when another time server will be substituted.

// NTP message format
struct __attribute__((__packed__)) SNTP::ntp_msg {
	uint8_t li_vn_mode;
	uint8_t stratum;
	uint8_t poll;
	uint8_t precision;
	uint32_t root_delay;
	uint32_t root_dispersion;
	uint32_t reference_identifier;
	uint32_t reference_timestamp[2];
	uint32_t originate_timestamp[2];
	uint32_t receive_timestamp[2];
	uint32_t transmit_timestamp[2];
};
#define NTP_MSG_LEN sizeof(struct SNTP::ntp_msg)

#define LI_NO_WARNING 0
#define LI_LAST_MINUTE_61_SEC 1
#define LI_LAST_MINUTE_59_SEC 2
#define LI_ALARM_CONDITION 3

// Some servers are still at version 3, but we want to be able to use them so we
// set our version to 3 also
#define NTP_VERSION 3

#define MODE_CLIENT 3
#define MODE_SERVER 4

#define STRATUM_KOD  0
#define STRATUM_MAX  15

#define NTP_PORT 123
#define NTP_SYNC_TIMEOUT	(5*1000)	// Server timeout (5 seconds)

#define MIN_NTP_RETRY_DELAY	15	// secs (RFC 4330)
#define NTP_SECOND (1ULL<<32)	// Value of one second in an NTP timestamp

#define DEFAULT_SERVER_NAME		"time.apple.com"
#define DEFAULT_POLL_INTERVAL	(1*3600)	// 1 hour
#define DEFAULT_MAX_RETRY_DELAY	(10*60)		// 10 minutes

// Seconds from unix epoch (1970/01/01T00:00:00Z) to NTP era 1 (2036/02/07T06:28:16Z)
#define SECS_FROM_UNIX_EPOCH_TO_NTP_ERA_1 (2085978496L)

#define MAX_ROUNDTRIP_DELAY	0.100	// secs

SNTP::SNTP()
	: m_sync_cb(nullptr)
	, m_sync_arg(nullptr)
	, m_sync_task_handle(nullptr) 
	, m_sync_delay(0)
	, m_poll_interval(DEFAULT_POLL_INTERVAL)
	, m_max_retry_delay(DEFAULT_MAX_RETRY_DELAY)
	, m_server_name(DEFAULT_SERVER_NAME)
{
	static_assert(NTP_MSG_LEN == (4 + 11*4), "invalid size: struct ntp_msg");


	next_server();
}

SNTP::~SNTP() 
{
	stop();
}

void SNTP::set_ntp_server(std::string server_name)
{
	m_server_name = server_name;
}

void SNTP::set_poll_interval(uint32_t poll_interval)
{
	if (poll_interval < MIN_NTP_RETRY_DELAY) {
		m_poll_interval = MIN_NTP_RETRY_DELAY;
	} else {
		m_poll_interval = poll_interval;
	}
}

void SNTP::set_max_retry_delay(uint32_t max_retry_delay)
{
	m_max_retry_delay = max_retry_delay;
}

void SNTP::set_sync_cb(sync_cb_t cb, void *arg) 
{
	m_sync_cb = cb;
	m_sync_arg = arg;
}

esp_err_t SNTP::start() 
{
	m_sync_delay = MIN_NTP_RETRY_DELAY;
	m_retry_delay = MIN_NTP_RETRY_DELAY;

	// Create sync task
	auto result = xTaskCreatePinnedToCore(sync_task,
		"sync-task",
		2200,	// TODO: monitor stack size
		this,
		4,
		&m_sync_task_handle,
		1	// APP CPU
	);
	if (result != pdPASS) {
		return ESP_ERR_NO_MEM;
	}

	return ESP_OK;
}

void SNTP::stop() 
{
	if (m_sync_task_handle != nullptr) {
		vTaskDelete(m_sync_task_handle);
		m_sync_task_handle = nullptr;
	}
}

// Select another NTP server.
void SNTP::next_server()
{
	for (int i = 0; i < 4; i++) {
		if (try_server() == ESP_OK) {
			return;
		}
	}
	ESP_LOGE(TAG, "no valid NTP servers");
	abort();
}

// Resolve the server name to an IP address and check the server is functional.
// The intent is that the DNS server name that is chosen maps to a pool of time
// servers that are resolved by DNS lookup on a round-robin basis. Thus, if we
// encounter an error with a server, we can simply call this function to get a
// new one to try. These are suitable hostnames that may be used (although this
// is not an exhaustive list): time.apple.com, time.google.com, pool.ntp.org, and
// time.nist.gov.
esp_err_t SNTP::try_server()
{
	// Get IP address
	auto err = netconn_gethostbyname(m_server_name.c_str(), &m_server_addr);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "DNS lookup failed: %s (%s)", m_server_name.c_str(), 
			esp_err_to_name(err));
		return err;
	}

	// In order for this function to resolve to different time servers when
	// called repeatedly (see method comment above) it is important that the
	// prior DNS lookup is not cached, however that is not the case with the
	// LwIP DNS implementation. An easy way to overcome this issue would be to
	// flush the DNS cache between lookups, but no public API exists to do this.
	// Therefore, I define DNS_TABLE_SIZE as 1 in lwipopts.h (0 causes errors)
	// thus limiting the cache size to one record. Then it is a simple matter to
	// flush the cache by performing an unrelated DNS lookup (e.g. google.com)
	// after each call that resolves an NTP hostname. 
	ip_addr addr;
	netconn_gethostbyname("google.com", &addr);

	// Check server is valid
	struct ntp_msg msg;
	init_request(&msg);
	err = sync_with_server(msg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "bad server: %s %s (%s)", m_server_name.c_str(), 
			ip4addr_ntoa(ip_2_ip4(&m_server_addr)), esp_err_to_name(err));
		return err;
	}

	ESP_LOGI(TAG, "using server: %s %s", m_server_name.c_str(), 
		ip4addr_ntoa(ip_2_ip4(&m_server_addr)));

	return ESP_OK;
}

// Sync task handler.
void SNTP::sync_task(void *arg) 
{
	auto self = static_cast<SNTP*>(arg);
	for (;;) {
		// Wait until next sync time
		vTaskDelay((self->m_sync_delay * 1000) / portTICK_PERIOD_MS);
		ESP_LOGD(TAG, "sync with NTP server");

		struct ntp_msg msg;
		self->init_request(&msg);
		auto err = self->sync_with_server(msg);
		if (err == ESP_OK) {
			err = self->process_reply(msg);
		}
		if (err == ESP_OK) {
			// Sync succeeded; wait for next sync
			self->sync_success();
		} else {
			// Sync failed; backoff and retry
			self->sync_fail(err);
		}
	}
}

// Initialize NTP request packet.
void SNTP::init_request(struct ntp_msg *msg) 
{
	memset(msg, 0, NTP_MSG_LEN);

	msg->li_vn_mode = (LI_NO_WARNING<<6) | (NTP_VERSION<<3) | MODE_CLIENT;
	
	// Set transmit_timestamp from system time
	struct timeval now;
	gettimeofday(&now, nullptr);
	msg->transmit_timestamp[0] = htonl(now.tv_sec - SECS_FROM_UNIX_EPOCH_TO_NTP_ERA_1); 
	msg->transmit_timestamp[1] = htonl(usec_to_uint(now.tv_usec));
	
	// Save a copy for validation check
	m_last_sync_timestamp[0] = msg->transmit_timestamp[0];
	m_last_sync_timestamp[1] = msg->transmit_timestamp[1];
}

// Communicate with NTP server. Using the raw api would have been simpler but I
// could not find a way to set a timeout with it.
esp_err_t SNTP::sync_with_server(struct ntp_msg &msg)
{
	esp_err_t err;
	struct netconn *conn = nullptr;
	struct netbuf *send_buf = nullptr;
	struct netbuf *recv_buf = nullptr;

	conn = netconn_new(NETCONN_UDP);
	if (conn == nullptr) {
		return ESP_ERR_NO_MEM;
	}

	netconn_set_recvtimeout(conn, NTP_SYNC_TIMEOUT);
	err = netconn_connect(conn, &m_server_addr, NTP_PORT);
	if (err == ESP_OK) {
		send_buf = netbuf_new();
		if (send_buf == nullptr) {
			err = ESP_ERR_NO_MEM;
		} else {
			err = netbuf_ref(send_buf, &msg, NTP_MSG_LEN);
			if (err == ESP_OK) {
				err = netconn_sendto(conn, send_buf, &m_server_addr, NTP_PORT);
				if (err == ESP_OK) {
					err = netconn_recv(conn, &recv_buf);
					if (err == ESP_OK) {
						auto msg_len = netbuf_copy(recv_buf, &msg, NTP_MSG_LEN);
						err = validate_reply(msg_len, msg,
							&conn->pcb.udp->remote_ip, 
							conn->pcb.udp->remote_port);
					}
				}
			}
		}
	}
	
	if (err < 0) {
		// LwIP errors are in their own "namespace" so report them here. I would
		// have liked to print an error message using lwip_strerr() but I would
		// have to define the LWIP_DEBUG macro in order to use it and I didn't
		// want to do that. Instead, just print the error code. A full list of
		// error codes can be found in lwip/err.h, but the only one I have seen
		// during code development is -3 (Timeout).
		ESP_LOGE(TAG, "netconn error: %d", err);
	}

	// Tidy up
	netconn_close(conn);
	netbuf_delete(recv_buf);
	netbuf_delete(send_buf);
	netconn_delete(conn);

	return err;
}

// Validate server response (RFC 4330).
esp_err_t SNTP::validate_reply(uint16_t msg_len, const struct ntp_msg &msg,
	ip_addr_t *remote_addr, uint16_t remote_port) const
{
	// Validate packet
	if (!ip_addr_cmp(remote_addr, &m_server_addr)) {
		ESP_LOGE(TAG, "addr mismatch");
	} else if (remote_port != NTP_PORT) {
		ESP_LOGE(TAG, "port mismatch");
	} else if (msg_len != NTP_MSG_LEN) {
		ESP_LOGE(TAG, "bad message length");
	} else {
		#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
			log_msg(msg);
		#endif
		// Validate message
		uint8_t li = (msg.li_vn_mode>>6)&0x03;
		uint8_t version = (msg.li_vn_mode>>3)&0x07;
		uint8_t mode = msg.li_vn_mode&0x07;
		if (li == LI_ALARM_CONDITION) {
			ESP_LOGE(TAG, "server clock not synchronized");
		} else if (version != NTP_VERSION) {
			ESP_LOGE(TAG, "invalid server reply (vn): %d", version);
		} else if (mode != MODE_SERVER) {
			ESP_LOGE(TAG, "invalid server reply (mode): %d", mode);
		} else if (msg.stratum > STRATUM_MAX) {
			ESP_LOGE(TAG, "invalid server reply (stratum): %d", msg.stratum);
		} else if (msg.stratum == STRATUM_KOD) {
			auto ri = ntohl(msg.reference_identifier);
			auto tag = reinterpret_cast<char*>(&ri);
			ESP_LOGE(TAG, "kiss-of-death packet, code: %.4s", tag);
		} else if ((msg.originate_timestamp[0] != m_last_sync_timestamp[0]) || 
			(msg.originate_timestamp[1] != m_last_sync_timestamp[1])) {
			ESP_LOGE(TAG, "invalid server reply (non-matching originate_timestamp)");
		} else if ((msg.transmit_timestamp[0] == 0) && (msg.transmit_timestamp[1] == 0)) {
			ESP_LOGE(TAG, "invalid server reply (transmit_timestamp zero)");
		} else {
			return ESP_OK;
		}
	}
	return ESP_ERR_INVALID_RESPONSE;
}

// Process server reply message and update system time.
esp_err_t SNTP::process_reply(const struct ntp_msg &msg) const 
{
	ntp_t t = 0;	// Time offset (RFC 4330)
	ntp_t d = 0;	// Roundtrip delay (RFC 4330)
	ntp_t sync;		// New system time

	struct timeval now;
	gettimeofday(&now, nullptr);
	ntp_t t4 = unix_to_ntp(now);
	ntp_t t3 = make_ntp_time(ntohl(msg.transmit_timestamp[0]), ntohl(msg.transmit_timestamp[1]));

	// Check that the time difference between the client and server clocks is
	// small enough to avoid overflowing the offset calculation (~34 years).
	// This can occur at boot time when the system clock is initialized to 0
	// (1970-01-01T00:00:00Z) unless updated from battery-backed RTC
	ntp_t time_diff = (t4 < t3)? t3 - t4: t4 - t3;
	if ((uint64_t(time_diff)>>62) == 0) {
		// Calculate the system clock offset and roundtrip delay (RFC 4330)
		ntp_t t1 = make_ntp_time(ntohl(msg.originate_timestamp[0]), ntohl(msg.originate_timestamp[1]));
		ntp_t t2 = make_ntp_time(ntohl(msg.receive_timestamp[0]), ntohl(msg.receive_timestamp[1]));

		t = ((t2 - t1) + (t3 - t4))/2;
		d = (t4 - t1) - (t3 - t2);

		double r = double(d)/NTP_SECOND;
		if (std::abs(r) > MAX_ROUNDTRIP_DELAY) {
			// Sometimes an NTP servers take hundreds of milliseconds longer
			// than usual to respond. When this happens the offset calculation
			// often seems to be erroneous (probably due to asymmetric packet
			// delay). Thus, I ignore syncs that have a long roundtrip times and
			// force a retry
			ESP_LOGE(TAG, "roundtrip too long: %.3fs", r);
			return ESP_FAIL;
		}

		sync = t4 + t;
	} else {
		ESP_LOGI(TAG, "offset calculation overflowed, substituting server transmit time");
		sync = t3;
	}

	// Synchronize the system time with the server
	ntp_to_unix(sync, now);
	#if 0
	settimeofday(&now, nullptr);
	#else
	static bool first = true;
	if (first) {
		settimeofday(&now, nullptr);
		first = false;
	}
	#endif

	if (m_sync_cb != nullptr) {
		// Report synchronization to client
		m_sync_cb(now, t, d, m_sync_arg);
	}

	#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
		// Log synchronization
		log_sync_info(t4, sync, t, d);
	#endif

	return ESP_OK;
}

// Convert NTP timestamp to Unix epoch.
void SNTP::ntp_to_unix(ntp_t t, struct timeval &tv) const 
{
	ntp_to_timeval(t, tv);
	tv.tv_sec += SECS_FROM_UNIX_EPOCH_TO_NTP_ERA_1;
}

// Convert Unix timestamp to NTP era.
SNTP::ntp_t SNTP::unix_to_ntp(struct timeval &tv) const 
{
	return make_ntp_time(
		tv.tv_sec - SECS_FROM_UNIX_EPOCH_TO_NTP_ERA_1,
		usec_to_uint(tv.tv_usec)
	);
}

SNTP::ntp_t SNTP::make_ntp_time(uint32_t secs, uint32_t frac) const 
{
	return (ntp_t(secs)<<32) | frac;
}

// Convert microseconds value to 32-bit unsigned int.
uint32_t SNTP::usec_to_uint(uint32_t usec) const 
{
	return (usec*NTP_SECOND)/1000000;
}

// Convert 32-bit unsigned int to microseconds value.
uint32_t SNTP::uint_to_usec(uint32_t frac) const
{
	return (frac*1000000ULL)/NTP_SECOND;
}

void SNTP::ntp_to_timeval(ntp_t t, struct timeval &tv) const 
{
	tv.tv_sec = time_t(t>>32);
	tv.tv_usec = uint_to_usec(t&0xffffffff);
}

void SNTP::sync_success() 
{
	m_sync_delay = m_poll_interval;
	m_retry_delay = MIN_NTP_RETRY_DELAY;
}

void SNTP::sync_fail(esp_err_t err) 
{
	ESP_LOGE(TAG, "error: %s", esp_err_to_name(err));
	m_retry_delay *= 2; // Exponential backoff
	if (m_retry_delay > m_max_retry_delay) {
		ESP_LOGE(TAG, "max retries on %s; trying next server", m_server_name.c_str());
		next_server();
		m_sync_delay = 0;	// Try next server immediately
		m_retry_delay = MIN_NTP_RETRY_DELAY;	// Reset backoff
	} else {
		ESP_LOGE(TAG, "retrying in %us", m_retry_delay);
		m_sync_delay = m_retry_delay;
	}
}

#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
void SNTP::log_msg(const struct ntp_msg &msg) const 
{
	ESP_LOGD(TAG, "li_vn_mode:           %02x", msg.li_vn_mode);
	ESP_LOGD(TAG, "stratum:              %u", msg.stratum);
	ESP_LOGD(TAG, "poll:                 %u", msg.poll);
	ESP_LOGD(TAG, "root_delay:           %u", ntohl(msg.root_delay));
	ESP_LOGD(TAG, "root_dispersion:      %u", ntohl(msg.root_dispersion));
	ESP_LOGD(TAG, "reference_identifier: %u", ntohl(msg.reference_identifier));
	ESP_LOGD(TAG, "reference_timestamp:  %10u.%010u", 
		ntohl(msg.reference_timestamp[0]), ntohl(msg.reference_timestamp[1]));
	ESP_LOGD(TAG, "originate_timestamp:  %10u.%010u", 
		ntohl(msg.originate_timestamp[0]), ntohl(msg.originate_timestamp[1]));
	ESP_LOGD(TAG, "receive_timestamp:    %10u.%010u", 
		ntohl(msg.receive_timestamp[0]), ntohl(msg.receive_timestamp[1]));
	ESP_LOGD(TAG, "transmit_timestamp:   %10u.%010u", 
		ntohl(msg.transmit_timestamp[0]), ntohl(msg.transmit_timestamp[1]));
}

// Convert NTP timestamp to string for logging.
void SNTP::ntp_to_str(char *buf, size_t size, ntp_t t) const 
{
	char sign = '+';
	if (t < 0) {
		t = -t;
		sign = '-';
	}
	uint32_t secs = uint32_t(t>>32);
	uint32_t frac = uint32_t(t&0xffffffff);
	// A maximum of 33 chars (including trailing NUL) can be written to buf
	snprintf(buf, size, "%c%d.%010u(%c.%06us)",
		sign, secs, frac, sign, uint_to_usec(frac));
}

void SNTP::log_sync_info(ntp_t t4, ntp_t sync, ntp_t t, ntp_t d) const 
{
	// Log NTP time sync update
	char t_buf[40];
	ntp_to_str(t_buf, sizeof(t_buf), t);
	uint32_t t4_secs = uint32_t(t4>>32);
	uint32_t t4_frac = uint32_t(t4&0xffffffff);
	uint32_t sync_secs = uint32_t(sync>>32);
	uint32_t sync_frac = uint32_t(sync&0xffffffff);
	ESP_LOGD(TAG, "NTP %u.%010u => %u.%010u, t=%s", 
		t4_secs, t4_frac,
		sync_secs, sync_frac,
		t_buf);

	// Log other sync details
	char buf[30];
	time_t unix_secs = sync_secs + SECS_FROM_UNIX_EPOCH_TO_NTP_ERA_1;
	struct tm dt;
	gmtime_r(&unix_secs, &dt);
	auto len = strftime(buf, sizeof(buf), "%FT%T", &dt);
	snprintf(&buf[len], sizeof(buf) - len, ".%06u", uint_to_usec(sync_frac));
	char d_buf[40];
	ntp_to_str(d_buf, sizeof(d_buf), d);
	ESP_LOGD(TAG, "UTC %s %s %s, d=%s",
		buf, m_server_name.c_str(),
		ip4addr_ntoa(ip_2_ip4(&m_server_addr)), d_buf);
}
#endif // LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG

#if 0
double out = double(t2 - t1)/NTP_SECOND;
double ret = double(t4 - t3)/NTP_SECOND;
ESP_LOGI(TAG, "out=%.3fs, ret=%.3fs", out, ret);

if (std::abs(out - ret)/std::min(out, ret) > 0.50) {
	// When the outbound and return legs of an NTP request are
	// asymmetric the offset calculation is untrustworthy. The cause of
	// this seems to be a network delay, especially on the return path.
	// Thus, ignore asymmetric requests and force a retry
	ESP_LOGE(TAG, "NTP request asymmetry: out=%.3fs, ret=%.3f, retrying", out, ret);
	return ESP_FAIL;
}
#endif
