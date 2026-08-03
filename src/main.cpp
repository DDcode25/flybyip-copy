#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>

extern "C" {
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_netif_glue.h"
#include "esp_eth_phy.h"
#include "esp_eth_phy_lan87xx.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sys/time.h"
#include "unistd.h"
}

namespace {
constexpr char TAG[] = "OpenFlyIP";
constexpr gpio_num_t ETH_PHY_POWER = GPIO_NUM_16;
constexpr gpio_num_t ETH_MDC = GPIO_NUM_23;
constexpr gpio_num_t ETH_MDIO = GPIO_NUM_18;
constexpr int ETH_PHY_ADDR = 1;
constexpr int ETH_RMII_CLK_IN = 0;  // LAN8720 feeds the 50 MHz RMII clock into GPIO0
constexpr gpio_num_t CRSF_PIN = GPIO_NUM_5;
constexpr gpio_num_t MAV_RX_PIN = GPIO_NUM_35;
constexpr gpio_num_t MAV_TX_PIN = GPIO_NUM_17;
constexpr uart_port_t CRSF_UART = UART_NUM_1;
constexpr uart_port_t MAV_UART = UART_NUM_2;
constexpr size_t CRSF_MAX_FRAME = 64;
constexpr size_t MAV_BUFFER = 512;

struct Config {
    bool dhcp = true;
    char local_ip[16] = "192.168.13.10";
    char gateway[16] = "192.168.13.1";
    char netmask[16] = "255.255.255.0";
    char peer_ip[16] = "192.168.13.11";
    uint16_t crsf_local_port = 1313;
    uint16_t crsf_remote_port = 1313;
    uint32_t crsf_baud = 420000;
    uint16_t crsf_turnaround_us = 80;
    bool rewrite_ee_to_c8 = true;
    uint16_t mav_local_port = 14550;
    uint16_t mav_remote_port = 14550;
    uint32_t mav_baud = 115200;
};

struct Counters {
    std::atomic<uint32_t> crsf_uart_frames{0};
    std::atomic<uint32_t> crsf_udp_frames{0};
    std::atomic<uint32_t> crsf_crc_errors{0};
    std::atomic<uint32_t> crsf_drops{0};
    std::atomic<uint32_t> mav_uart_bytes{0};
    std::atomic<uint32_t> mav_udp_bytes{0};
};

Config cfg;
Counters stats;
std::atomic<bool> eth_connected{false};
int crsf_socket = -1;
int mav_socket = -1;
httpd_handle_t http_server = nullptr;
esp_netif_t *eth_netif = nullptr;
esp_eth_handle_t eth_handle = nullptr;
esp_eth_netif_glue_handle_t eth_glue = nullptr;

uint8_t crc8_dvb_s2(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0xD5U)
                                : static_cast<uint8_t>(crc << 1U);
        }
    }
    return crc;
}

void nvs_get_string(nvs_handle_t handle, const char *key, char *dst, size_t dst_len) {
    size_t required = 0;
    if (nvs_get_str(handle, key, nullptr, &required) != ESP_OK || required == 0 || required > dst_len) return;
    (void)nvs_get_str(handle, key, dst, &required);
}

void load_config() {
    nvs_handle_t handle;
    if (nvs_open("openflyip", NVS_READONLY, &handle) != ESP_OK) return;

    uint8_t value8 = 0;
    if (nvs_get_u8(handle, "dhcp", &value8) == ESP_OK) cfg.dhcp = value8 != 0;
    if (nvs_get_u8(handle, "rewrite", &value8) == ESP_OK) cfg.rewrite_ee_to_c8 = value8 != 0;
    nvs_get_string(handle, "local", cfg.local_ip, sizeof(cfg.local_ip));
    nvs_get_string(handle, "gateway", cfg.gateway, sizeof(cfg.gateway));
    nvs_get_string(handle, "netmask", cfg.netmask, sizeof(cfg.netmask));
    nvs_get_string(handle, "peer", cfg.peer_ip, sizeof(cfg.peer_ip));
    (void)nvs_get_u16(handle, "clp", &cfg.crsf_local_port);
    (void)nvs_get_u16(handle, "crp", &cfg.crsf_remote_port);
    (void)nvs_get_u32(handle, "cbaud", &cfg.crsf_baud);
    (void)nvs_get_u16(handle, "cturn", &cfg.crsf_turnaround_us);
    (void)nvs_get_u16(handle, "mlp", &cfg.mav_local_port);
    (void)nvs_get_u16(handle, "mrp", &cfg.mav_remote_port);
    (void)nvs_get_u32(handle, "mbaud", &cfg.mav_baud);
    nvs_close(handle);
}

int make_udp_socket(uint16_t port) {
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket failed: errno=%d", errno);
        return -1;
    }

    int reuse = 1;
    (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind(%u) failed: errno=%d", static_cast<unsigned>(port), errno);
        close(sock);
        return -1;
    }

    timeval timeout{};
    timeout.tv_usec = 1000;
    (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return sock;
}

bool send_udp(int sock, const uint8_t *data, size_t len, uint16_t port) {
    if (sock < 0 || data == nullptr || len == 0) return false;
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(port);
    if (inet_pton(AF_INET, cfg.peer_ip, &peer.sin_addr) != 1) return false;
    return sendto(sock, data, len, 0, reinterpret_cast<sockaddr *>(&peer), sizeof(peer)) == static_cast<int>(len);
}

void init_uart() {
    uart_config_t crsf{};
    crsf.baud_rate = static_cast<int>(cfg.crsf_baud);
    crsf.data_bits = UART_DATA_8_BITS;
    crsf.parity = UART_PARITY_DISABLE;
    crsf.stop_bits = UART_STOP_BITS_1;
    crsf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    crsf.rx_flow_ctrl_thresh = 0;
    crsf.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(CRSF_UART, 2048, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(CRSF_UART, &crsf));
    ESP_ERROR_CHECK(uart_set_pin(CRSF_UART, CRSF_PIN, CRSF_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(gpio_set_direction(CRSF_PIN, GPIO_MODE_INPUT));

    uart_config_t mav = crsf;
    mav.baud_rate = static_cast<int>(cfg.mav_baud);
    ESP_ERROR_CHECK(uart_driver_install(MAV_UART, 4096, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART, &mav));
    ESP_ERROR_CHECK(uart_set_pin(MAV_UART, MAV_TX_PIN, MAV_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void crsf_write_single_wire(const uint8_t *data, size_t len) {
    (void)uart_flush_input(CRSF_UART);
    ESP_ERROR_CHECK(gpio_set_direction(CRSF_PIN, GPIO_MODE_OUTPUT));
    const int written = uart_write_bytes(CRSF_UART, data, len);
    if (written > 0) (void)uart_wait_tx_done(CRSF_UART, pdMS_TO_TICKS(20));
    esp_rom_delay_us(cfg.crsf_turnaround_us);
    ESP_ERROR_CHECK(gpio_set_direction(CRSF_PIN, GPIO_MODE_INPUT));
    (void)uart_flush_input(CRSF_UART);
}

void crsf_task(void *) {
    uint8_t frame[CRSF_MAX_FRAME]{};
    uint8_t rx[CRSF_MAX_FRAME]{};
    size_t pos = 0;
    size_t expected = 0;

    while (true) {
        const int count = uart_read_bytes(CRSF_UART, rx, sizeof(rx), pdMS_TO_TICKS(2));
        for (int i = 0; i < count; ++i) {
            const uint8_t byte = rx[i];
            if (pos == 0) {
                frame[pos++] = byte;
                continue;
            }
            if (pos == 1) {
                frame[pos++] = byte;
                expected = static_cast<size_t>(byte) + 2U;
                if (expected < 4U || expected > CRSF_MAX_FRAME) {
                    pos = expected = 0;
                    ++stats.crsf_drops;
                }
                continue;
            }
            if (pos >= sizeof(frame)) {
                pos = expected = 0;
                ++stats.crsf_drops;
                continue;
            }
            frame[pos++] = byte;
            if (expected != 0U && pos == expected) {
                if (crc8_dvb_s2(frame + 2, expected - 3U) == frame[expected - 1U]) {
                    if (eth_connected && crsf_socket >= 0) (void)send_udp(crsf_socket, frame, expected, cfg.crsf_remote_port);
                    ++stats.crsf_uart_frames;
                } else {
                    ++stats.crsf_crc_errors;
                }
                pos = expected = 0;
            }
        }

        if (crsf_socket >= 0) {
            sockaddr_in source{};
            socklen_t source_len = sizeof(source);
            const int received = recvfrom(crsf_socket, frame, sizeof(frame), 0,
                                          reinterpret_cast<sockaddr *>(&source), &source_len);
            if (received >= 4 && static_cast<size_t>(received) <= sizeof(frame) &&
                static_cast<int>(frame[1]) + 2 == received &&
                crc8_dvb_s2(frame + 2, static_cast<size_t>(received) - 3U) == frame[received - 1]) {
                if (cfg.rewrite_ee_to_c8 && frame[0] == 0xEE) frame[0] = 0xC8;
                crsf_write_single_wire(frame, static_cast<size_t>(received));
                ++stats.crsf_udp_frames;
            } else if (received > 0) {
                ++stats.crsf_drops;
            }
        }
    }
}

void mavlink_task(void *) {
    uint8_t buffer[MAV_BUFFER]{};
    while (true) {
        const int count = uart_read_bytes(MAV_UART, buffer, sizeof(buffer), pdMS_TO_TICKS(2));
        if (count > 0) {
            stats.mav_uart_bytes += static_cast<uint32_t>(count);
            if (eth_connected && mav_socket >= 0) (void)send_udp(mav_socket, buffer, static_cast<size_t>(count), cfg.mav_remote_port);
        }
        if (mav_socket >= 0) {
            sockaddr_in source{};
            socklen_t source_len = sizeof(source);
            const int received = recvfrom(mav_socket, buffer, sizeof(buffer), 0,
                                          reinterpret_cast<sockaddr *>(&source), &source_len);
            if (received > 0) {
                const int written = uart_write_bytes(MAV_UART, buffer, received);
                if (written > 0) stats.mav_udp_bytes += static_cast<uint32_t>(written);
            }
        }
    }
}

esp_err_t status_handler(httpd_req_t *req) {
    char ip[16] = "0.0.0.0";
    if (eth_netif != nullptr) {
        esp_netif_ip_info_t info{};
        if (esp_netif_get_ip_info(eth_netif, &info) == ESP_OK) snprintf(ip, sizeof(ip), IPSTR, IP2STR(&info.ip));
    }
    char json[384]{};
    snprintf(json, sizeof(json),
             "{\"ethernet\":%s,\"ip\":\"%s\",\"crsf_uart_frames\":%lu,\"crsf_udp_frames\":%lu,\"crsf_crc_errors\":%lu,\"crsf_drops\":%lu,\"mav_uart_bytes\":%lu,\"mav_udp_bytes\":%lu}",
             eth_connected ? "true" : "false", ip,
             static_cast<unsigned long>(stats.crsf_uart_frames), static_cast<unsigned long>(stats.crsf_udp_frames),
             static_cast<unsigned long>(stats.crsf_crc_errors), static_cast<unsigned long>(stats.crsf_drops),
             static_cast<unsigned long>(stats.mav_uart_bytes), static_cast<unsigned long>(stats.mav_udp_bytes));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

esp_err_t root_handler(httpd_req_t *req) {
    static constexpr char HTML[] =
        "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>OpenFlyIP</title></head><body><h1>OpenFlyIP ESP-IDF</h1>"
        "<p>WT32-ETH01 CRSF single-wire + MAVLink UDP bridge.</p><p><a href='/status'>JSON status</a></p></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, HTML, HTTPD_RESP_USE_STRLEN);
}

void start_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&http_server, &config) != ESP_OK) return;

    httpd_uri_t root{};
    root.uri = "/";
    root.method = HTTP_GET;
    root.handler = root_handler;

    httpd_uri_t status{};
    status.uri = "/status";
    status.method = HTTP_GET;
    status.handler = status_handler;

    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server, &status));
}

void got_ip_handler(void *, esp_event_base_t, int32_t, void *event_data) {
    const auto *event = static_cast<ip_event_got_ip_t *>(event_data);
    eth_connected = true;
    if (crsf_socket < 0) crsf_socket = make_udp_socket(cfg.crsf_local_port);
    if (mav_socket < 0) mav_socket = make_udp_socket(cfg.mav_local_port);
    ESP_LOGI(TAG, "Ethernet IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

void eth_event_handler(void *, esp_event_base_t, int32_t event_id, void *) {
    if (event_id == ETHERNET_EVENT_DISCONNECTED || event_id == ETHERNET_EVENT_STOP) eth_connected = false;
}

bool parse_ipv4(const char *text, esp_ip4_addr_t *out) {
    if (text == nullptr || out == nullptr) return false;
    in_addr parsed{};
    if (inet_pton(AF_INET, text, &parsed) != 1) return false;
    out->addr = parsed.s_addr;
    return true;
}

void init_ethernet() {
    gpio_config_t power_config{};
    power_config.pin_bit_mask = 1ULL << static_cast<unsigned>(ETH_PHY_POWER);
    power_config.mode = GPIO_MODE_OUTPUT;
    power_config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&power_config));
    ESP_ERROR_CHECK(gpio_set_level(ETH_PHY_POWER, 1));
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_config);
    if (eth_netif == nullptr) abort();

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = ETH_MDC;
    emac_config.smi_gpio.mdio_num = ETH_MDIO;
    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = ETH_RMII_CLK_IN;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (mac == nullptr) abort();

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = -1;
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
    if (phy == nullptr) {
        mac->del(mac);
        abort();
    }

    esp_eth_config_t ethernet_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&ethernet_config, &eth_handle));

    eth_glue = esp_eth_new_netif_glue(eth_handle);
    if (eth_glue == nullptr) abort();
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, eth_glue));

    if (!cfg.dhcp) {
        const esp_err_t stopped = esp_netif_dhcpc_stop(eth_netif);
        if (stopped != ESP_OK && stopped != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) ESP_ERROR_CHECK(stopped);

        esp_netif_ip_info_t info{};
        if (!parse_ipv4(cfg.local_ip, &info.ip) || !parse_ipv4(cfg.gateway, &info.gw) ||
            !parse_ipv4(cfg.netmask, &info.netmask)) {
            ESP_LOGE(TAG, "Invalid static IPv4 configuration");
            abort();
        }
        ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &info));
    }

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, got_ip_handler, nullptr));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}
}  // namespace

extern "C" void app_main(void) {
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    load_config();
    init_uart();
    init_ethernet();
    start_http_server();

    if (xTaskCreatePinnedToCore(crsf_task, "crsf", 4096, nullptr, 12, nullptr, 1) != pdPASS) abort();
    if (xTaskCreatePinnedToCore(mavlink_task, "mavlink", 4096, nullptr, 10, nullptr, 0) != pdPASS) abort();

    ESP_LOGI(TAG, "OpenFlyIP ESP-IDF started");
}
