#include <cstring>
#include <string>
#include <algorithm>

extern "C" {
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"
}

namespace {
constexpr char TAG[] = "OpenFlyIP";
constexpr gpio_num_t ETH_PHY_POWER = GPIO_NUM_16;
constexpr gpio_num_t ETH_MDC = GPIO_NUM_23;
constexpr gpio_num_t ETH_MDIO = GPIO_NUM_18;
constexpr int ETH_PHY_ADDR = 1;
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
    uint16_t mav_idle_gap_us = 1500;
};

struct Counters {
    uint32_t crsf_uart_frames = 0;
    uint32_t crsf_udp_frames = 0;
    uint32_t crsf_crc_errors = 0;
    uint32_t crsf_drops = 0;
    uint32_t mav_uart_bytes = 0;
    uint32_t mav_udp_bytes = 0;
};

Config cfg;
Counters stats;
volatile bool eth_connected = false;
int crsf_socket = -1;
int mav_socket = -1;
httpd_handle_t http_server = nullptr;
esp_netif_t *eth_netif = nullptr;

uint8_t crc8_dvb_s2(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}

void nvs_get_string(nvs_handle_t h, const char *key, char *dst, size_t dst_len) {
    size_t len = dst_len;
    nvs_get_str(h, key, dst, &len);
}

void load_config() {
    nvs_handle_t h;
    if (nvs_open("openflyip", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t b = 0;
    if (nvs_get_u8(h, "dhcp", &b) == ESP_OK) cfg.dhcp = b;
    if (nvs_get_u8(h, "rewrite", &b) == ESP_OK) cfg.rewrite_ee_to_c8 = b;
    nvs_get_string(h, "local", cfg.local_ip, sizeof(cfg.local_ip));
    nvs_get_string(h, "gateway", cfg.gateway, sizeof(cfg.gateway));
    nvs_get_string(h, "netmask", cfg.netmask, sizeof(cfg.netmask));
    nvs_get_string(h, "peer", cfg.peer_ip, sizeof(cfg.peer_ip));
    nvs_get_u16(h, "clp", &cfg.crsf_local_port);
    nvs_get_u16(h, "crp", &cfg.crsf_remote_port);
    nvs_get_u32(h, "cbaud", &cfg.crsf_baud);
    nvs_get_u16(h, "cturn", &cfg.crsf_turnaround_us);
    nvs_get_u16(h, "mlp", &cfg.mav_local_port);
    nvs_get_u16(h, "mrp", &cfg.mav_remote_port);
    nvs_get_u32(h, "mbaud", &cfg.mav_baud);
    nvs_get_u16(h, "mgap", &cfg.mav_idle_gap_us);
    nvs_close(h);
}

int make_udp_socket(uint16_t port) {
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s < 0) return -1;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        close(s);
        return -1;
    }
    timeval tv{0, 1000};
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return s;
}

void send_udp(int sock, const uint8_t *data, size_t len, uint16_t port) {
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(port);
    inet_pton(AF_INET, cfg.peer_ip, &peer.sin_addr);
    sendto(sock, data, len, 0, reinterpret_cast<sockaddr *>(&peer), sizeof(peer));
}

void init_uart() {
    uart_config_t crsf{};
    crsf.baud_rate = cfg.crsf_baud;
    crsf.data_bits = UART_DATA_8_BITS;
    crsf.parity = UART_PARITY_DISABLE;
    crsf.stop_bits = UART_STOP_BITS_1;
    crsf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    crsf.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(CRSF_UART, 2048, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(CRSF_UART, &crsf));
    ESP_ERROR_CHECK(uart_set_pin(CRSF_UART, CRSF_PIN, CRSF_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_set_direction(CRSF_PIN, GPIO_MODE_INPUT);

    uart_config_t mav = crsf;
    mav.baud_rate = cfg.mav_baud;
    ESP_ERROR_CHECK(uart_driver_install(MAV_UART, 4096, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART, &mav));
    ESP_ERROR_CHECK(uart_set_pin(MAV_UART, MAV_TX_PIN, MAV_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void crsf_write_single_wire(const uint8_t *data, size_t len) {
    gpio_set_direction(CRSF_PIN, GPIO_MODE_OUTPUT);
    uart_write_bytes(CRSF_UART, data, len);
    uart_wait_tx_done(CRSF_UART, pdMS_TO_TICKS(20));
    ets_delay_us(cfg.crsf_turnaround_us);
    gpio_set_direction(CRSF_PIN, GPIO_MODE_INPUT);
}

void crsf_task(void *) {
    uint8_t frame[CRSF_MAX_FRAME];
    size_t pos = 0, expected = 0;
    uint8_t rx[CRSF_MAX_FRAME];
    while (true) {
        int n = uart_read_bytes(CRSF_UART, rx, sizeof(rx), pdMS_TO_TICKS(2));
        for (int i = 0; i < n; ++i) {
            uint8_t b = rx[i];
            if (pos == 0) { frame[pos++] = b; continue; }
            if (pos == 1) {
                frame[pos++] = b;
                expected = static_cast<size_t>(b) + 2;
                if (expected < 4 || expected > CRSF_MAX_FRAME) { pos = expected = 0; ++stats.crsf_drops; }
                continue;
            }
            frame[pos++] = b;
            if (expected && pos == expected) {
                if (crc8_dvb_s2(frame + 2, expected - 3) == frame[expected - 1]) {
                    if (eth_connected && crsf_socket >= 0) send_udp(crsf_socket, frame, expected, cfg.crsf_remote_port);
                    ++stats.crsf_uart_frames;
                } else ++stats.crsf_crc_errors;
                pos = expected = 0;
            }
        }

        if (crsf_socket >= 0) {
            sockaddr_in src{}; socklen_t sl = sizeof(src);
            int r = recvfrom(crsf_socket, frame, sizeof(frame), 0, reinterpret_cast<sockaddr *>(&src), &sl);
            if (r >= 4 && frame[1] + 2 == r && crc8_dvb_s2(frame + 2, r - 3) == frame[r - 1]) {
                if (cfg.rewrite_ee_to_c8 && frame[0] == 0xEE) frame[0] = 0xC8;
                crsf_write_single_wire(frame, r);
                ++stats.crsf_udp_frames;
            }
        }
    }
}

void mavlink_task(void *) {
    uint8_t buf[MAV_BUFFER];
    while (true) {
        int n = uart_read_bytes(MAV_UART, buf, sizeof(buf), pdMS_TO_TICKS(2));
        if (n > 0) {
            stats.mav_uart_bytes += n;
            if (eth_connected && mav_socket >= 0) send_udp(mav_socket, buf, n, cfg.mav_remote_port);
        }
        if (mav_socket >= 0) {
            sockaddr_in src{}; socklen_t sl = sizeof(src);
            int r = recvfrom(mav_socket, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&src), &sl);
            if (r > 0) {
                uart_write_bytes(MAV_UART, buf, r);
                stats.mav_udp_bytes += r;
            }
        }
    }
}

esp_err_t status_handler(httpd_req_t *req) {
    char ip[16] = "0.0.0.0";
    if (eth_netif) {
        esp_netif_ip_info_t info{};
        if (esp_netif_get_ip_info(eth_netif, &info) == ESP_OK) snprintf(ip, sizeof(ip), IPSTR, IP2STR(&info.ip));
    }
    char json[384];
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
    const char *html = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>OpenFlyIP</title></head><body><h1>OpenFlyIP ESP-IDF</h1><p>WT32-ETH01 CRSF single-wire + MAVLink UDP bridge.</p><p><a href='/status'>JSON status</a></p></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, html);
}

void start_http_server() {
    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&http_server, &hc) != ESP_OK) return;
    httpd_uri_t root{.uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = nullptr};
    httpd_uri_t status{.uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = nullptr};
    httpd_register_uri_handler(http_server, &root);
    httpd_register_uri_handler(http_server, &status);
}

void got_ip_handler(void *, esp_event_base_t, int32_t, void *) {
    eth_connected = true;
    if (crsf_socket < 0) crsf_socket = make_udp_socket(cfg.crsf_local_port);
    if (mav_socket < 0) mav_socket = make_udp_socket(cfg.mav_local_port);
    ESP_LOGI(TAG, "Ethernet connected");
}

void eth_event_handler(void *, esp_event_base_t, int32_t event_id, void *) {
    if (event_id == ETHERNET_EVENT_DISCONNECTED || event_id == ETHERNET_EVENT_STOP) eth_connected = false;
}

void init_ethernet() {
    gpio_config_t pwr{};
    pwr.pin_bit_mask = 1ULL << ETH_PHY_POWER;
    pwr.mode = GPIO_MODE_OUTPUT;
    gpio_config(&pwr);
    gpio_set_level(ETH_PHY_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num = ETH_MDC;
    emac_cfg.smi_gpio.mdio_num = ETH_MDIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = -1;
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_cfg);

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t handle = nullptr;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(handle)));

    if (!cfg.dhcp) {
        esp_netif_dhcpc_stop(eth_netif);
        esp_netif_ip_info_t info{};
        ip4addr_aton(cfg.local_ip, &info.ip);
        ip4addr_aton(cfg.gateway, &info.gw);
        ip4addr_aton(cfg.netmask, &info.netmask);
        ESP_ERROR_CHECK(esp_netif_set_ip_info(eth_netif, &info));
    }

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_handler, nullptr));
    ESP_ERROR_CHECK(esp_eth_start(handle));
}
} // namespace

extern "C" void app_main(void) {
    esp_err_t nvs_rc = nvs_flash_init();
    if (nvs_rc == ESP_ERR_NVS_NO_FREE_PAGES || nvs_rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    load_config();
    init_uart();
    init_ethernet();
    start_http_server();
    xTaskCreatePinnedToCore(crsf_task, "crsf", 4096, nullptr, 12, nullptr, 1);
    xTaskCreatePinnedToCore(mavlink_task, "mavlink", 4096, nullptr, 10, nullptr, 0);
    ESP_LOGI(TAG, "OpenFlyIP ESP-IDF started");
}
