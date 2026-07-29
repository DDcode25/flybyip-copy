#include <Arduino.h>
#include <ETH.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFiUdp.h>

namespace {

constexpr uint8_t CRSF_PIN = 5;
constexpr uint8_t MAV_RX_PIN = 35;
constexpr uint8_t MAV_TX_PIN = 17;
constexpr size_t CRSF_MAX_FRAME = 64;
constexpr size_t MAV_UDP_MAX = 512;

struct Config {
  IPAddress localIp{192, 168, 13, 10};
  IPAddress gateway{192, 168, 13, 1};
  IPAddress subnet{255, 255, 255, 0};
  IPAddress dns{192, 168, 13, 1};
  IPAddress peerIp{192, 168, 13, 11};
  bool dhcp = true;
  uint16_t crsfLocalPort = 1313;
  uint16_t crsfRemotePort = 1313;
  uint32_t crsfBaud = 420000;
  uint16_t crsfTurnaroundUs = 80;
  bool crsfInvert = false;
  bool rewriteEeToC8 = true;
  uint16_t mavLocalPort = 14550;
  uint16_t mavRemotePort = 14550;
  uint32_t mavBaud = 115200;
  uint16_t mavIdleGapUs = 1500;
};

struct Counters {
  uint32_t crsfUartFrames = 0;
  uint32_t crsfUdpFrames = 0;
  uint32_t crsfCrcErrors = 0;
  uint32_t crsfDrops = 0;
  uint32_t mavUartBytes = 0;
  uint32_t mavUdpBytes = 0;
};

Config cfg;
Counters stats;
Preferences prefs;
WebServer web(80);
WiFiUDP crsfUdp;
WiFiUDP mavUdp;
HardwareSerial crsfSerial(1);
HardwareSerial mavSerial(2);
volatile bool ethConnected = false;

uint8_t crc8DvbS2(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool parseIp(const String &value, IPAddress &out) {
  return out.fromString(value);
}

void loadConfig() {
  prefs.begin("openflyip", true);
  cfg.dhcp = prefs.getBool("dhcp", cfg.dhcp);
  cfg.localIp.fromString(prefs.getString("local", cfg.localIp.toString()));
  cfg.gateway.fromString(prefs.getString("gw", cfg.gateway.toString()));
  cfg.subnet.fromString(prefs.getString("mask", cfg.subnet.toString()));
  cfg.dns.fromString(prefs.getString("dns", cfg.dns.toString()));
  cfg.peerIp.fromString(prefs.getString("peer", cfg.peerIp.toString()));
  cfg.crsfLocalPort = prefs.getUShort("clp", cfg.crsfLocalPort);
  cfg.crsfRemotePort = prefs.getUShort("crp", cfg.crsfRemotePort);
  cfg.crsfBaud = prefs.getUInt("cbaud", cfg.crsfBaud);
  cfg.crsfTurnaroundUs = prefs.getUShort("cturn", cfg.crsfTurnaroundUs);
  cfg.crsfInvert = prefs.getBool("cinv", cfg.crsfInvert);
  cfg.rewriteEeToC8 = prefs.getBool("rewrite", cfg.rewriteEeToC8);
  cfg.mavLocalPort = prefs.getUShort("mlp", cfg.mavLocalPort);
  cfg.mavRemotePort = prefs.getUShort("mrp", cfg.mavRemotePort);
  cfg.mavBaud = prefs.getUInt("mbaud", cfg.mavBaud);
  cfg.mavIdleGapUs = prefs.getUShort("mgap", cfg.mavIdleGapUs);
  prefs.end();
}

void saveConfig() {
  prefs.begin("openflyip", false);
  prefs.putBool("dhcp", cfg.dhcp);
  prefs.putString("local", cfg.localIp.toString());
  prefs.putString("gw", cfg.gateway.toString());
  prefs.putString("mask", cfg.subnet.toString());
  prefs.putString("dns", cfg.dns.toString());
  prefs.putString("peer", cfg.peerIp.toString());
  prefs.putUShort("clp", cfg.crsfLocalPort);
  prefs.putUShort("crp", cfg.crsfRemotePort);
  prefs.putUInt("cbaud", cfg.crsfBaud);
  prefs.putUShort("cturn", cfg.crsfTurnaroundUs);
  prefs.putBool("cinv", cfg.crsfInvert);
  prefs.putBool("rewrite", cfg.rewriteEeToC8);
  prefs.putUShort("mlp", cfg.mavLocalPort);
  prefs.putUShort("mrp", cfg.mavRemotePort);
  prefs.putUInt("mbaud", cfg.mavBaud);
  prefs.putUShort("mgap", cfg.mavIdleGapUs);
  prefs.end();
}

void setCrsfReceiveMode() {
  crsfSerial.flush();
  pinMode(CRSF_PIN, INPUT);
}

void sendCrsfSingleWire(const uint8_t *data, size_t len) {
  pinMode(CRSF_PIN, OUTPUT);
  crsfSerial.write(data, len);
  crsfSerial.flush();
  delayMicroseconds(cfg.crsfTurnaroundUs);
  pinMode(CRSF_PIN, INPUT);
}

void onEthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_GOT_IP:
      ethConnected = true;
      crsfUdp.begin(cfg.crsfLocalPort);
      mavUdp.begin(cfg.mavLocalPort);
      Serial.printf("ETH IP: %s\n", ETH.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      ethConnected = false;
      break;
    default:
      break;
  }
}

String htmlPage() {
  String s;
  s.reserve(5000);
  s += F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  s += F("<title>OpenFlyIP</title><style>body{font-family:Arial;max-width:850px;margin:auto;padding:20px;background:#111;color:#eee}fieldset{margin:14px 0;border:1px solid #555}label{display:block;margin:8px 0}input{width:180px;padding:6px}button{padding:10px 18px}</style></head><body>");
  s += F("<h1>OpenFlyIP WT32-ETH01</h1><form method='post' action='/save'>");
  s += "<fieldset><legend>Network</legend><label>DHCP <input type='checkbox' name='dhcp' " + String(cfg.dhcp ? "checked" : "") + "></label>";
  s += "<label>Local IP <input name='local' value='" + cfg.localIp.toString() + "'></label>";
  s += "<label>Gateway <input name='gw' value='" + cfg.gateway.toString() + "'></label>";
  s += "<label>Subnet <input name='mask' value='" + cfg.subnet.toString() + "'></label>";
  s += "<label>Peer IP <input name='peer' value='" + cfg.peerIp.toString() + "'></label></fieldset>";
  s += "<fieldset><legend>CRSF single-wire GPIO5</legend><label>Baud <input name='cbaud' value='" + String(cfg.crsfBaud) + "'></label>";
  s += "<label>Local UDP <input name='clp' value='" + String(cfg.crsfLocalPort) + "'></label>";
  s += "<label>Remote UDP <input name='crp' value='" + String(cfg.crsfRemotePort) + "'></label>";
  s += "<label>Turnaround us <input name='cturn' value='" + String(cfg.crsfTurnaroundUs) + "'></label>";
  s += "<label>Invert <input type='checkbox' name='cinv' " + String(cfg.crsfInvert ? "checked" : "") + "></label>";
  s += "<label>Rewrite EE to C8 <input type='checkbox' name='rewrite' " + String(cfg.rewriteEeToC8 ? "checked" : "") + "></label></fieldset>";
  s += "<fieldset><legend>MAVLink UART RX35/TX17</legend><label>Baud <input name='mbaud' value='" + String(cfg.mavBaud) + "'></label>";
  s += "<label>Local UDP <input name='mlp' value='" + String(cfg.mavLocalPort) + "'></label>";
  s += "<label>Remote UDP <input name='mrp' value='" + String(cfg.mavRemotePort) + "'></label>";
  s += "<label>Idle gap us <input name='mgap' value='" + String(cfg.mavIdleGapUs) + "'></label></fieldset>";
  s += F("<button type='submit'>Save and reboot</button></form><p><a href='/status'>JSON status</a></p></body></html>");
  return s;
}

void configureWeb() {
  web.on("/", HTTP_GET, [] { web.send(200, "text/html", htmlPage()); });
  web.on("/status", HTTP_GET, [] {
    String json = "{\"ethernet\":" + String(ethConnected ? "true" : "false") +
                  ",\"ip\":\"" + ETH.localIP().toString() + "\"" +
                  ",\"crsf_uart_frames\":" + String(stats.crsfUartFrames) +
                  ",\"crsf_udp_frames\":" + String(stats.crsfUdpFrames) +
                  ",\"crsf_crc_errors\":" + String(stats.crsfCrcErrors) +
                  ",\"mav_uart_bytes\":" + String(stats.mavUartBytes) +
                  ",\"mav_udp_bytes\":" + String(stats.mavUdpBytes) + "}";
    web.send(200, "application/json", json);
  });
  web.on("/save", HTTP_POST, [] {
    cfg.dhcp = web.hasArg("dhcp");
    parseIp(web.arg("local"), cfg.localIp);
    parseIp(web.arg("gw"), cfg.gateway);
    parseIp(web.arg("mask"), cfg.subnet);
    parseIp(web.arg("peer"), cfg.peerIp);
    cfg.crsfBaud = web.arg("cbaud").toInt();
    cfg.crsfLocalPort = web.arg("clp").toInt();
    cfg.crsfRemotePort = web.arg("crp").toInt();
    cfg.crsfTurnaroundUs = web.arg("cturn").toInt();
    cfg.crsfInvert = web.hasArg("cinv");
    cfg.rewriteEeToC8 = web.hasArg("rewrite");
    cfg.mavBaud = web.arg("mbaud").toInt();
    cfg.mavLocalPort = web.arg("mlp").toInt();
    cfg.mavRemotePort = web.arg("mrp").toInt();
    cfg.mavIdleGapUs = web.arg("mgap").toInt();
    saveConfig();
    web.send(200, "text/plain", "Saved. Rebooting...");
    delay(500);
    ESP.restart();
  });
  web.begin();
}

void processCrsfUart() {
  static uint8_t frame[CRSF_MAX_FRAME];
  static size_t pos = 0;
  static size_t expected = 0;

  while (crsfSerial.available()) {
    const uint8_t b = crsfSerial.read();
    if (pos == 0) {
      frame[pos++] = b;
      expected = 0;
      continue;
    }
    if (pos == 1) {
      frame[pos++] = b;
      expected = static_cast<size_t>(b) + 2;
      if (expected < 4 || expected > CRSF_MAX_FRAME) {
        pos = expected = 0;
        ++stats.crsfDrops;
      }
      continue;
    }
    frame[pos++] = b;
    if (expected && pos == expected) {
      const uint8_t receivedCrc = frame[expected - 1];
      const uint8_t calculatedCrc = crc8DvbS2(frame + 2, expected - 3);
      if (receivedCrc == calculatedCrc) {
        crsfUdp.beginPacket(cfg.peerIp, cfg.crsfRemotePort);
        crsfUdp.write(frame, expected);
        crsfUdp.endPacket();
        ++stats.crsfUartFrames;
      } else {
        ++stats.crsfCrcErrors;
      }
      pos = expected = 0;
    }
  }
}

void processCrsfUdp() {
  const int packetSize = crsfUdp.parsePacket();
  if (packetSize < 4 || packetSize > static_cast<int>(CRSF_MAX_FRAME)) return;
  uint8_t frame[CRSF_MAX_FRAME];
  const int n = crsfUdp.read(frame, sizeof(frame));
  if (n != packetSize || frame[1] + 2 != n) {
    ++stats.crsfDrops;
    return;
  }
  if (crc8DvbS2(frame + 2, n - 3) != frame[n - 1]) {
    ++stats.crsfCrcErrors;
    return;
  }
  if (cfg.rewriteEeToC8 && frame[0] == 0xEE) frame[0] = 0xC8;
  sendCrsfSingleWire(frame, n);
  ++stats.crsfUdpFrames;
}

void processMavUart() {
  static uint8_t buffer[MAV_UDP_MAX];
  static size_t len = 0;
  static uint32_t lastByteUs = 0;

  while (mavSerial.available() && len < sizeof(buffer)) {
    buffer[len++] = mavSerial.read();
    lastByteUs = micros();
    ++stats.mavUartBytes;
  }
  if (len && static_cast<uint32_t>(micros() - lastByteUs) >= cfg.mavIdleGapUs) {
    mavUdp.beginPacket(cfg.peerIp, cfg.mavRemotePort);
    mavUdp.write(buffer, len);
    mavUdp.endPacket();
    len = 0;
  }
}

void processMavUdp() {
  int packetSize = mavUdp.parsePacket();
  if (packetSize <= 0) return;
  uint8_t buffer[MAV_UDP_MAX];
  while (packetSize > 0) {
    const int chunk = min(packetSize, static_cast<int>(sizeof(buffer)));
    const int n = mavUdp.read(buffer, chunk);
    if (n <= 0) break;
    mavSerial.write(buffer, n);
    stats.mavUdpBytes += n;
    packetSize -= n;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  loadConfig();

  WiFi.onEvent(onEthEvent);
  if (!cfg.dhcp) ETH.config(cfg.localIp, cfg.gateway, cfg.subnet, cfg.dns);
  ETH.begin();

  crsfSerial.begin(cfg.crsfBaud, SERIAL_8N1, CRSF_PIN, CRSF_PIN, cfg.crsfInvert);
  setCrsfReceiveMode();
  mavSerial.begin(cfg.mavBaud, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);
  configureWeb();
}

void loop() {
  web.handleClient();
  if (ethConnected) {
    processCrsfUart();
    processCrsfUdp();
    processMavUart();
    processMavUdp();
  }
  delay(1);
}
