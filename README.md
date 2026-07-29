# OpenFlyIP for WT32-ETH01

Experimental open-source Ethernet bridge firmware for WT32-ETH01 / ESP32 + LAN8720.

Implemented in this initial version:

- CRSF single-wire half-duplex bridge over UDP
- CRSF CRC8-DVB-S2 validation
- optional CRSF address rewrite `0xEE -> 0xC8`
- MAVLink transparent UART-to-UDP bridge
- Ethernet DHCP with static fallback
- embedded Web UI and JSON status
- persistent settings in ESP32 NVS

## Default wiring

| Function | GPIO |
|---|---:|
| CRSF single-wire | 5 |
| MAVLink RX | 35 |
| MAVLink TX | 17 |

LAN8720 pins are fixed by the WT32-ETH01 board definition.

## Default network

- CRSF UDP port: `1313`
- MAVLink UDP port: `14550`
- peer IP: `192.168.13.11`
- local static fallback: `192.168.13.10/24`

Open the device IP in a browser to change the peer IP, ports and protocol parameters.

## Build

```bash
pio run
pio run -t upload
pio device monitor
```

## Electrical warning

ESP32 pins are 3.3 V only. For a RadioMaster JR-bay or other long/noisy CRSF single-wire connection, use a proper transistor or logic-buffer interface instead of connecting an unknown-voltage signal directly to GPIO5.

## Status

This is an experimental first implementation. It has not yet been validated on real flight hardware. Test on the bench without propellers and with a logic analyzer before operational use.
