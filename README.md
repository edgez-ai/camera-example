# EdgeZ HaLow camera example

A standalone PlatformIO project for a Heltec HT-HC33 camera gateway and an
EdgeZ nRF54L15 + FGH200M/MM8108 remote node with a Putal/VC0706 UART camera.
Connect a phone or laptop to HC33's Wi-Fi, open its page, and click **Take photo**.
The nRF wakes, powers the camera, captures a JPEG, and returns it in checked,
retryable blocks. The page displays the completed image and offers a download.
HC33's USER button (GPIO0, active low) and the page's **Wake LED** button
send the same LED-only command: the nRF LED lights for five seconds, without
powering the camera. **Take photo** is a separate page action. Serial `w`
triggers the LED, `c` triggers a photo, and `s` prints current progress.

## Clone and build

Install PlatformIO Core, Git, and the platform's required build tools. Clone
with submodules; the example uses only public SDK checkouts and protected
prebuilt mesh archives. No private mesh source checkout is required.

```sh
git clone --recurse-submodules https://github.com/edgez-ai/camera-example.git
cd camera-example
pio run -e hc33
pio run -e nrf54-camera
```

Run the two platform builds sequentially: the installed PlatformIO platforms
share CMake/Ninja tool packages. The nRF prebuild hook applies the same
Zephyr/PlatformIO integration as the LED example. SPI sleep support is already
included in the pinned public SDK; builds leave its tracked sources untouched.

```sh
pio run -e hc33 -t upload --upload-port /dev/cu.SLAB_USBtoUART
pio run -e nrf54-camera -t upload
pio device monitor -e hc33 --port /dev/cu.SLAB_USBtoUART
```

HC33 uses its UART USB interface. The nRF environment uses a nRF54-capable
CMSIS-DAP/OpenOCD probe, verifies the written image, and resets to run. If more
than one probe is present, supply its serial using `--upload-port SERIAL`.
Diagnostics on the nRF use nonblocking SEGGER RTT.

## Camera wiring

This preserves the **actual diagnostic code** from offgrid-tracker commit
`39820d3`, including its swapped UART mapping and 921600-baud camera. The README
in that older commit described the opposite pin order; the code/overlay is the
reference used here. Unplug the L76K GPS before attaching the camera.

| Camera signal | nRF54L15 carrier |
| --- | --- |
| RXD | P1.12 (MCU UART TX) |
| TXD | P1.11 (MCU UART RX) |
| Switched camera supply enable | P1.09, active high, through the existing carrier circuit |
| Ground | Common ground |
| UART | 921600 baud, 8N1, 3.3 V TTL |

P1.09 is an enable signal, not a camera power supply output. Use the existing
carrier's camera power connection. The camera replaces the GPS on UART22;
IMU, ADC and GPS services are disabled. UART pins return to their sleep pinctrl
state and camera power is disabled after capture transfer or abandonment.
Edit `zephyr/boards/camera.overlay` for another wiring layout and
`CONFIG_CAMERA_UART_BAUD` in `zephyr/prj.conf` for another baud rate. The default
resolution command is the same 640x480 command used in the existing diagnostic.
A generic 38400-baud VC0706 may require different camera settings.

## Use the page

1. Power HC33 and the remote camera board.
2. Join **EdgeZ-Camera**, password **edgez-camera**.
3. Open **http://192.168.4.1**. Keep this Wi-Fi connection even if the phone
   reports that it has no Internet access.
4. Click **Wake LED** (or press HC33 USER) to check the remote LED.
5. Click **Take photo** separately, wait for capture and progress, then view or save it.

SSID/password are configurable under the ESP-IDF **Camera example** Kconfig
menu. The page is fully local and needs no Appwrite, MQTT, cloud service, CDN,
or phone application. A photo remains in HC33 RAM until the next capture or
reset; a reboot clears it. One camera/one capture is supported at a time.

## Radio and sleep behavior

The ESP32-S3's **2.4 GHz WPA2 SoftAP** serves the browser. Independently,
HC33's MM6108 runs the LED example's **HaLow AP timing interface**, and the
nRF's MM8108 uses a **STA interface** with a static peer/BSSID contract.
There are no on-air association exchanges or mesh peering. This is the same
experimental manually configured AP/STA mechanism as the LED wake test,
not native TWT on an association-free MESH interface.

- HaLow is fixed to **US 914.5 MHz / 1 MHz (channel 25)**. Match antennas and
  use this test profile only where that channel is permitted.
- **DTIM10**, 100 TU beacon interval: nominal receive opportunity every
  **1.024 seconds**. Dynamic PS timeout: **13 ms**, checked after setup.
- HC33 sets AID 1's TIM pending bit and sends a bounded capture burst around
  the actual DTIM beacon. No serial mode selection is required.
- The nRF holds the radio awake throughout camera capture and block reads,
  then releases host wake. The camera turns off and its UART suspends.
- There are no periodic sensor reports or autonomous captures. Initial
  discovery retries until HC33 is found. A lost transfer expires after
  90 seconds without requests and switches the camera off.

Camera startup/reset alone includes two four-second waits from the existing
camera diagnostic. The initial wake is therefore only part of capture latency.
The JPEG stays frozen in the camera's frame buffer while blocks are read;
the nRF does not allocate a full-image buffer. HC33 uses PSRAM when available,
with a checked internal-RAM fallback. Maximum image size is 512 KiB.

## Block protocol and HTTP API

The application uses EdgeZ vendor Public Action frames carrying the existing
PAwR bulk-stream envelope. The example's own versioned camera payload has
header and data CRC-32, request/session IDs, absolute offsets and lengths.
It pulls 768-byte blocks using stop-and-wait retransmission. Duplicate capture
requests replay the original result instead of taking another photograph;
read retries address the same frozen frame. Stale sessions, wrong offsets,
invalid sizes and corrupted blocks cannot advance the receiver. A JPEG is
published only after all bytes, JPEG markers and the finish exchange pass.
The action-frame camera protocol is unencrypted; the SoftAP password protects
the browser connection, not the HaLow link. It is an evaluation example.

| Endpoint | Purpose |
| --- | --- |
| `GET /` | Offline camera page |
| `POST /api/led` | Queue LED-only wake, identical to HC33 USER |
| `POST /api/capture` | Queue capture; 202 accepted or 409 busy |
| `GET /api/status` | Phase, session ID, progress, retries and error |
| `GET /photo.jpg?id=ID` | Completed JPEG for that exact capture, or 404 |

```sh
curl -X POST http://192.168.4.1/api/led
curl -X POST http://192.168.4.1/api/capture
curl http://192.168.4.1/api/status
# Replace ID with the completed capture's numeric id from status.
curl 'http://192.168.4.1/photo.jpg?id=ID' -o photo.jpg
```

## Validation

```sh
python3 scripts/test_protocol.py
pio run -e hc33
pio run -e nrf54-camera
```

The host test runs the **same C codec and image assembler used by the HC33**
with address/undefined-behavior sanitizers. It checks every single-bit packet
corruption, truncated messages, buffer limits, wrong sessions, out-of-order and
duplicate delivery, JPEG markers, and ten transfer sizes up to 512 KiB.
Firmware build and hardware results are recorded separately; compiling and
simulating a transfer do not establish that a physical camera captured a photo.

Sources: [VC0706 protocol](https://cdn-shop.adafruit.com/datasheets/VC0706protocol.pdf),
[ESP-IDF Wi-Fi documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32s3/api-guides/wifi.html).
See `NOTICE` for code provenance and `LICENSE` for EdgeZ licensing.
