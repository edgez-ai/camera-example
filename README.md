# EdgeZ HaLow camera example

A standalone PlatformIO project for a Heltec HT-HC33 camera gateway and an
EdgeZ nRF54L15 + FGH200M/MM8108 remote node with a Putal/VC0706 UART camera.
**Current test mode:** `CONFIG_CAMERA_DUMMY_IMAGE=y` makes **Take photo** send a
known JPEG labelled **TEST IMAGE / NO CAMERA**, with camera power kept off.
It exercises the same DTIM wake, block transfer, retries, whole-image reference
CRC, HTTP download and browser display as a real capture. All dropdown sizes
have checked-in fixtures. Set this option to `n` and rebuild/reflash nRF to
resume real camera capture. **Get camera version** remains an explicit hardware
query and may power the camera.

Connect a phone or laptop to HC33's Wi-Fi, open its page, and click **Take photo**.
The nRF wakes and returns a JPEG in checked, retryable blocks. In real-camera
mode it powers the camera and captures first; the current test mode reads a
known fixture from flash. The page displays the completed image and offers a download.
HC33's USER button (GPIO0, active low) and the page's **Wake LED** button
send the same LED-only command: the nRF LED lights for five seconds, without
powering the camera. **Take photo** sends the selected resolution to the nRF.
**Get camera version** reads the camera firmware version without taking a photo.
Serial `w` triggers the LED, `c` triggers a 640x480 photo, `v` reads the version,
and `s` prints current progress.

## Clone and build

Install PlatformIO Core, Git, and the platform's required build tools. Clone
with submodules; the example uses only public SDK checkouts and protected
prebuilt mesh archives. Both platforms pin the same `mm-iot-sdk` source
commit, `ad3551b58dbaaa969e5f32dda3ca6a3ca45372c4`. No private mesh source
checkout is required.

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
`CONFIG_CAMERA_UART_BAUD` in `zephyr/prj.conf` for another baud rate. The receiver uses the later working camera example's double-buffered DMA UART
path. The default resolution command is its 640x480 setting.
A generic 38400-baud VC0706 may require different camera settings.

## Use the page

1. Power HC33 and the remote camera board.
2. Join **EdgeZ-Camera**, password **edgez-camera**.
3. Open **http://192.168.4.1**. Keep this Wi-Fi connection even if the phone
   reports that it has no Internet access.
4. Click **Wake LED** (or press HC33 USER) to check the remote LED.
5. Click **Get camera version** to show the camera's version reply.
6. Select a resolution, then click **Take photo**, wait for progress, and view or save it.

The dropdown includes 160x120, 320x240, 640x480 (default), 1024x768, 1280x720,
1280x960, 1600x1200, 1920x1080 and 2304x1296. Available modes depend on the
attached camera. The selected width/height travel in the HTTP request and the
HaLow capture payload; nRF applies their matching camera register setting.
HC33 checks the received JPEG's actual dimensions against that request before
publishing it. A camera that returns a different size produces an error.

SSID/password are configurable under the ESP-IDF **Camera example** Kconfig
menu. The page is fully local and needs no Appwrite, MQTT, cloud service, CDN,
or phone application. A photo remains in HC33 RAM until the next capture or
reset; a reboot clears it. One camera/one capture is supported at a time.

## Radio and sleep behavior

The ESP32-S3's **2.4 GHz WPA2 SoftAP** serves the browser. Independently,
HC33's MM6108 runs the LED example's **HaLow AP timing interface**, and the
nRF's MM8108 uses a **STA interface** with a static peer/BSSID contract.
The AP BSSID is learned from discovery, never hardcoded. There are no on-air association exchanges or mesh peering. This is the same
experimental manually configured AP/STA mechanism as the LED wake test,
not native TWT on an association-free MESH interface.

- HaLow is fixed to **US 914.5 MHz / 1 MHz (channel 25)**. Match antennas and
  use this test profile only where that channel is permitted.
- **DTIM10**, 100 TU beacon interval: nominal receive opportunity every
  **1.024 seconds**. Dynamic PS timeout: **13 ms**, checked after setup.
- HC33 sets AID 1's TIM pending bit and sends a bounded capture burst around
  the actual DTIM beacon. No serial mode selection is required.
- During camera/version transactions the nRF explicitly disables firmware
  radio power saving and holds host wake, keeping RX available between block
  requests. It waits for actual TX completion before ending the transaction,
  then restores DTIM10/13 ms and releases host wake. Camera power and UART are
  off while idle.
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
It pulls 256-byte blocks using stop-and-wait retransmission. Duplicate capture
requests replay the original result instead of taking another photograph;
read retries address the same frozen frame. Stale sessions, wrong offsets,
invalid sizes and corrupted blocks cannot advance the receiver. Capture arguments
are checked on both processors. A JPEG is
published only after all bytes, JPEG markers, requested dimensions and the finish exchange pass.
The action-frame camera protocol is unencrypted; the SoftAP password protects
the browser connection, not the HaLow link. It is an evaluation example.

| Endpoint | Purpose |
| --- | --- |
| `GET /` | Offline camera page |
| `POST /api/led` | Queue LED-only wake, identical to HC33 USER |
| `POST /api/capture` | JSON `{ "width": 640, "height": 480 }`; queue capture or 409 busy |
| `POST /api/version` | Power camera, read version, power it off |
| `GET /api/status` | Phase, session ID, progress, retries and error |
| `GET /photo.jpg?id=ID` | Completed JPEG for that exact capture, or 404 |

```sh
curl -X POST http://192.168.4.1/api/led
curl -X POST http://192.168.4.1/api/version
curl -X POST http://192.168.4.1/api/capture \
  -H 'Content-Type: application/json' -d '{"width":1280,"height":720}'
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

The host test also streams all nine deterministic JPEG fixtures through the
actual codec/assembler and checks their known CRCs and dimensions. Regenerate
them only when needed with `pip install Pillow` and
`python3 scripts/generate_dummy_images.py`; normal firmware builds have no
Pillow dependency.

The host test runs the **same C codec and image assembler used by the HC33**
with address/undefined-behavior sanitizers. It checks every single-bit packet
corruption, truncated messages, buffer limits, wrong sessions, out-of-order and
duplicate delivery, JPEG markers and dimensions, capture arguments, and thirteen
transfer sizes up to 512 KiB.
Both the physical USER and web Wake LED actions were manually verified on
2026-11-11. Native DTIM10 setup and five-second LED on/off cycles were also
recorded in device logs. On 2026-11-12 the version-only command returned
`PTC3M1 1.2` (raw hex `505443334d3120312e32`). A real 640x480 capture transferred
9,603 bytes in 256-byte blocks with zero retries, passed JPEG dimension checks,
and completed the finish exchange (CRC32 `c97a3012`). This verified the radio
transport; the browser still reported an error, which is being isolated using
the dummy-image test. The nRF then logged camera
power-off and restoration of DTIM10/13 ms. Larger 768-byte replies did not
complete reception in this setup; they are not a validated mode for this
example. Camera communication uses the later working DMA receiver. This camera returns status 1 on its resolution-write ACK, matching
the older driver's accepted framing; the receiver therefore requires the
actual JPEG dimensions to match the selected mode.

Firmware build and hardware results are recorded separately; compiling and
simulating a transfer do not establish that a physical camera captured a photo.

Sources: [VC0706 protocol](https://cdn-shop.adafruit.com/datasheets/VC0706protocol.pdf),
[ESP-IDF Wi-Fi documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.5/esp32s3/api-guides/wifi.html).
See `NOTICE` for code provenance and `LICENSE` for EdgeZ licensing.
