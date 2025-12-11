# Smart Parking System 🚗🅿️

**Version:** Final Production Version  
**Status:** Active  

## 📖 Project Overview

This **Smart Parking System** is a robust IoT solution designed to manage and monitor parking spots in real-time. It utilizes **ESP32** microcontrollers, **ESP-NOW** for low-latency local communication, and **Firebase** for cloud-based monitoring and revenue tracking.

The system consists of multiple component types working together:

1. **Central Gateway (Dashboard)**: The "Server" node that manages all parking spots, displays global status on a TFT screen, and syncs data with the cloud.
2. **Sensor Nodes**: Individual units placed at each parking spot to detect vehicle presence, indicate status via LEDs, and report to the Gateway.
3. **ESP-CAM Module**: Camera-based node for license plate capture and image processing.

---

## 📁 Project Structure

```
Smart-Parking-System/
├── main_Dashboard/         # Central Gateway - Master controller & UI
│   ├── main_Dashboard.ino  # Main gateway code
│   ├── image.h             # QR code image for WiFi config mode
│   └── data/               # LittleFS web server files
├── ESPCAM/                 # ESP32-CAM module for camera-based detection
│   ├── ESPCAM.ino          # Motion detection, LED control, buzzer
│   └── pitches.h           # Musical note definitions for buzzer
├── ESP_module_dummy/       # Simplified sensor node (testing/demo)
│   └── ESP_module_dummy.ino
├── ESP_module_full_test/   # Full-featured sensor node with sensor input
│   └── ESP_module_full_test.ino
├── extract_letter/         # License plate recognition module
│   ├── extract_letter.ino  # OCR/image processing code
│   ├── camera_pins.h       # Camera pin definitions
│   ├── board_config.h      # Board configuration
│   └── ...                 # Additional support files
├── README.md               # This file
├── IoT_report.pdf          # Project report documentation
└── IoT_slide.pdf           # Project presentation slides
```

---

## 🏗️ System Architecture

### 1. Central Gateway (`main_Dashboard`)
*   **Role:** Master Controller & User Interface.
*   **Hardware:** ESP32, TFT Display (with Touch), Wi-Fi functionality, SD Card for offline logging.
*   **Key Features:**
    *   **ESP-NOW Receiver:** Collects status updates (Occupied/Free) from all Sensor Nodes.
    *   **Cloud Sync:** Pushes real-time updates to **Firebase Realtime Database** (Status, License Plate, Duration, Revenue).
    *   **GUI:** Displays live status of each spot, revenue, and connection health on a local touchscreen interface (LVGL).
    *   **Network Manager:** Monitors "Heartbeats" from nodes to detect offline devices.
    *   **WiFi Manager:** Web-based configuration portal for setting up WiFi credentials.
    *   **Offline Mode:** Continues to log data to SD card when WiFi is unavailable.
    *   **LED Control:** Synchronizes LED states between Firebase and physical nodes.

### 2. ESP-CAM Module (`ESPCAM`)
*   **Role:** Camera-based motion detection and license plate capture.
*   **Hardware:** ESP32-CAM, PIR Motion Sensor, WS2812B LED Strip, Piezo Buzzer.
*   **Key Features:**
    *   **PIR Motion Detection:** Detects vehicle presence with debounce logic.
    *   **LED Indicator:** Visual status display (Red = Occupied, Green = Available, Blue = Available + LED Command ON).
    *   **Buzzer Feedback:** Plays melody on motion detection.
    *   **ESP-NOW Communication:** Sends parking status with simulated license plates.
    *   **Auto-Discovery:** Automatically finds and connects to Gateway via broadcast.

### 3. Sensor Nodes (`ESP_module_full_test` / `ESP_module_dummy`)
*   **Role:** Endpoint Detectors for parking spots.
*   **Hardware:** ESP32, Presence Sensor (GPIO 13), WS2812B LED Strip, Piezo Buzzer.
*   **Key Features:**
    *   **Detection:** Monitors the physical parking spot sensor.
    *   **Feedback:** Controls WS2812B LEDs (Green = Free, Red = Occupied) based on commands from the Gateway.
    *   **Communication:** Sends sensor data + (Simulated) License Plate info to the Gateway via ESP-NOW.
    *   **Discovery:** Auto-discovers the Gateway via broadcast messages.
    *   **Channel Hopping:** Scans all WiFi channels to find the Gateway.

### 4. License Plate Recognition (`extract_letter`)
*   **Role:** Image processing for license plate OCR.
*   **Hardware:** ESP32-CAM with OV2640/OV5640 camera.
*   **Features:** Captures images and processes them for character extraction.

---

## 🛠️ Hardware Requirements

| Component | Quantity (Per Unit) | Notes |
| :--- | :--- | :--- |
| **ESP32 Dev Module** | 1 per Node + 1 for Gateway | Core controller |
| **ESP32-CAM** | 1 per camera node | For ESPCAM/extract_letter modules |
| **TFT Touch Display** | 1 (Gateway Only) | ILI9341 / XPT2046 recommended |
| **WS2812B LED Strip** | 1 Strip (Nodes) | Visual occupancy indicator (8 pixels used) |
| **Presence Sensor** | 1 (Nodes) | IR, Ultrasonic, or Magnetic (High = Active) |
| **PIR Motion Sensor** | 1 (ESPCAM) | Motion detection |
| **Piezo Buzzer** | 1 (Nodes) | Audio feedback |
| **SD Card Module** | 1 (Gateway) | For offline data logging |
| **Power Supply** | 5V USB / Battery | |

---

## 💻 Software & Libraries

The project is built using the **Arduino Framework** for ESP32.

### Required Libraries
*   **ESP-NOW / WiFi**: Core ESP32 libraries.
*   **Firebase ESP Client**: For Google Firebase Realtime Database connection.
    *   *Authors: Mobizt*
*   **LVGL**: Light and Versatile Graphics Library (for Gateway UI).
*   **TFT_eSPI**: Display driver.
*   **XPT2046_Touchscreen**: Touch controller driver.
*   **Adafruit NeoPixel**: For controlling WS2812B LEDs.
*   **ESPAsyncWebServer**: For WiFi Manager web portal.
*   **LittleFS**: File system for storing WiFi credentials.

---

## ⚙️ Configuration & Setup

### 1. Firebase Setup
The system requires a Firebase Realtime Database configured with the following structure:
```json
/parkingLots/mainStreetGarage
  ├── /spots
  │     ├── /A01
  │     │     ├── status: "available" | "occupied"
  │     │     ├── led_status: "on" | "off"
  │     │     ├── connection_status: "online" | "offline"
  │     │     └── ... (revenue, duration)
  │     └── ... (A02, A03)
  └── /lotInfo
        ├── MaxDevice: 10
        └── /BoardMac
              ├── 0: "FF:FF:FF:FF:FF:FF"
              └── 1: "..."
```

### 2. Gateway Configuration (`main_Dashboard.ino`)
*   **Wi-Fi**: Uses WiFi Manager - access 192.168.4.1 when in config mode.
*   **Firebase**: Update `WEB_API_KEY`, `DATABASE_URL`, `USER_EMAIL`, `USER_PASS`.
*   **Boards**: The system pulls allowed MAC addresses from Firebase (`FB_CONFIG_MAC_ROOT`). Ensure your Node MAC addresses are registered there.

### 3. Sensor Node Configuration
*   **ID**: Set `boardId` manually if not dynamically assigned.
*   **Pin Defs**: Ensure pin definitions match your wiring:
    *   `WAKEUP_PIN` / `PIN_PIR` - Sensor input
    *   `BUZZZER_PIN` / `PIN_BUZZER` - Buzzer output
    *   `PIN_WS2812B` - LED strip data

---

## 🚀 Usage

1.  **Power On**: Start the **Gateway** first. It will initialize the display and connect to Wi-Fi/Firebase.
2.  **WiFi Setup**: If no WiFi is configured, the Gateway enters Config Mode displaying a QR code. Connect to "SMART-PARKING-CONFIG" AP and navigate to 192.168.4.1.
3.  **Start Nodes**: Power on the **Sensor Nodes**. They will listen for the Gateway's broadcast and auto-connect.
4.  **Monitoring**:
    *   **Local**: View the specific parking spot tabs on the Gateway's TFT screen.
    *   **Cloud**: Check the Firebase Console or connected App to see real-time updates.
5.  **Interaction**:
    *   **Park a Car**: Trigger the sensor on a Node → Node sends "Occupied" → Gateway updates Cloud & calculates start time → Node LED turns RED.
    *   **Leave**: Sensor clears → Node sends "Free" → Gateway calculates duration & revenue → Node LED turns GREEN.
    *   **LED Control**: Toggle the LED buttons on the Gateway UI to manually override Node LEDs.

---

## 🔧 Pin Configurations

### Gateway (main_Dashboard)
| Function | Pin |
|----------|-----|
| TFT Display | Standard TFT_eSPI pins |
| Touch IRQ | GPIO 36 |
| Touch CS | GPIO 33 |
| Touch CLK | GPIO 25 |
| Touch MOSI | GPIO 32 |
| Touch MISO | GPIO 39 |
| SD Card CS | GPIO 5 |
| SD Card SCK | GPIO 18 |
| SD Card MOSI | GPIO 23 |
| SD Card MISO | GPIO 19 |

### ESPCAM / Sensor Nodes
| Function | Pin |
|----------|-----|
| WS2812B LED | GPIO 14 |
| PIR Sensor | GPIO 13 |
| Buzzer | GPIO 15 |

---

## 📡 Communication Protocol

### ESP-NOW Messages

**Sensor → Gateway (struct_message):**
```cpp
typedef struct {
  int id;           // Board ID
  int status;       // 0 = Available, 1 = Occupied
  char CarLicense[11];  // License plate string
  int readingId;    // Sequence number
} struct_message;
```

**Gateway → Sensor (led_message):**
```cpp
typedef struct {
  int id;     // Target Board ID
  bool state; // LED state: true = ON, false = OFF
} led_message;
```

### Gateway Discovery
Nodes scan all WiFi channels looking for Gateway broadcasts:
- Format: `DISCOVER_MASTER:SSID_NAME`
- Upon receiving, nodes lock to that channel and register the Gateway as a peer.

---

## ⚠️ Notes
*   **MAC Address Security**: The Gateway strictly filters incoming ESP-NOW messages based on MAC addresses registered in Firebase. Unregistered devices are ignored.
*   **Demo Mode**: The Sensor Nodes currently generate a **random license plate** for demonstration purposes when triggered. In a real deployment, this would be replaced by an OCR camera module.
*   **Offline Mode**: When WiFi is unavailable but SSID is configured, the Gateway logs all ESP-NOW data to the SD card for later sync.

---

## 📄 License

This project is open source and available for educational and personal use.

---

## 👥 Contributors

TAPIT Team - IoT Project

