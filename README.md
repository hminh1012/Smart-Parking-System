# Smart Parking System 🚗🅿️

**Version:** Final Production Version  
**Status:** Active  

## 📖 Project Overview

This **Smart Parking System** is a robust IoT solution designed to manage and monitor parking spots in real-time. It utilizes **ESP32** microcontrollers, **ESP-NOW** for low-latency local communication, and **Firebase** for cloud-based monitoring and revenue tracking.

The system consists of two main component types:
1.  **Central Gateway (Dashboard)**: The "Server" node that manages all parking spots, displays global status on a TFT screen, and syncs data with the cloud.
2.  **Sensor Nodes**: Individual units placed at each parking spot to detect vehicle presence, indicate status via LEDs, and report to the Gateway.

---

## 🏗️ System Architecture

### 1. Central Gateway (`main_Dashboard`)
*   **Role:** Master Controller & User Interface.
*   **Hardware:** ESP32, TFT Display (with Touch), Wi-Fi functionality.
*   **Key Responsibilities:**
    *   **ESP-NOW Receiver:** Collects status updates (Occupied/Free) from all Sensor Nodes.
    *   **Cloud Sync:** Pushes real-time updates to **Firebase Realtime Database** (Status, License Plate, Duration, Revenue).
    *   **GUI:** Displays live status of each spot, revenue, and connection health on a local touchscreen interface (LVGL).
    *   **Network Manager:** Monitors "Heartbeats" from nodes to detect offline devices.
    *   **Control:** Synchronizes LED states (Red/Green) between Firebase and physical nodes.

### 2. Sensor Nodes (`ESP_module_full_test`)
*   **Role:** Endpoint Detector.
*   **Hardware:** ESP32, Presence Sensor (GPIO 13), WS2812B LED Strip, Piezo Buzzer.
*   **Key Responsibilities:**
    *   **Detection:** Monitors the physical parking spot sensor.
    *   **Feedback:** Controls WS2812B LEDs (Green = Free, Red = Occupied) based on commands from the Gateway.
    *   **Communication:** Sends sensor data + (Simulated) License Plate info to the Gateway via ESP-NOW.
    *   **Discovery:** Auto-discovers the Gateway via broadcast messages.

---

## 🛠️ Hardware Requirements

| Component | Quantity (Per Unit) | Notes |
| :--- | :--- | :--- |
| **ESP32 Dev Module** | 1 per Node + 1 for Gateway | Core controller |
| **TFT Touch Display** | 1 (Gateway Only) | ILI9341 / XPT2046 recommended |
| **WS2812B LED Strip** | 1 Strip (Nodes) | Visual occupancy indicator (8 pixels used) |
| **Presence Sensor** | 1 (Nodes) | IR, Ultrasonic, or Magnetic (High = Active) |
| **Piezo Buzzer** | 1 (Nodes) | Audio feedback |
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

---

## ⚙️ Configuration & Setup

### 1. Firebase Setup
The system requires a Firebase Realtime Database. configured with the following structure:
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
*   **Wi-Fi**: Update `WIFI_SSID` and `WIFI_PASSWORD`.
*   **Firebase**: Update `WEB_API_KEY`, `DATABASE_URL`, `USER_EMAIL`, `USER_PASS`.
*   **Boards**: The system pulls allowed MAC addresses from Firebase (`FB_CONFIG_MAC_ROOT`). Ensure your Node MAC addresses are registered there.

### 3. Sensor Node Configuration (`ESP_module_full_test.ino`)
*   **Wi-Fi**: Update `WIFI_SSID` (Must match Gateway for channel synchronization).
*   **ID**: Set `boardId` manually if not dynamically assigned (currently hardcoded as `2`).
*   **Pin Defs**: Ensure `WAKEUP_PIN`, `BUZZZER_PIN`, `PIN_WS2812B` match your wiring.

---

## 🚀 Usage

1.  **Power On**: Start the **Gateway** first. It will initialize the display and connect to Wi-Fi/Firebase.
2.  **Start Nodes**: Power on the **Sensor Nodes**. They will listen for the Gateway's broadcast or transmit data upon sensor triggers.
3.  **Monitoring**:
    *   **Local**: View the specific parking spot tabs on the Gateway's TFT screen.
    *   **Cloud**: Check the Firebase Console or connected App to see real-time updates.
4.  **Interaction**:
    *   **Park a Car**: Trigger the sensor on a Node -> Node sends "Occupied" -> Gateway updates Cloud & calculates start time -> Node LED turns RED.
    *   **Leave**: Sensor clears -> Node sends "Free" -> Gateway calculates duration & revenue -> Node LED turns GREEN.
    *   **LED Control**: Toggle the LED buttons on the Gateway UI to manually override Node LEDs.

---

## ⚠️ Notes
*   **MAC Address Security**: The Gateway strictly filters incoming ESP-NOW messages based on MAC addresses registered in Firebase. Unregistered devices are ignored.
*   **Demo Mode**: The Sensor Node currently generates a **random license plate** for demonstration purposes when triggered. In a real deployment, this would be replaced by an OCR camera module (like `ESPCAM`).

