#include <esp_now.h>
#include <WiFi.h>

// Structure to match sender's message
typedef struct led_message {
  int id;
  bool state; // true for ON, false for OFF
} led_message;

// Callback function when data is received
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  led_message myData;

  // Ensure we don't copy more bytes than expected
  if (len == sizeof(myData)) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("ID: ");
    Serial.println(myData.id);
    Serial.print("LED state: ");
    Serial.println(myData.state ? "ON" : "OFF");
  } else {
    Serial.println("Received data length mismatch!");
  }

  // Optional: print MAC address of sender
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
  Serial.print("From MAC: ");
  Serial.println(macStr);
  Serial.println();
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000); // Give time for serial to start

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println("ESP32 Receiver Ready.");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing here — receiver waits for messages
}
