/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Adapted version: send random data instead of BME280 sensor readings
*********/
#include <esp_now.h>
#include <WiFi.h>

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1
// Replace with your receiver's MAC address
uint8_t receiverAddress[] = {0x3C, 0x8A, 0x1F, 0xAB, 0xF9, 0x34};

// Structure to send data, must match the receiver structure
typedef struct {
    int id;
    int status;
    char CarLicense[10];
    int readingId;
} struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long previousMillis = 0;   // Stores last time data was published
const long interval = 1000;        // Interval at which to publish random readings

unsigned int readingId = 0;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Packet to: ");
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Seed random generator with analog input
  randomSeed(analogRead(0));
}
 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Set values to send (random values instead of BME280)
    myData.id = BOARD_ID;
    // Random status between "yes" and "no"
    myData.status = random(2);


    // Random CarLicense between "mot", "hai", "ba"
    int carChoice = random(3);
    if (carChoice == 0) {
      strcpy(myData.CarLicense, "mot");
    } else if (carChoice == 1) {
      strcpy(myData.CarLicense, "hai");
    } else {
      strcpy(myData.CarLicense, "ba");
    }

    myData.readingId = readingId++;
     
    // Send data
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.print("Sending data (Reading ID: ");
    Serial.print(myData.readingId);
    Serial.print(", status: ");
    Serial.print(myData.status);
    Serial.print(", CarLicense: ");
    Serial.print(myData.CarLicense);
    Serial.print(": ");
    Serial.println(result == ESP_OK ? "Sent" : "Failed to send");
  }
}
