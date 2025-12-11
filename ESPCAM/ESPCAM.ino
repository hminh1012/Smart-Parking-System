#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <vector>
#include <Adafruit_NeoPixel.h>
#include "pitches.h"

// --- PIN DEFINITIONS ---
#define PIN_WS2812B   14  // LED Strip
#define PIN_PIR       13  // PIR Motion Sensor
#define PIN_BUZZER    15  // Buzzer (GPIO15)

#define NUM_PIXELS    8
#define BOARD_ID      3

// --- GLOBAL VARIABLES ---
Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

String received_ssid = "";
unsigned long lastChannelScan = 0;
uint8_t peerAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast default
std::vector<std::vector<uint8_t>> masters;

bool motionDetected = false;
bool firstMessageSent = false;
bool isOccupied = false; // Track current parking status
bool receivedLedOn = false; // Track received LED command state
int pirState = LOW;
unsigned int readingId = 0;

// --- PIR DEBOUNCE (Reduce Sensitivity) ---
unsigned long pirHighStart = 0;        // When PIR first went HIGH
bool pirPendingConfirmation = false;   // Waiting for confirmation
const unsigned long PIR_CONFIRM_MS = 500; // Must stay HIGH for 500ms to confirm

// --- MESSAGE STRUCTURES ---
typedef struct {
  int id;
  int status;
  char CarLicense[11];
  int readingId;
} struct_message;

typedef struct {
  int id;
  bool state;
} led_message;

struct_message myData;
led_message incomingData;
esp_now_peer_info_t peerInfo;

// --- HELPER FUNCTIONS ---
void setStripColor(uint32_t color) {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    WS2812B.setPixelColor(pixel, color);
  }
  WS2812B.show();
}

void updateLedColor() {
  // LED Logic:
  // RED if occupied
  // BLUE if available AND received LED state is ON
  // GREEN if available (default)
  if (isOccupied) {
    Serial.println("LED -> RED (Occupied)");
    setStripColor(WS2812B.Color(255, 0, 0)); // RED
  } else if (receivedLedOn) {
    Serial.println("LED -> BLUE (Available + LED ON)");
    setStripColor(WS2812B.Color(0, 0, 255)); // BLUE
  } else {
    Serial.println("LED -> GREEN (Available)");
    setStripColor(WS2812B.Color(0, 255, 0)); // GREEN
  }
}

// --- MELODY FOR BUZZER ---
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void triggerBuzzer() {
  Serial.println("Playing Buzzer Melody...");
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(PIN_BUZZER, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(PIN_BUZZER);
  }
  Serial.println("Buzzer Melody Done.");
}

void sendMotionAlert() {
  myData.id = BOARD_ID;
  myData.status = 1; // Occupied
  myData.CarLicense[0] = '\0'; // Null/Empty license
  myData.readingId = readingId++;

  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&myData, sizeof(myData));
  Serial.printf("Sent OCCUPIED: (License: NULL) - %s\n", 
                result == ESP_OK ? "Success" : "Failed");
}

void sendOccupiedMessage() {
  // Generate Random License Plate (Vietnam Format)
  int province = 43;
  char series = random('A', 'Z' + 1);
  int num1 = random(100, 1000);
  int num2 = random(0, 100);

  myData.id = BOARD_ID;
  myData.status = 1; // Occupied
  snprintf(myData.CarLicense, 11, "%d%c-%03d.%02d", province, series, num1, num2);
  myData.readingId = readingId++;

  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&myData, sizeof(myData));
  Serial.printf("Sent OCCUPIED: (License: %s) - %s\n", 
                myData.CarLicense,
                result == ESP_OK ? "Success" : "Failed");
}

void sendAvailableMessage() {
  myData.id = BOARD_ID;
  myData.status = 0; // Available
  myData.CarLicense[0] = '\0'; // Null/Empty license
  myData.readingId = readingId++;

  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&myData, sizeof(myData));
  Serial.printf("Sent AVAILABLE: (License: NULL) - %s\n", 
                result == ESP_OK ? "Success" : "Failed");
}

void sendFirstMessage() {
  myData.id = BOARD_ID;
  myData.status = 0; // Available
  strncpy(myData.CarLicense, "First Msg", 10); // "First Message" truncated to fit
  myData.CarLicense[10] = '\0';
  myData.readingId = readingId++;

  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&myData, sizeof(myData));
  Serial.printf("Sent FIRST MESSAGE: (License: %s) - %s\n", 
                myData.CarLicense,
                result == ESP_OK ? "Success" : "Failed");
}

// --- ESP-NOW CALLBACKS ---
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  // Optional: Log send status
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataBytes, int len) {
  // 1. Handle Broadcast (Gateway Discovery)
  if (recv_info->des_addr[0] == 0xFF && recv_info->des_addr[1] == 0xFF &&
      recv_info->des_addr[2] == 0xFF && recv_info->des_addr[3] == 0xFF &&
      recv_info->des_addr[4] == 0xFF && recv_info->des_addr[5] == 0xFF) {
    
    // Check if already registered
    bool known = false;
    for (const auto& master : masters) {
      if (memcmp(master.data(), recv_info->src_addr, 6) == 0) {
        known = true;
        break;
      }
    }

    if (!known) {
      Serial.println("New Master detected! Registering...");
      std::vector<uint8_t> newMaster(recv_info->src_addr, recv_info->src_addr + 6);
      masters.push_back(newMaster);

      esp_now_peer_info_t newPeerInfo = {};
      memcpy(newPeerInfo.peer_addr, recv_info->src_addr, 6);
      newPeerInfo.channel = WiFi.channel();
      newPeerInfo.encrypt = false;

      if (esp_now_add_peer(&newPeerInfo) == ESP_OK) {
        Serial.println("Master registered successfully.");
        memcpy(peerAddress, recv_info->src_addr, 6);
      }
    }

    // Parse SSID from broadcast
    String msg = "";
    for (int i = 0; i < len; i++) msg += (char)incomingDataBytes[i];
    if (msg.startsWith("DISCOVER_MASTER:")) {
      if (received_ssid == "") {
        received_ssid = msg.substring(16);
        
        // Lock WiFi channel to the gateway's channel
        int gatewayChannel = WiFi.channel();
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(gatewayChannel, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);
        
        Serial.println(">> CONNECTED TO GATEWAY: " + received_ssid);
        Serial.printf(">> WiFi Channel locked to: %d\n", gatewayChannel);
      }
    }
    return;
  }

  // 2. Handle LED Commands
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
    
    Serial.println("\n--- LED COMMAND RECEIVED ---");
    Serial.print("From Board ID: ");
    Serial.println(incomingData.id);
    Serial.print("LED State: ");
    Serial.println(incomingData.state ? "ON" : "OFF");
    Serial.println("----------------------------");
    
    // Update received LED state and refresh LED color
    receivedLedOn = incomingData.state;
    updateLedColor();
  }
}

// ------------------------------------------------
// SETUP
// ------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // --- Hardware Init ---
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  WS2812B.begin();
  WS2812B.clear();
  setStripColor(WS2812B.Color(0, 0, 255)); // Blue = Booting
  Serial.println("System Booting...");

  // --- WiFi & ESP-NOW Init ---
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register broadcast peer initially
  memset(peerInfo.peer_addr, 0xFF, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  randomSeed(analogRead(0));
  
  setStripColor(WS2812B.Color(0, 255, 0)); // Green = Ready
  Serial.println("System Ready. Waiting for Gateway...");
}

// ------------------------------------------------
// LOOP
// ------------------------------------------------
void loop() {
  // --- 1. CHANNEL SCANNING (If not connected to Gateway) ---
  if (received_ssid == "") {
    if (millis() - lastChannelScan > 200) {
      lastChannelScan = millis();
      int newCh = (WiFi.channel() % 13) + 1;
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(newCh, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
    }
    return; // Don't process motion until connected
  }

  // --- 1.5 SEND FIRST MESSAGE ---
  if (!firstMessageSent) {
    Serial.println(">>> SENDING FIRST MESSAGE <<<");
    sendFirstMessage();
    firstMessageSent = true;
  }

  // --- 2. MOTION DETECTION LOGIC (Toggle on each motion, with debounce) ---
  pirState = digitalRead(PIN_PIR);

  if (pirState == HIGH) {
    if (!pirPendingConfirmation) {
      // First time HIGH detected, start the confirmation timer
      pirHighStart = millis();
      pirPendingConfirmation = true;
    } else if (!motionDetected && (millis() - pirHighStart >= PIR_CONFIRM_MS)) {
      // PIR has been HIGH for the confirmation period -> CONFIRMED MOTION
      Serial.println(">>> MOTION DETECTED (Confirmed) <<<");
      motionDetected = true;

      // Toggle parking status on each detection
      if (!isOccupied) {
        // Was available -> Now occupied (with random license)
        sendOccupiedMessage();
        isOccupied = true;
      } else {
        // Was occupied -> Now available (with null license)
        sendAvailableMessage();
        isOccupied = false;
      }

      // Update LED color based on new status
      updateLedColor();

      // Trigger Buzzer Melody
      triggerBuzzer();
    }
  } else {
    // PIR went LOW -> Reset everything
    if (motionDetected) {
      Serial.println("Motion sensor reset, ready for next detection.");
    }
    pirPendingConfirmation = false;
    motionDetected = false;
  }
}
