#include <esp_now.h>
#include <esp_wifi.h> // Required for Wi-Fi channel setup
#include <WiFi.h>
#include <vector>



// --- WS2812B LIBRARY & CONFIGURATION ---
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for Adafruit Trinket 16 MHz
#endif.

#define WAKEUP_PIN GPIO_NUM_13 // Sensor trigger pin (waits for HIGH signal)
#define BUZZZER_PIN 15         // Piezo Buzzer pin
#define PIN_WS2812B 14          // LED indicator pin for signal reception
#define NUM_PIXELS 8 // Number of LEDs on the strip
#define boardId 2             // ID of this board (1 or 2)

// ------------------------------------

// --- CONFIGURATION ---
String received_ssid = ""; // Dynamic SSID from Gateway
bool master_found = false;
int current_channel = 1;
unsigned long last_channel_hop = 0;
const int HOP_INTERVAL = 2000; // Time (ms) to stay on a channel while scanning

// ------------------------------------
// ------------------------------------


// ------------------------------------------------
// !! DEVICE MAC ADDRESSES !!
// UPDATE THESE ADDRESSES TO MATCH YOUR BOARDS
// ------------------------------------------------
uint8_t macAddress[] = {0x3C, 0x8A, 0x1F, 0xAB, 0xF9, 0x34};

// ------------------------------------------------

uint8_t peerAddress[6]; // MAC address of the peer board

// List of known masters
std::vector<std::vector<uint8_t>> masters;



// --- WS2812B INITIALIZATION ---
Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
// ------------------------------


// Structure for SENDING data (Same as your original code)
typedef struct {
  int id;
  int status;
  char CarLicense[11];
  int readingId;
} struct_message;


// Structure for RECEIVING LED control commands
typedef struct led_message {
  int id;
  bool state; // true (ON), false (OFF)
} led_message;

// Create structure variable for sending
struct_message myData;

// Create structure variable for receiving
led_message incomingData;

// ESP-NOW Peer info
esp_now_peer_info_t peerInfo;

// Variables for data sending
unsigned long previousMillis = 0;
// Variables for data sending
unsigned long lastDebounceTime = 0;
const long debounceDelay = 1000; // 1 second debounce
unsigned int readingId = 0;

// --- HELPER FUNCTIONS ---
// Set all LED pixels to a specific color
void setStripColor(uint32_t color) {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) { // Loop through each pixel
    WS2812B.setPixelColor(pixel, color); // Set color
  }
  WS2812B.show(); // Update LED strip
}
// ------------------------------


// ------------------------------------------------
// CALLBACKS
// ------------------------------------------------

// 1. Callback when data is SENT
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send successful" : "Send failed");
}

// 2. Callback when data is RECEIVED
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataBytes, int len) {
  
  // Check for broadcast message
  if (recv_info->des_addr[0] == 0xFF && recv_info->des_addr[1] == 0xFF && 
      recv_info->des_addr[2] == 0xFF && recv_info->des_addr[3] == 0xFF && 
      recv_info->des_addr[4] == 0xFF && recv_info->des_addr[5] == 0xFF) {
    
    // Parse Payload for "DISCOVER_MASTER"
    String msg = "";
    for(int i=0; i<len; i++) msg += (char)incomingDataBytes[i];

    if (msg.startsWith("DISCOVER_MASTER")) {
      Serial.println(">> Broadcast Received: " + msg);
      
      // Extract SSID if present
      int splitIndex = msg.indexOf(':');
      if (splitIndex != -1) {
        String new_ssid = msg.substring(splitIndex + 1);
        if (new_ssid.length() > 0 && new_ssid != received_ssid) {
           received_ssid = new_ssid;
           master_found = true;
           Serial.println(">> TARGET SSID UPDATED: " + received_ssid);
           // We are currently on the correct channel because we heard the packet!
           // No need to change channel.
        }
      }
    }

    Serial.printf("Broadcast received from: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                  recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    // Check if already known
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

      // Register as peer
      esp_now_peer_info_t newPeerInfo = {};
      memcpy(newPeerInfo.peer_addr, recv_info->src_addr, 6);
      newPeerInfo.channel = WiFi.channel(); // Assuming same channel
      newPeerInfo.encrypt = false;

      if (esp_now_add_peer(&newPeerInfo) == ESP_OK) {
        Serial.println("Master registered successfully.");
        // Optionally update the primary peerAddress to the latest master
        memcpy(peerAddress, recv_info->src_addr, 6);
      } else {
        Serial.println("Failed to register Master.");
      }
    }
    return; // Don't process broadcast as LED command
  }

  // Check if data length matches led_message structure
  if (len == sizeof(incomingData)) {
    // Copy raw bytes into incomingData structure
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

    // Print received data
    Serial.println("\n--- RECEIVED DATA PACKET ---");
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("From Board ID: ");
    Serial.println(incomingData.id);
    Serial.print("LED State: ");
    Serial.println(incomingData.state ? "ON" : "OFF");
    Serial.println("-----------------------------------");
    

  // --- WS2812B CONTROL LOGIC ---
    if (incomingData.state == true) { 
      Serial.println("Setting LED strip to RED");
      setStripColor(WS2812B.Color(255, 0, 0)); // Set all to RED 
    } else { 
      Serial.println("Setting LED strip to GREEN");
      setStripColor(WS2812B.Color(0, 255, 0)); // Set all to GREEN
    }
    // --- END NEW LOGIC ---


    // Print sender MAC address
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
              recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
              recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    Serial.print("From MAC: ");
    Serial.println(macStr);
    Serial.println("-----------------------");
    Serial.println();

  } else {
    Serial.println("Received data length does not match!");
  }
}

// ------------------------------------------------
// SETUP
// ------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for Serial to start
// --- WS2812B INITIALIZATION ---
  WS2812B.begin();  // Initialize WS2812B strip object 
  WS2812B.clear();
  setStripColor(WS2812B.Color(0, 255, 0)); // Set default color to GREEN
  Serial.println("LED strip has been set to GREEN");

  pinMode(WAKEUP_PIN, INPUT);
  pinMode(BUZZZER_PIN, OUTPUT);
  digitalWrite(BUZZZER_PIN, LOW);
// ------------------------------

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // --- Channel Scanning Logic handled in Loop now ---
  // Default to Channel 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // -------------------------------------

  // --- Identify this Board ---
  uint8_t myMac[6];
  WiFi.macAddress(myMac);
  Serial.print("My MAC Address: ");
  char myMacStr[18];
  snprintf(myMacStr, sizeof(myMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
            myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  Serial.println(myMacStr);

  memcpy(peerAddress, macAddress, 6);
  

  // -----------------------------

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register both callbacks
  esp_now_register_send_cb(OnDataSent); // For sending
  esp_now_register_recv_cb(OnDataRecv); // For receiving

  // Register peer (the peer board)
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0; // Use current channel
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Initialize random number generator
  randomSeed(analogRead(0));
}



// ------------------------------------------------
// LOOP
// ------------------------------------------------
// --- LOOP ---
void loop() {
  // 1. Channel Hopping (Scanning for Master)
  if (!master_found) {
    if (millis() - last_channel_hop > HOP_INTERVAL) {
      last_channel_hop = millis();
      current_channel++;
      if (current_channel > 13) current_channel = 1;
      
      Serial.printf("Scanning for Master on Channel %d...\n", current_channel);
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
    }
  }
  
  // 2. Non-blocking Buzzer Logic
  static bool buzzerActive = false;
  static unsigned long buzzerOffTime = 0;

  if (buzzerActive && millis() > buzzerOffTime) {
    digitalWrite(BUZZZER_PIN, LOW);
    buzzerActive = false;
  }

  // 3. Sensor & Data Sending Logic
  int reading = digitalRead(WAKEUP_PIN);

  if (reading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = millis();

      // Trigger Buzzer Non-blocking
      digitalWrite(BUZZZER_PIN, HIGH);
      buzzerActive = true;
      buzzerOffTime = millis() + 1000; // Buzz for 1 second

      // Prepare Data
      myData.id = boardId; // Use fixed ID
      myData.status = random(2); // Random 0 or 1 for demo

      // --- Random License Plate (Demo) ---
      int province = 43; 
      char series = random('A', 'Z' + 1);
      int num1 = random(100, 1000); 
      int num2 = random(0, 100);    

      snprintf(myData.CarLicense, 10, "%d%c-%03d.%02d",
                province, series, num1, num2);
      
      myData.readingId = readingId++;

      // Send Data
      esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));

      Serial.print("Sending Data (ID: ");
      Serial.print(myData.readingId);
      Serial.print("): ");
      Serial.println(result == ESP_OK ? "Success" : "Failed");
    }
  }

  // 4. Heartbeat Logic
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    // Keep connection alive / update status
    esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));
  }
  
  // Yield
  delay(1); 
}
