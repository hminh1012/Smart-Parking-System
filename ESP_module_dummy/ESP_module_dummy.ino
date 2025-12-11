#include <esp_now.h>
#include <esp_wifi.h> // Required for Wi-Fi channel setup
#include <WiFi.h>
#include <vector>



// --- WS2812B LIBRARY & CONFIGURATION ---
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for Adafruit Trinket 16 MHz
#endif

#define PIN_WS2812B 14 // ESP32 pin connected to WS2812B
#define NUM_PIXELS 8 // Number of LEDs on the strip
#define boardId 3;             // ID of this board (1 or 2)
// ------------------------------------

// --- WIFI CONFIGURATION (IMPORTANT) ---
// Please replace with your actual Wi-Fi SSID
String received_ssid = ""; // Dynamic SSID from Gateway
unsigned long lastChannelScan = 0;


// Function to find Wi-Fi SSID channel (Required for ESP-NOW)
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0; // Return 0 if not found
}
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
const long interval = 5000; // Send data every 5 seconds
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
    
    // --- PARSE SSID FROM BROADCAST ---
    // Msg format: "DISCOVER_MASTER:MySSID"
    String msg = "";
    for(int i=0; i<len; i++) msg += (char)incomingDataBytes[i];
    
    if (msg.startsWith("DISCOVER_MASTER:")) {
       String extracted = msg.substring(16);
       if (received_ssid == "") {
          received_ssid = extracted;
          Serial.println(">> SSID DISCOVERED: " + received_ssid);
          Serial.printf(">> LOCKED CHANNEL: %d\n", WiFi.channel());
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
// ------------------------------

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // --- CHANNEL SETUP LOGIC ---
  // We will scan channels in loop() if no SSID yet
  Serial.println("Waiting for DISCOVER_MASTER Broadcast to get SSID...");
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
  peerInfo.channel = 0; // Set peer channel (0 = current/unknown channel)
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
void loop() {
  // Data SENDING logic
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Set values to send
    myData.id = boardId; // Use our dynamic Board ID
    myData.status = random(2); // Random 0 or 1 (Example: Parking status)

// --- Generate random license plate (Example: Vietnam) ---
    int province = 43; // Province code '43' (Example: Da Nang)
    char series = random('A', 'Z' + 1); // Random letter
    int num1 = random(100, 1000); // Random 3-digit number
    int num2 = random(0, 100);    // Random 2-digit number

    // Format string: "43A-230.42"
    snprintf(myData.CarLicense, 10, "%d%c-%03d.%02d",
              province,
              series,
              num1,
              num2);
// ------------------------------------------------

    myData.readingId = readingId++;

    // Send data
    esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));

    // Print send status
    Serial.print("Sending data (Reading ID: ");
    Serial.print(myData.readingId);
    Serial.print("): ");
    Serial.println(result == ESP_OK ? "Sent" : "Send failed");
  }

  // Receiving logic works entirely in the background via OnDataRecv callback.

  // --- CHANNEL SCANNING ---
  if (received_ssid == "") {
    if (millis() - lastChannelScan > 200) {
      lastChannelScan = millis();
      int newCh = (WiFi.channel() % 13) + 1;
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(newCh, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      // Serial.printf("Scanning Ch: %d\n", newCh); 
    }
  }
}
