/*********
  Merged ESP-NOW Transceiver Code
  - Sends "struct_message" every 1 second
  - Receives "struct_message" from its peer
  
  - FIXED CALLBACK SIGNATURES (Oct 28, 2025)
*********/

#include <esp_now.h>
#include <WiFi.h>

// ------------------------------------------------
// !! IMPORTANT !!
// UPDATE THESE MAC ADDRESSES TO MATCH YOUR BOARDS
// ------------------------------------------------
uint8_t macAddress1[] = {0x3C, 0x8A, 0x1F, 0xAB, 0xF9, 0x34};
uint8_t macAddress2[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02};
// ------------------------------------------------

uint8_t peerAddress[6]; // MAC Address of the *other* board
int boardId;             // This board's ID (1 or 2)

// This structure will be used for *both* sending and receiving
// It's from your first "sender" code.
typedef struct {
  int id;
  int status;
  char CarLicense[11];
  int readingId;
} struct_message;

// Create one struct for sending data
struct_message myData;

// Create one struct for receiving data
struct_message incomingData;

// ESP-NOW peer info
esp_now_peer_info_t peerInfo;

// Variables for sending data (from your sender code)
unsigned long previousMillis = 0;
const long interval = 5000; // Send data every 1 second
unsigned int readingId = 0;


// ------------------------------------------------
// CALLBACKS (Event-driven functions)
// ------------------------------------------------

// 1. Callback when data is SENT (from your sender code)
// FIXED: Changed signature from (const uint8_t*, ...) to (const wifi_tx_info_t*, ...)
//        to match your compiler's expectation.
// NOTE: The 'mac_addr' parameter in this version is not the MAC address,
//       so we can't print it. We'll just print the status.
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 2. Callback when data is RECEIVED (from your receiver code)
// FIXED: Changed signature from (const uint8_t*, ...) to (const esp_now_recv_info*, ...)
//        and updated body to use 'recv_info->src_addr'
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataBytes, int len) {
  
  // Check if the data length matches our structure
  if (len == sizeof(incomingData)) {
    // Copy the raw bytes into our "incomingData" struct
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

    // Print the received data
    Serial.println();
    Serial.println("--- PACKET RECEIVED ---");
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("From Board ID: ");
    Serial.println(incomingData.id);
    Serial.print("Status: ");
    Serial.println(incomingData.status);
    Serial.print("CarLicense: ");
    Serial.println(incomingData.CarLicense);
    Serial.print("Reading ID: ");
    Serial.println(incomingData.readingId);

    // Print the sender's MAC address (FIXED: using recv_info->src_addr)
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    Serial.print("From MAC: ");
    Serial.println(macStr);
    Serial.println("-----------------------");
    Serial.println();

  } else {
    Serial.println("Received data length mismatch!");
  }
}

// ------------------------------------------------
// SETUP
// ------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to start

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // --- Identify This Board ---
  uint8_t myMac[6];
  WiFi.macAddress(myMac);
  Serial.print("My MAC Address: ");
  char myMacStr[18];
  snprintf(myMacStr, sizeof(myMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  Serial.println(myMacStr);
  
  // Check if we are Board 1 or Board 2
  if (memcmp(myMac, macAddress1, 6) == 0) {
    boardId = 1;
    memcpy(peerAddress, macAddress2, 6); // Set peer to Board 2
    Serial.println("I am Board 1. Sending to Board 2.");
  } else {
    // Assume we are Board 2 (or default)
    boardId = 2;
    memcpy(peerAddress, macAddress1, 6); // Set peer to Board 1
    Serial.println("I am Board 2. Sending to Board 1.");
  }
  // -----------------------------

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register *both* callbacks
  esp_now_register_send_cb(OnDataSent); // For sending
  esp_now_register_recv_cb(OnDataRecv); // For receiving

  // Register peer (the *other* board)
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Seed random generator
  randomSeed(analogRead(0));
}

// ------------------------------------------------
// LOOP
// ------------------------------------------------
void loop() {
  // This part of the loop is the "sender" logic
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Set values to send
    myData.id = boardId; // Use our dynamic boardId
    myData.status = random(2); // Random 0 or 1

// --- Generate random Vietnamese license plate ---
    int province = 43; // Using '43' from your Da Nang example
    char series = random('A', 'Z' + 1); // Random letter 'A' through 'Z'
    int num1 = random(100, 1000); // Random number 100-999
    int num2 = random(0, 100);    // Random number 0-99

    // Format the string: "43A-230.42"
    // snprintf is a safe way to create formatted strings
    snprintf(myData.CarLicense, 10, "%d%c-%03d.%02d",
             province,
             series,
             num1,
             num2);
    // ------------------------------------------------

    myData.readingId = readingId++;

    // Send the data
    esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));

    // Print a quick status of the send attempt
    Serial.print("Sending data (Reading ID: ");
    Serial.print(myData.readingId);
    Serial.print("): ");
    Serial.println(result == ESP_OK ? "Sent" : "Failed to send");
  }

  // The "receiver" part requires no code here.
  // It works entirely in the background via the OnDataRecv callback.
}