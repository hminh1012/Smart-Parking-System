#include <esp_now.h>
#include <esp_wifi.h> // Cần thiết để thiết lập kênh Wi-Fi
#include <WiFi.h>
#include <vector>



// --- THƯ VIỆN & CẤU HÌNH WS2812B ---
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Cần thiết cho Adafruit Trinket 16 MHz
#endif

#define WAKEUP_PIN GPIO_NUM_13 // Chân cảm biến kích hoạt (chờ tín hiệu HIGH)
#define BUZZZER_PIN 15         // Chân Piezo Buzzer
#define PIN_WS2812B 14          // Chân LED báo hiệu nhận tín hiệu
#define NUM_PIXELS 8 // Số lượng LED trên dải
#define boardId 2             // ID của board này (1 hoặc 2)

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
// !! ĐỊA CHỈ MAC CỦA CÁC THIẾT BỊ !!
// CẬP NHẬT ĐỊA CHỈ NÀY PHÙ HỢP VỚI CÁC BOARD CỦA BẠN
// ------------------------------------------------
uint8_t macAddress[] = {0x3C, 0x8A, 0x1F, 0xAB, 0xF9, 0x34};

// ------------------------------------------------

uint8_t peerAddress[6]; // Địa chỉ MAC của board đối diện

// List of known masters
std::vector<std::vector<uint8_t>> masters;



// --- KHỞI TẠO WS2812B ---
Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
// ------------------------------


// Cấu trúc dùng để GỬI dữ liệu (Giống như code gốc của bạn)
typedef struct {
  int id;
  int status;
  char CarLicense[11];
  int readingId;
} struct_message;


// Cấu trúc dùng để NHẬN lệnh điều khiển LED
typedef struct led_message {
  int id;
  bool state; // true (BẬT), false (TẮT)
} led_message;

// Tạo biến cấu trúc để gửi
struct_message myData;

// Tạo biến cấu trúc để nhận
led_message incomingData;

// Thông tin Peer ESP-NOW
esp_now_peer_info_t peerInfo;

// Biến cho việc gửi dữ liệu
unsigned long previousMillis = 0;
// Biến cho việc gửi dữ liệu
unsigned long lastDebounceTime = 0;
const long debounceDelay = 1000; // 1 second debounce
unsigned int readingId = 0;

// --- HÀM HỖ TRỢ ---
// Thiết lập tất cả các pixel LED về một màu cụ thể
void setStripColor(uint32_t color) {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) { // Lặp qua từng pixel
    WS2812B.setPixelColor(pixel, color); // Thiết lập màu
  }
  WS2812B.show(); // Cập nhật dải LED
}
// ------------------------------


// ------------------------------------------------
// HÀM GỌI LẠI (CALLBACKS)
// ------------------------------------------------

// 1. Hàm gọi lại khi dữ liệu được GỬI
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Trạng thái gửi: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Gửi thành công" : "Gửi thất bại");
}

// 2. Hàm gọi lại khi dữ liệu được NHẬN
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

  // Kiểm tra xem độ dài dữ liệu có khớp với cấu trúc led_message hay không
  if (len == sizeof(incomingData)) {
    // Sao chép byte thô vào cấu trúc incomingData
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

    // In dữ liệu nhận được
    Serial.println("\n--- GÓI DỮ LIỆU ĐÃ NHẬN ---");
    Serial.print("Bytes nhận được: ");
    Serial.println(len);
    Serial.print("Từ Board ID: ");
    Serial.println(incomingData.id);
    Serial.print("Trạng thái Led: ");
    Serial.println(incomingData.state ? "ON (BẬT)" : "OFF (TẮT)");
    Serial.println("-----------------------------------");
    

  // --- LOGIC ĐIỀU KHIỂN WS2812B ---
    if (incomingData.state == true) { 
      Serial.println("Thiết lập dải LED thành MÀU ĐỎ");
      setStripColor(WS2812B.Color(255, 0, 0)); // Đặt tất cả thành ĐỎ 
    } else { 
      Serial.println("Thiết lập dải LED thành MÀU XANH LÁ");
      setStripColor(WS2812B.Color(0, 255, 0)); // Đặt tất cả thành XANH LÁ
    }
    // --- KẾT THÚC LOGIC MỚI ---


    // In địa chỉ MAC của người gửi
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
              recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
              recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    Serial.print("Từ MAC: ");
    Serial.println(macStr);
    Serial.println("-----------------------");
    Serial.println();

  } else {
    Serial.println("Độ dài dữ liệu nhận được không khớp!");
  }
}

// ------------------------------------------------
// THIẾT LẬP (SETUP)
// ------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000); // Đợi Serial bắt đầu
// --- KHỞI TẠO WS2812B ---
  WS2812B.begin();  // Khởi tạo đối tượng dải WS2812B 
  WS2812B.clear();
  setStripColor(WS2812B.Color(0, 255, 0)); // Đặt màu mặc định là XANH LÁ
  Serial.println("Dải LED đã được đặt thành XANH LÁ");

  pinMode(WAKEUP_PIN, INPUT);
  pinMode(BUZZZER_PIN, OUTPUT);
  digitalWrite(BUZZZER_PIN, LOW);
// ------------------------------

  // Thiết lập thiết bị là Trạm Wi-Fi
  WiFi.mode(WIFI_STA);
  
  // --- Channel Scanning Logic handled in Loop now ---
  // Default to Channel 1
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  // -------------------------------------

  // --- Xác định Board này ---
  uint8_t myMac[6];
  WiFi.macAddress(myMac);
  Serial.print("Địa chỉ MAC của tôi: ");
  char myMacStr[18];
  snprintf(myMacStr, sizeof(myMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
            myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  Serial.println(myMacStr);

  memcpy(peerAddress, macAddress, 6);
  

  // -----------------------------

  // Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Lỗi khi khởi tạo ESP-NOW");
    return;
  }

  // Đăng ký cả hai hàm gọi lại
  esp_now_register_send_cb(OnDataSent); // Cho việc gửi
  esp_now_register_recv_cb(OnDataRecv); // Cho việc nhận

  // Đăng ký peer (board đối diện)
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0; // Use current channel
  peerInfo.encrypt = false;

  // Thêm peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Thêm peer thất bại");
    return;
  }

  // Khởi tạo bộ tạo số ngẫu nhiên
  randomSeed(analogRead(0));
} led_message;

// Tạo biến cấu trúc để gửi
struct_message myData;

// Tạo biến cấu trúc để nhận
led_message incomingData;

// Thông tin Peer ESP-NOW
esp_now_peer_info_t peerInfo;

// Biến cho việc gửi dữ liệu
unsigned long previousMillis = 0;
// Biến cho việc gửi dữ liệu
unsigned long lastDebounceTime = 0;
const long debounceDelay = 1000; // 1 second debounce
unsigned int readingId = 0;

// --- HÀM HỖ TRỢ ---
// Thiết lập tất cả các pixel LED về một màu cụ thể
void setStripColor(uint32_t color) {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) { // Lặp qua từng pixel
    WS2812B.setPixelColor(pixel, color); // Thiết lập màu
  }
  WS2812B.show(); // Cập nhật dải LED
}
// ------------------------------


// ------------------------------------------------
// HÀM GỌI LẠI (CALLBACKS)
// ------------------------------------------------

// 1. Hàm gọi lại khi dữ liệu được GỬI
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Trạng thái gửi: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Gửi thành công" : "Gửi thất bại");
}

// 2. Hàm gọi lại khi dữ liệu được NHẬN
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
    return; // Don't process broadcast as LED command
  }

  // Kiểm tra xem độ dài dữ liệu có khớp với cấu trúc led_message hay không
  if (len == sizeof(incomingData)) {
    // Sao chép byte thô vào cấu trúc incomingData
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

    // In dữ liệu nhận được
    Serial.println("\n--- GÓI DỮ LIỆU ĐÃ NHẬN ---");
    Serial.print("Bytes nhận được: ");
    Serial.println(len);
    Serial.print("Từ Board ID: ");
    Serial.println(incomingData.id);
    Serial.print("Trạng thái Led: ");
    Serial.println(incomingData.state ? "ON (BẬT)" : "OFF (TẮT)");
    Serial.println("-----------------------------------");
    

  // --- LOGIC ĐIỀU KHIỂN WS2812B ---
    if (incomingData.state == true) { 
      Serial.println("Thiết lập dải LED thành MÀU ĐỎ");
      setStripColor(WS2812B.Color(255, 0, 0)); // Đặt tất cả thành ĐỎ 
    } else { 
      Serial.println("Thiết lập dải LED thành MÀU XANH LÁ");
      setStripColor(WS2812B.Color(0, 255, 0)); // Đặt tất cả thành XANH LÁ
    }
    // --- KẾT THÚC LOGIC MỚI ---


    // In địa chỉ MAC của người gửi
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
              recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
              recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    Serial.print("Từ MAC: ");
    Serial.println(macStr);
    Serial.println("-----------------------");
    Serial.println();

  } else {
    Serial.println("Độ dài dữ liệu nhận được không khớp!");
  }
}

// ------------------------------------------------
// THIẾT LẬP (SETUP)
// ------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000); // Đợi Serial bắt đầu
// --- KHỞI TẠO WS2812B ---
  WS2812B.begin();  // Khởi tạo đối tượng dải WS2812B 
  WS2812B.clear();
  setStripColor(WS2812B.Color(0, 255, 0)); // Đặt màu mặc định là XANH LÁ
  Serial.println("Dải LED đã được đặt thành XANH LÁ");

  pinMode(WAKEUP_PIN, INPUT);
  pinMode(BUZZZER_PIN, OUTPUT);
  digitalWrite(BUZZZER_PIN, LOW);
// ------------------------------

  // Thiết lập thiết bị là Trạm Wi-Fi
  WiFi.mode(WIFI_STA);
  
  // --- LOGIC THIẾT LẬP KÊNH (Đã thêm) ---
  int32_t channel = getWiFiChannel(WIFI_SSID);
  if (channel != 0) {
    Serial.printf("Tìm thấy kênh WiFi: %d\n", channel);
    // Thiết lập kênh cho ESP32
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  } else {
    Serial.println("Cảnh báo: Không thể xác định kênh WiFi. Đang sử dụng kênh mặc định.");
    channel = 0; // Đảm bảo kênh là 0 nếu không tìm thấy
  }
  // -------------------------------------

  // --- Xác định Board này ---
  uint8_t myMac[6];
  WiFi.macAddress(myMac);
  Serial.print("Địa chỉ MAC của tôi: ");
  char myMacStr[18];
  snprintf(myMacStr, sizeof(myMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
            myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  Serial.println(myMacStr);

  memcpy(peerAddress, macAddress, 6);
  

  // -----------------------------

  // Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Lỗi khi khởi tạo ESP-NOW");
    return;
  }

  // Đăng ký cả hai hàm gọi lại
  esp_now_register_send_cb(OnDataSent); // Cho việc gửi
  esp_now_register_recv_cb(OnDataRecv); // Cho việc nhận

  // Đăng ký peer (board đối diện)
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = channel; // Thiết lập kênh peer
  peerInfo.encrypt = false;

  // Thêm peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Thêm peer thất bại");
    return;
  }

  // Khởi tạo bộ tạo số ngẫu nhiên
  randomSeed(analogRead(0));
}

// ------------------------------------------------
// VÒNG LẶP (LOOP)
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
