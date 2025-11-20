#include <esp_now.h>
#include <esp_wifi.h> // Cần thiết để thiết lập kênh Wi-Fi
#include <WiFi.h>



// --- THƯ VIỆN & CẤU HÌNH WS2812B ---
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Cần thiết cho Adafruit Trinket 16 MHz
#endif

#define WAKEUP_PIN GPIO_NUM_14 // Chân cảm biến kích hoạt (chờ tín hiệu HIGH)
#define BUZZZER_PIN 17         // Chân Piezo Buzzer
#define PIN_WS2812B 16          // Chân LED báo hiệu nhận tín hiệu
#define NUM_PIXELS 8 // Số lượng LED trên dải
// ------------------------------------

// --- CẤU HÌNH WIFI (QUAN TRỌNG) ---
// Vui lòng thay thế bằng SSID Wi-Fi thực tế của bạn
constexpr char WIFI_SSID[] = "Bubuchacha";

// Hàm tìm kiếm kênh của SSID Wi-Fi (Cần thiết cho ESP-NOW)
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0; // Trả về 0 nếu không tìm thấy
}
// ------------------------------------


// ------------------------------------------------
// !! ĐỊA CHỈ MAC CỦA CÁC THIẾT BỊ !!
// CẬP NHẬT ĐỊA CHỈ NÀY PHÙ HỢP VỚI CÁC BOARD CỦA BẠN
// ------------------------------------------------
uint8_t macAddress[] = {0x3C, 0x8A, 0x1F, 0xAB, 0xF9, 0x34};

// ------------------------------------------------

uint8_t peerAddress[6]; // Địa chỉ MAC của board đối diện
#define boardId 3;             // ID của board này (1 hoặc 2)


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
void loop() {
  // Phần logic GỬI dữ liệu
  int reading = digitalRead(WAKEUP_PIN);

  if (reading == HIGH) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = millis();

      // Trigger Buzzer
      digitalWrite(BUZZZER_PIN, HIGH);
      delay(200);
      digitalWrite(BUZZZER_PIN, LOW);

      // Đặt giá trị để gửi
      myData.id = boardId; // Sử dụng Board ID động của chúng ta
      myData.status = random(2); // Ngẫu nhiên 0 hoặc 1 (Ví dụ: Trạng thái đậu xe)

      // --- Tạo biển số xe ngẫu nhiên (Ví dụ: Việt Nam) ---
      int province = 43; // Mã tỉnh '43' (Ví dụ: Đà Nẵng)
      char series = random('A', 'Z' + 1); // Chữ cái ngẫu nhiên
      int num1 = random(100, 1000); // Số ngẫu nhiên 3 chữ số
      int num2 = random(0, 100);    // Số ngẫu nhiên 2 chữ số

      // Định dạng chuỗi: "43A-230.42"
      snprintf(myData.CarLicense, 10, "%d%c-%03d.%02d",
                province,
                series,
                num1,
                num2);
      // ------------------------------------------------

      myData.readingId = readingId++;

      // Gửi dữ liệu
      esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));

      // In trạng thái gửi
      Serial.print("Đang gửi dữ liệu (Reading ID: ");
      Serial.print(myData.readingId);
      Serial.print("): ");
      Serial.println(result == ESP_OK ? "Đã gửi" : "Gửi thất bại");
    }
  }

  // Phần nhận hoạt động hoàn toàn ở chế độ nền thông qua hàm gọi lại OnDataRecv.
}
