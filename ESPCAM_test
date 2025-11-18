/****************************************************************
 * Merged ESP32-CAM, Buzzer, ESP-NOW Transceiver, and Deep Sleep Trigger Code
 * Đã thêm Logic Thiết lập Kênh Wi-Fi
 ***************************************************************/

// --- Cấu hình Thư viện ---
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "FS.h"
#include "SD_MMC.h"
#include "EEPROM.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Thư viện MỚI để điều khiển kênh Wi-Fi
#include "pitches.h" // PHẢI có file này trong thư mục sketch

// --- Cấu hình MẠNG ---
constexpr char WIFI_SSID[] = "Bubuchacha"; // <-- HÃY THAY ĐỔI TÊN MẠNG NÀY
// ----------------------

// --- Định nghĩa chân (Pins) ---
#define WAKEUP_PIN GPIO_NUM_14 // Chân cảm biến kích hoạt (chờ tín hiệu HIGH)
#define BUZZZER_PIN 16         // Chân Piezo Buzzer
#define STATUS_LED_PIN 4       // Chân LED báo hiệu nhận tín hiệu

// EEPROM
#define EEPROM_SIZE 1
unsigned int pictureCount = 0;

// Pin definitions for CAMERA_MODEL_AI_THINKER (Giữ nguyên)
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// --- Cấu hình ESP-NOW ---
uint8_t peerAddress[6] = {0x3C, 0x8A, 0x1F, 0xAB, 0xF9, 0x34}; // <-- MAC Peer CẦN CẬP NHẬT

// Cấu trúc dữ liệu GỬI đi (Thông báo kích hoạt)
typedef struct {
  int id;
  int status;
  char CarLicense[11];
  int readingId;
} struct_message;

// Cấu trúc dữ liệu NHẬN về (Tín hiệu LED từ Peer)
typedef struct {
  int id;
  bool state; 
} led_message;

struct_message myData;
led_message incomingData;
esp_now_peer_info_t peerInfo;
const long LISTEN_DURATION_MS = 3000; // 3 giây lắng nghe

// --- Hàm tìm kiếm kênh Wi-Fi (Đã thêm) ---
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
// ------------------------------------------

// --- Hàm chức năng Buzzer (Giữ nguyên) ---
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void playAlarm() {
  Serial.println("Playing Alarm...");
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZZER_PIN, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZZER_PIN);
  }
}

// --- Hàm chức năng Camera, SD, EEPROM (Giữ nguyên) ---
void configESPCamera() {
  camera_config_t config;
  // ... (cấu hình chân) ...
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA; 
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
  }
}

void initMicroSDCard() {
  Serial.println("Mounting MicroSD Card");
  if (!SD_MMC.begin()) {
    Serial.println("MicroSD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No MicroSD Card found");
    return;
  }
}

void takeNewPhoto(String path) {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  fs::FS &fs = SD_MMC;
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in write mode");
  }
  else {
    file.write(fb->buf, fb->len);
    Serial.printf("Saved file to path: %s\n", path.c_str());
  }
  file.close();
  esp_camera_fb_return(fb);
}

// --- CALLBACK GỬI & NHẬN (Giữ nguyên) ---
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataBytes, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
    Serial.println("\n--- TÍN HIỆU LED NHẬN ĐƯỢC ---");
    Serial.print("Led State: ");
    Serial.println(incomingData.state ? "ON" : "OFF");
    
    // Điều khiển LED báo hiệu (GPIO 4)
    if (incomingData.state == true) { 
      digitalWrite(STATUS_LED_PIN, HIGH);
      Serial.println("-> Bật LED trạng thái (GPIO 4)");
    } else {
      digitalWrite(STATUS_LED_PIN, LOW);
      Serial.println("-> Tắt LED trạng thái (GPIO 4)");
    }
    Serial.println("------------------------------\n");
  } else {
    Serial.println("Received data length mismatch (Ignoring non-LED message).");
  }
}

void sendRandomData() {
    static unsigned int readingId = 0;
    // ... (Tạo dữ liệu biển số ngẫu nhiên)
    int province = 43; 
    char series = random('A', 'Z' + 1);
    int num1 = random(100, 1000);
    int num2 = random(0, 100);
    snprintf(myData.CarLicense, 11, "%d%c-%03d.%02d",
             province, series, num1, num2);

    myData.id = 1; 
    myData.status = 1; 
    myData.readingId = readingId++;

    // Gửi dữ liệu
    esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));

    Serial.print("Sending data (License: ");
    Serial.print(myData.CarLicense);
    Serial.println(result == ESP_OK ? " Sent" : " Failed to send");
}

// --- SETUP ---
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(1000); 
  Serial.println("--- ESP32-CAM Deep Sleep Transceiver ---");

  // Cấu hình chân LED báo hiệu và Buzzer
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW); 
  
  // --- 1. THIẾT LẬP KÊNH WI-FI CHO ESP-NOW ---
  WiFi.mode(WIFI_STA);
  int32_t channel = getWiFiChannel(WIFI_SSID);
  
  if (channel != 0) {
    Serial.printf("Tìm thấy kênh WiFi '%s': %d\n", WIFI_SSID, channel);
    // Thiết lập chế độ Promiscuous để cho phép đặt kênh
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  } else {
    Serial.printf("Cảnh báo: Không tìm thấy mạng '%s'. Đang sử dụng kênh mặc định.\n", WIFI_SSID);
    channel = 0; 
  }
  // ---------------------------------------------

  // --- 2. KHỞI TẠO ESP-NOW ---
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv); 
    
    // Đăng ký Peer
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = channel; // Sử dụng kênh đã tìm thấy (hoặc 0)
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  } else {
    Serial.println("Error initializing ESP-NOW");
  }

  // --- 3. LOGIC KHI THỨC DẬY / KHỞI ĐỘNG ---
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("!!! WAKEUP: Kích hoạt từ cảm biến GPIO 13 (HIGH) !!!");
    
    // Thực hiện chuỗi tác vụ
    playAlarm();
    configESPCamera();
    initMicroSDCard();
    EEPROM.begin(EEPROM_SIZE);
    pictureCount = EEPROM.read(0) + 1;
    String path = "/image" + String(pictureCount) + ".jpg";
    takeNewPhoto(path);
    EEPROM.write(0, pictureCount);
    EEPROM.commit();
    
    // Gửi thông báo
    sendRandomData();
    
    // Lắng nghe tín hiệu phản hồi từ Peer trong 3 giây
    Serial.printf("Lắng nghe tín hiệu LED từ Peer trong %ld ms...\n", LISTEN_DURATION_MS);
    unsigned long startTime = millis();
    while (millis() - startTime < LISTEN_DURATION_MS) {
        delay(1); 
    }
    digitalWrite(STATUS_LED_PIN, LOW); 
    Serial.println("Kết thúc lắng nghe.");

  } else {
    Serial.println("System Restart or First Boot. Entering Sleep Mode.");
  }

  // --- 4. CẤU HÌNH NGỦ SÂU ---
  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 1);
  Serial.print("Cấu hình cảm biến tại GPIO ");
  Serial.print(WAKEUP_PIN);
  Serial.println(" để kích hoạt ở mức HIGH.");

  // Dọn dẹp và ngủ
  esp_camera_deinit(); // Tắt camera để tiết kiệm năng lượng
  Serial.println("Entering Deep Sleep mode...");
  delay(1000);
  esp_deep_sleep_start();
}

void loop() {
  // Không có code trong loop
}
