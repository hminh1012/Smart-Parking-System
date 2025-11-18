

// --- Wi-Fi Credentials ---
#define WIFI_SSID "Bubuchacha"
#define WIFI_PASSWORD "umbalaxibua"

// --- Firebase Project Credentials ---
#define WEB_API_KEY "AIzaSyC58kY22AMwBzdzzOfp66BRBzOZG9Kl8xo"
#define DATABASE_URL "https://esp-project-5cd9d-default-rtdb.asia-southeast1.firebasedatabase.app/"

// --- Firebase User Authentication ---
#define USER_EMAIL "starsrising8888@gmail.com"
#define USER_PASS "kuroba12"


// --- Cấu hình hệ thống ---
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#define NUM_BOARDS 2 // --- OPTIMIZED: Quản lý số lượng board
#define HOURLY_RATE 2.5f // --- OPTIMIZED: Đưa giá tiền ra làm hằng số
#define FIREBASE_READ_INTERVAL 10000 // Đọc Firebase mỗi 10 giây

// --- Đường dẫn Firebase ---
// --- OPTIMIZED: Dùng hằng số cho đường dẫn và trạng thái
const char* FB_BASE_PATH = "/parkingLots/mainStreetGarage/spots";
const char* FB_STATUS_OCCUPIED = "occupied";
const char* FB_STATUS_AVAILABLE = "available";
const char* FB_LED_ON = "on";
const char* FB_LED_OFF = "off";

// --- Core Libraries ---
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <esp_now.h>
#include <WiFi.h>
#include <freertos/queue.h>


// --- Firebase Libraries ---
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// --- MAC Addresses ---
// --- OPTIMIZED: Dùng mảng 2D để lưu MAC address
uint8_t board_macs[NUM_BOARDS][6] = {
  {0x08, 0xF9, 0xE0, 0xEC, 0xF5, 0xA4}, // Board 1
  {0x24, 0x6F, 0x28, 0x45, 0x53, 0xDC}  // Board 2
};

// --- Display & Touchscreen Configuration ---
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320
#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);
int x, y, z; // Tọa độ cảm ứng

// --- ESP-NOW Configuration ---
QueueHandle_t esp_now_queue;

typedef struct struct_message {
  int id; // 1 hoặc 2
  int status; // 1 (Occupied) hoặc 0 (Available)
  char CarLicense[11];
  int readingId;
} struct_message;

typedef struct led_message {
  int id;
  bool state;
} led_message;

// --- Firebase Components ---
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// --- LVGL GUI Objects ---
// --- OPTIMIZED: Dùng mảng để lưu các đối tượng GUI
static lv_obj_t * info_tables[NUM_BOARDS];
static lv_obj_t * led_buttons[NUM_BOARDS];

// --- Quản lý trạng thái bãi đỗ ---
// --- OPTIMIZED: Dùng struct và mảng để quản lý trạng thái
struct ParkingSpot {
  int last_status = 0; // 0 = available, 1 = occupied
  unsigned long start_time = 0;
};
ParkingSpot spots[NUM_BOARDS];

// --- Variables for Periodic Firebase Read ---
unsigned long lastReadMillis = 0;

// --- Helper Function ---

/**
 * @brief Lấy đường dẫn Firebase cho một vị trí đỗ xe cụ thể.
 * @param board_id ID của board (1 hoặc 2).
 * @return String đường dẫn đầy đủ.
 */
String getSpotPath(int board_id) {
  return String(FB_BASE_PATH) + "/A0" + String(board_id);
}

// --- Callback Functions ---

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  struct_message myData;
  memcpy(&myData, incomingData, sizeof(myData));
  xQueueSendFromISR(esp_now_queue, &myData, NULL);
}

/**
 * @brief Cập nhật giao diện nút nhấn LED từ trạng thái Firebase.
 * @param board_id ID của board (1 hoặc 2).
 * @param state Trạng thái "on" hoặc "off".
 */
void update_led_button_ui(int board_id, String state) {
  int board_index = board_id - 1;
  if (board_index < 0 || board_index >= NUM_BOARDS) return; // Kiểm tra an toàn

  lv_obj_t* btn = led_buttons[board_index];
  if (btn == NULL) return;

  if (state == FB_LED_ON) {
    lv_obj_add_state(btn, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(btn, LV_STATE_CHECKED);
  }
}

/**
 * @brief Đọc dữ liệu (trạng thái LED) từ Firebase cho một board.
 * @param board_id ID của board (1 hoặc 2).
 */
/**
 * @brief Reads LED status from Firebase, updates GUI, AND syncs via ESP-NOW.
 * @param board_id ID of the board (1 or 2).
 */
void readDataFromFirebase(int board_id) {
  if (!Firebase.ready()) {
    Serial.println("Firebase is not ready to read data.");
    return;
  }

  String ledPath = getSpotPath(board_id) + "/led_status";

  // Read string data ("on" or "off")
  if (Firebase.RTDB.getString(&fbdo, ledPath)) {
    String led_state_str = fbdo.stringData();
    bool isLedOn = (led_state_str == FB_LED_ON);

    // --- 1. Update Local GUI ---
    update_led_button_ui(board_id, led_state_str);

    // --- 2. Send Command via ESP-NOW ---
    // Calculate array index
    int board_index = board_id - 1; 

    if (board_index >= 0 && board_index < NUM_BOARDS) {
      // Prepare the message
      led_message msg;
      msg.id = board_id;
      msg.state = isLedOn; // Convert to boolean for the struct

      // Send to the specific MAC address of this board
      esp_err_t result = esp_now_send(board_macs[board_index], (uint8_t *) &msg, sizeof(msg));

      if (result == ESP_OK) {
        Serial.printf(">> Synced Board %d LED: %s (Firebase -> GUI -> ESP-NOW)\n", board_id, led_state_str.c_str());
      } else {
        Serial.printf(">> Error syncing Board %d via ESP-NOW\n", board_id);
      }
    }

  } else {
    Serial.printf("Failed to read LED status for Board %d: %s\n", board_id, fbdo.errorReason().c_str());
  }
}

// --- GUI Functions ---

static void led_button_event_handler(lv_event_t * e) {
  lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
  // --- OPTIMIZED: Lấy board_id (1 hoặc 2) từ user_data
  int board_id = (int)lv_event_get_user_data(e);
  int board_index = board_id - 1;
  bool led_state = lv_obj_has_state(btn, LV_STATE_CHECKED);

  Serial.printf("Board %d LED button toggled. New state: %s\n", board_id, led_state ? "ON" : "OFF");

  // 1. Gửi lệnh ESP-NOW
  led_message msg;
  msg.id = board_id;
  msg.state = led_state;

  if (board_index < 0 || board_index >= NUM_BOARDS) return; // Kiểm tra an toàn
  
  uint8_t *target_mac = board_macs[board_index];
  esp_err_t result = esp_now_send(target_mac, (uint8_t *) &msg, sizeof(msg));

  if (result != ESP_OK) {
    Serial.println("Error sending ESP-NOW command.");
  }

  // 2. Cập nhật Firebase
  if (Firebase.ready()) {
    String path = getSpotPath(board_id) + "/led_status";
    String state_str = led_state ? FB_LED_ON : FB_LED_OFF;
    
    if (!Firebase.RTDB.setString(&fbdo, path, state_str)) {
      Serial.printf("Firebase setString error: %s\n", fbdo.errorReason().c_str());
    }
  }
}

void update_table_values(struct_message *myData) {
  int board_index = myData->id - 1;
  // --- OPTIMIZED: Kiểm tra index an toàn
  if (board_index < 0 || board_index >= NUM_BOARDS) return;

  const char* status_str = (myData->status == 1) ? FB_STATUS_OCCUPIED : FB_STATUS_AVAILABLE;
  lv_obj_t* target_table = info_tables[board_index];
  
  lv_table_set_cell_value(target_table, 0, 1, status_str);
  lv_table_set_cell_value(target_table, 1, 1, (myData->status == 1) ? myData->CarLicense : "--");
  lv_table_set_cell_value(target_table, 2, 1, String(myData->readingId).c_str());
}

// --- Touchscreen and Display Driver Functions ---
// (Không thay đổi - Giữ nguyên các hàm log_print, touchscreen_read)

void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();
    float alpha_x, beta_x, alpha_y, beta_y, delta_x, delta_y;
    alpha_x = -0.000; beta_x = 0.090; delta_x = -33.771;
    alpha_y = 0.066; beta_y = 0.000; delta_y = -14.632;
    x = alpha_y * p.x + beta_y * p.y + delta_y;
    x = max(0, x); x = min(SCREEN_WIDTH - 1, x);
    y = alpha_x * p.x + beta_x * p.y + delta_x;
    y = max(0, y); y = min(SCREEN_HEIGHT - 1, y);
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  }
  else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// --- OPTIMIZED: Hàm trợ giúp tạo tab thông tin
static void create_info_tab(lv_obj_t * parent_tab, int board_index) {
  lv_obj_t * table = lv_table_create(parent_tab);
  lv_table_set_cell_value(table, 0, 0, "Status");
  lv_table_set_cell_value(table, 1, 0, "Car License");
  lv_table_set_cell_value(table, 2, 0, "Reading ID");
  lv_table_set_cell_value(table, 0, 1, "--");
  lv_table_set_cell_value(table, 1, 1, "--");
  lv_table_set_cell_value(table, 2, 1, "--");
  lv_obj_center(table);
  info_tables[board_index] = table; // Lưu tham chiếu vào mảng
}

// --- OPTIMIZED: Hàm trợ giúp tạo nút LED
static void create_led_button(lv_obj_t * parent_cont, int board_id) {
  int board_index = board_id - 1;
  
  lv_obj_t* btn = lv_button_create(parent_cont);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn, 200, 50);
  // --- OPTIMIZED: Truyền board_id (1 hoặc 2) làm user_data
  lv_obj_add_event_cb(btn, led_button_event_handler, LV_EVENT_VALUE_CHANGED, (void*)board_id);
  
  lv_obj_t * label = lv_label_create(btn);
  lv_label_set_text(label, ("Board " + String(board_id) + " LED").c_str());
  lv_obj_center(label);
  
  led_buttons[board_index] = btn; // Lưu tham chiếu vào mảng
}

void lv_create_main_gui(void) {
  lv_obj_t * tabview = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_size(tabview, 40);

  // --- OPTIMIZED: Dùng vòng lặp tạo tab thông tin
  for (int i = 0; i < NUM_BOARDS; i++) {
    String tab_name = "BOARD #" + String(i + 1);
    lv_obj_t * tab = lv_tabview_add_tab(tabview, tab_name.c_str());
    create_info_tab(tab, i);
  }

  // Tab điều khiển LED
  lv_obj_t * tab_control = lv_tabview_add_tab(tabview, "LED Control");
  lv_obj_set_flex_flow(tab_control, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(tab_control, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  // --- OPTIMIZED: Dùng vòng lặp tạo nút LED
  for (int i = 0; i < NUM_BOARDS; i++) {
    create_led_button(tab_control, i + 1); // board_id là 1-based (1, 2)
  }
}

// --- Main Setup ---
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Gateway Initializing...");

  // --- Wi-Fi Connection ---
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // --- ESP-NOW Initialization ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // --- OPTIMIZED: Dùng vòng lặp để thêm peer
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;
  
  for (int i = 0; i < NUM_BOARDS; i++) {
    memcpy(peerInfo.peer_addr, board_macs[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.printf("Failed to add peer %d\n", i + 1);
      return;
    }
  }

  esp_now_queue = xQueueCreate(10, sizeof(struct_message));
  if (esp_now_queue == NULL) {
    Serial.println("Error creating queue");
    return;
  }

  // --- LVGL and Display Initialization ---
  lv_init();
  lv_log_register_print_cb(log_print);
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(2);
  lv_display_t * disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);
  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchscreen_read);
  lv_create_main_gui();

  // --- Firebase Initialization ---
  Serial.println("Initializing Firebase...");
  config.api_key = WEB_API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASS;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  // --- OPTIMIZED: Đồng bộ trạng thái LED khi khởi động
  // Đợi Firebase sẵn sàng
  while (!Firebase.ready()) {
    Serial.println("Waiting for Firebase connection...");
    delay(1000);
  }
  Serial.println("Firebase connected.");
  // Đọc trạng thái ban đầu
  for (int i = 0; i < NUM_BOARDS; i++) {
    readDataFromFirebase(i + 1);
  }

  Serial.println("Initialization complete. Waiting for data...");
}

// --- Main Loop ---
void loop() {
  // Handle LVGL tasks
  lv_task_handler();
  lv_tick_inc(5);
  delay(5);

  // Check queue for incoming ESP-NOW messages
  struct_message receivedData;
  if (xQueueReceive(esp_now_queue, &receivedData, 0) == pdTRUE) {
    Serial.printf("Processing data from Board ID: %d, Status: %d\n", receivedData.id, receivedData.status);

    // --- OPTIMIZED: Sử dụng mảng struct để quản lý
    int board_index = receivedData.id - 1;
    if (board_index < 0 || board_index >= NUM_BOARDS) {
      Serial.println("Received data from unknown board ID.");
      return; // Thoát nếu ID không hợp lệ
    }
    
    ParkingSpot* spot = &spots[board_index];
    int current_status_int = receivedData.status;

    // --- OPTIMIZED: Tạo một đối tượng JSON để gộp các lệnh ghi
    FirebaseJson json;
    String spotPath = getSpotPath(receivedData.id);

    // Event starts: Car arrives (status changes to 1)
    if (current_status_int == 1 && spot->last_status != 1) {
      spot->start_time = millis();
      Serial.printf("Board %d: Car arrived. Timer started.\n", receivedData.id);

      if (Firebase.ready()) {
        // --- OPTIMIZATION: Gộp 4 lệnh thành 1 ---
        json.set("status", FB_STATUS_OCCUPIED);
        json.set("currentVehicle/licensePlate", String(receivedData.CarLicense));
        json.set("duration", "null"); // Dùng "null" để xóa node
        json.set("revenue", "null");  // Dùng "null" để xóa node

        Serial.print("Sending 'Car Arrived' data to Firebase... ");
        // Dùng updateNode (hoặc setNode) để ghi toàn bộ JSON
        if (!Firebase.RTDB.updateNode(&fbdo, spotPath, &json)) {
          Serial.printf("Firebase update error: %s\n", fbdo.errorReason().c_str());
        } else {
          Serial.println("OK.");
        }
      }
    }
    // Event ends: Car leaves (status changes from 1 to 0)
    else if (current_status_int != 1 && spot->last_status == 1) {
      unsigned long duration_ms = millis() - spot->start_time;
      float duration_h = (float)duration_ms / (1000.0f * 60.0f * 60.0f);
      float revenue = duration_h * HOURLY_RATE;

      Serial.printf("Board %d: Car left. Duration: %.4f hours. Revenue: $%.2f\n", receivedData.id, duration_h, revenue);

      if (Firebase.ready()) {
        // --- OPTIMIZATION: Gộp 4 lệnh thành 1 ---
        json.set("status", FB_STATUS_AVAILABLE);
        json.set("currentVehicle", "null"); // Xóa toàn bộ node currentVehicle
        json.set("duration", duration_h);
        json.set("revenue", revenue);
        
        Serial.print("Sending 'Car Left' data to Firebase... ");
        if (!Firebase.RTDB.updateNode(&fbdo, spotPath, &json)) {
          Serial.printf("Firebase update error: %s\n", fbdo.errorReason().c_str());
        } else {
          Serial.println("OK.");
        }
      }
    }

    // Update the last known status
    spot->last_status = current_status_int;

    // Cập nhật màn hình (luôn luôn)
    update_table_values(&receivedData);
  }

  // --- Periodically read data from Firebase ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastReadMillis >= FIREBASE_READ_INTERVAL) {
    lastReadMillis = currentMillis;

    if (Firebase.ready()) {
      Serial.println();
      // --- OPTIMIZED: Dùng vòng lặp để đọc dữ liệu
      for (int i = 0; i < NUM_BOARDS; i++) {
        readDataFromFirebase(i + 1);
      }
      Serial.println();
    } else {
      Serial.println("Firebase not ready, skipping periodic read.");
    }
  }
}
