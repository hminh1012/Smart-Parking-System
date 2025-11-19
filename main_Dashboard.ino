/*
 * PROJECT: SMART PARKING GATEWAY (Dynamic Config)
 * FEATURES: ESP-NOW, Firebase, LVGL GUI, Connectivity Monitor, Dynamic Device Count
 */

// --- Wi-Fi Credentials ---
#define WIFI_SSID "Bubuchacha"
#define WIFI_PASSWORD "umbalaxibua"

// --- Firebase Project Credentials ---
#define WEB_API_KEY "AIzaSyC58kY22AMwBzdzzOfp66BRBzOZG9Kl8xo"
#define DATABASE_URL "https://esp-project-5cd9d-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL "starsrising8888@gmail.com"
#define USER_PASS "kuroba12"

// --- System Config ---
#define MAX_BOARDS 10        // Giới hạn bộ nhớ tối đa (Cấp phát sẵn)
#define HOURLY_RATE 2.5f     // Giá tiền mỗi giờ ($)
#define FIREBASE_READ_INTERVAL 10000 // Đọc Firebase mỗi 10 giây

int current_boards = 2;      // Mặc định là 2, sẽ được cập nhật từ Firebase khi khởi động

// --- Firebase Paths ---
const char* FB_BASE_PATH = "/parkingLots/mainStreetGarage/spots";
const char* FB_CONFIG_PATH = "/parkingLots/mainStreetGarage/lotInfo/MaxDevice"; // Đường dẫn cấu hình
const char* FB_STATUS_OCCUPIED = "occupied";
const char* FB_STATUS_AVAILABLE = "available";
const char* FB_LED_ON = "on";
const char* FB_LED_OFF = "off";

// --- Libraries ---
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <esp_now.h>
#include <WiFi.h>
#include <freertos/queue.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// --- MAC Addresses (Pre-filled for MAX_BOARDS) ---
// Bạn cần điền đúng MAC của các board thực tế vào các dòng đầu
uint8_t board_macs[MAX_BOARDS][6] = {
  {0x08, 0xF9, 0xE0, 0xEC, 0xF5, 0xA4}, // Board ID 1
  {0x24, 0x6F, 0x28, 0x45, 0x53, 0xDC}, // Board ID 2
  {0xD8, 0x13, 0x2A, 0x73, 0x2D, 0x24}, // Board ID 3
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // Board ID 4
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, // ...
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // Board ID 10
};

// --- Display & Touchscreen Config ---
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
int x, y, z;

// --- Data Structures ---
QueueHandle_t esp_now_queue;

typedef struct struct_message {
  int id;
  int status; // 1 = Occupied, 0 = Available
  char CarLicense[11];
  int readingId;
} struct_message;

typedef struct led_message {
  int id;
  bool state;
} led_message;

struct ParkingSpot {
  int last_status = 0;
  unsigned long start_time = 0;
};
// Sử dụng mảng tĩnh với kích thước MAX
ParkingSpot spots[MAX_BOARDS];

// --- Firebase Objects ---
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long lastReadMillis = 0;

// --- GUI Objects ---
// Sử dụng mảng tĩnh với kích thước MAX
static lv_obj_t * info_tables[MAX_BOARDS];
static lv_obj_t * led_buttons[MAX_BOARDS];
static lv_obj_t * conn_table; 

// --- Helper Functions ---

String getSpotPath(int board_id) {
  return String(FB_BASE_PATH) + "/A0" + String(board_id);
}

String formatMacAddress(const uint8_t *macAddr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  return String(macStr);
}

// --- ESP-NOW Callback ---
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  struct_message myData;
  memcpy(&myData, incomingData, sizeof(myData));
  xQueueSendFromISR(esp_now_queue, &myData, NULL);
}

// --- GUI Helpers ---
void update_led_button_ui(int board_id, String state) {
  int board_index = board_id - 1;
  if (board_index < 0 || board_index >= current_boards) return; // Check bound theo current_boards
  
  lv_obj_t* btn = led_buttons[board_index];
  if (btn == NULL) return;

  if (state == FB_LED_ON) lv_obj_add_state(btn, LV_STATE_CHECKED);
  else lv_obj_clear_state(btn, LV_STATE_CHECKED);
}

void readDataFromFirebase(int board_id) {
  if (!Firebase.ready()) return;

  String ledPath = getSpotPath(board_id) + "/led_status";
  if (Firebase.RTDB.getString(&fbdo, ledPath)) {
    String led_state_str = fbdo.stringData();
    bool isLedOn = (led_state_str == FB_LED_ON);

    // Update UI & Sync ESP-NOW
    update_led_button_ui(board_id, led_state_str);

    int board_index = board_id - 1; 
    if (board_index >= 0 && board_index < current_boards) {
      led_message msg;
      msg.id = board_id;
      msg.state = isLedOn;
      esp_now_send(board_macs[board_index], (uint8_t *) &msg, sizeof(msg));
    }
  }
}

static void led_button_event_handler(lv_event_t * e) {
  lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
  int board_id = (int)lv_event_get_user_data(e);
  int board_index = board_id - 1;
  bool led_state = lv_obj_has_state(btn, LV_STATE_CHECKED);

  Serial.printf("Toggle LED Board %d -> %s\n", board_id, led_state ? "ON" : "OFF");

  // 1. ESP-NOW
  if (board_index >= 0 && board_index < current_boards) {
    led_message msg;
    msg.id = board_id;
    msg.state = led_state;
    esp_now_send(board_macs[board_index], (uint8_t *) &msg, sizeof(msg));
  }

  // 2. Firebase
  if (Firebase.ready()) {
    String path = getSpotPath(board_id) + "/led_status";
    Firebase.RTDB.setString(&fbdo, path, led_state ? FB_LED_ON : FB_LED_OFF);
  }
}

void update_table_values(struct_message *myData) {
  int board_index = myData->id - 1;
  if (board_index < 0 || board_index >= current_boards) return;

  const char* status_str = (myData->status == 1) ? FB_STATUS_OCCUPIED : FB_STATUS_AVAILABLE;
  lv_obj_t* target_table = info_tables[board_index];
  
  if (target_table) {
    lv_table_set_cell_value(target_table, 0, 1, status_str);
    lv_table_set_cell_value(target_table, 1, 1, (myData->status == 1) ? myData->CarLicense : "--");
    lv_table_set_cell_value(target_table, 2, 1, String(myData->readingId).c_str());
  }
}

// --- Display Driver ---
void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();
    // CALIBRATION (Tùy chỉnh theo màn hình của bạn)
    float alpha_x = -0.000, beta_x = 0.090, delta_x = -33.771;
    float alpha_y = 0.066, beta_y = 0.000, delta_y = -14.632;
    
    x = alpha_y * p.x + beta_y * p.y + delta_y;
    x = max(0, min(SCREEN_WIDTH - 1, x));
    y = alpha_x * p.x + beta_x * p.y + delta_x;
    y = max(0, min(SCREEN_HEIGHT - 1, y));
    
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// --- GUI Creation Functions ---
static void create_info_tab(lv_obj_t * parent_tab, int board_index) {
  lv_obj_t * table = lv_table_create(parent_tab);
  lv_table_set_cell_value(table, 0, 0, "Status");
  lv_table_set_cell_value(table, 1, 0, "License");
  lv_table_set_cell_value(table, 2, 0, "Read ID");
  lv_table_set_cell_value(table, 0, 1, "--");
  lv_table_set_cell_value(table, 1, 1, "--");
  lv_table_set_cell_value(table, 2, 1, "--");
  lv_obj_center(table);
  info_tables[board_index] = table;
}

static void create_led_button(lv_obj_t * parent_cont, int board_id) {
  int board_index = board_id - 1;
  lv_obj_t* btn = lv_button_create(parent_cont);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn, 200, 40); // Button nhỏ lại chút
  lv_obj_add_event_cb(btn, led_button_event_handler, LV_EVENT_VALUE_CHANGED, (void*)board_id);
  
  lv_obj_t * label = lv_label_create(btn);
  lv_label_set_text(label, ("Board " + String(board_id) + " LED").c_str());
  lv_obj_center(label);
  led_buttons[board_index] = btn;
}

void lv_create_main_gui(void) {
  lv_obj_t * tabview = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_size(tabview, 30);

  // 1. Dynamic Info Tabs
  for (int i = 0; i < current_boards; i++) {
    lv_obj_t * tab = lv_tabview_add_tab(tabview, ("B" + String(i + 1)).c_str());
    create_info_tab(tab, i);
  }

  // 2. LED Control Tab
  lv_obj_t * tab_control = lv_tabview_add_tab(tabview, "LED");
  lv_obj_set_flex_flow(tab_control, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(tab_control, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  for (int i = 0; i < current_boards; i++) {
    create_led_button(tab_control, i + 1);
  }

  // 3. Connectivity Tab
  lv_obj_t * tab_conn = lv_tabview_add_tab(tabview, "Conn");
  conn_table = lv_table_create(tab_conn);
  lv_obj_set_size(conn_table, 230, 200);
  lv_obj_center(conn_table);
  
  // Header
  lv_table_set_cell_value(conn_table, 0, 0, "ID");
  lv_table_set_cell_value(conn_table, 0, 1, "MAC");
  lv_table_set_cell_value(conn_table, 0, 2, "Sts");
  
  // Rows (Dynamic)
  for (int i = 0; i < current_boards; i++) {
    int row = i + 1;
    lv_table_set_cell_value(conn_table, row, 0, String(i + 1).c_str());
    // Hiển thị 3 byte cuối của MAC cho gọn
    String shortMac = formatMacAddress(board_macs[i]);
    lv_table_set_cell_value(conn_table, row, 1, shortMac.c_str());
    lv_table_set_cell_value(conn_table, row, 2, "Off");
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  Serial.println("System Initializing...");

  // 1. WiFi Connection
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifi < 10000) {
    Serial.print("."); delay(200);
  }
  Serial.println();

  // 2. Init Firebase & Fetch Config (CRITICAL STEP)
  config.api_key = WEB_API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASS;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("Fetching Lot Configuration...");
  unsigned long startWait = millis();
  while (!Firebase.ready() && millis() - startWait < 10000) {
    delay(100);
  }

  if (Firebase.ready()) {
    // Lấy số lượng board từ Firebase
    if (Firebase.RTDB.getInt(&fbdo, FB_CONFIG_PATH)) {
      int fetched_val = fbdo.intData();
      Serial.printf(">> Firebase Config MaxDevice: %d\n", fetched_val);
      
      // Validate
      if (fetched_val > 0 && fetched_val <= MAX_BOARDS) {
        current_boards = fetched_val;
      } else {
        Serial.println(">> Invalid MaxDevice value! Using default.");
      }
    } else {
      Serial.println(">> Failed to read MaxDevice. Using default.");
    }
  } else {
    Serial.println(">> Firebase not ready. Using default config.");
  }
  Serial.printf(">> System Configured for %d Boards.\n", current_boards);

  // 3. Init ESP-NOW (Dựa trên current_boards)
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = WiFi.channel();
  peerInfo.encrypt = false;
  
  for (int i = 0; i < current_boards; i++) {
    memcpy(peerInfo.peer_addr, board_macs[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.printf("Failed to add peer %d\n", i+1);
    }
  }
  esp_now_queue = xQueueCreate(10, sizeof(struct_message));

  // 4. Init Display & GUI (Dựa trên current_boards)
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
  
  lv_create_main_gui(); // Vẽ giao diện

  // 5. Initial Sync
  if (Firebase.ready()) {
    for (int i = 0; i < current_boards; i++) readDataFromFirebase(i + 1);
  }
}

// --- LOOP ---
void loop() {
  lv_task_handler();
  lv_tick_inc(5);
  delay(5);

  // Handle ESP-NOW Data
  struct_message receivedData;
  if (xQueueReceive(esp_now_queue, &receivedData, 0) == pdTRUE) {
    int board_index = receivedData.id - 1;
    
    // Chỉ xử lý nếu ID nằm trong giới hạn cấu hình hiện tại
    if (board_index >= 0 && board_index < current_boards) {
      Serial.printf("Data from Board %d (Status: %d)\n", receivedData.id, receivedData.status);

      // Connectivity Update
      if (conn_table != NULL) {
         lv_table_set_cell_value(conn_table, board_index + 1, 2, "Online");
      }
      
      // Parking Logic & Firebase
      ParkingSpot* spot = &spots[board_index];
      FirebaseJson json;
      String spotPath = getSpotPath(receivedData.id);

      if (receivedData.status == 1 && spot->last_status != 1) {
        spot->start_time = millis();
        if (Firebase.ready()) {
          json.set("status", FB_STATUS_OCCUPIED);
          json.set("currentVehicle/licensePlate", String(receivedData.CarLicense));
          json.set("duration", "null");
          json.set("revenue", "null");
          Firebase.RTDB.updateNode(&fbdo, spotPath, &json);
        }
      }
      else if (receivedData.status == 0 && spot->last_status == 1) {
        float duration_h = (millis() - spot->start_time) / 3600000.0f;
        float revenue = duration_h * HOURLY_RATE;
        if (Firebase.ready()) {
          json.set("status", FB_STATUS_AVAILABLE);
          json.set("currentVehicle", "null");
          json.set("duration", duration_h);
          json.set("revenue", revenue);
          Firebase.RTDB.updateNode(&fbdo, spotPath, &json);
        }
      }
      spot->last_status = receivedData.status;
      update_table_values(&receivedData);
    } else {
      Serial.printf("Ignored data from Board ID %d (Outside Config limit %d)\n", receivedData.id, current_boards);
    }
  }

  // Periodic Firebase Read
  if (millis() - lastReadMillis >= FIREBASE_READ_INTERVAL) {
    lastReadMillis = millis();
    if (Firebase.ready()) {
      // Chỉ đọc số lượng board đang active
      for (int i = 0; i < current_boards; i++) {
        readDataFromFirebase(i + 1);
      }
    }
  }
}
