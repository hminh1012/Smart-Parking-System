/*
 * PROJECT: SMART PARKING GATEWAY (MAC-Based ID Security)
 * STATUS: Final Production Version
 * UPDATES: 
 * - ID is determined by MAC Address lookup (Not incoming packet ID)
 * - "Unknown MAC" packets are ignored
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
#define MAX_BOARDS 10        
#define HOURLY_RATE 2.5f     
#define FIREBASE_READ_INTERVAL 10000 
#define CONNECTION_TIMEOUT_MS 60000  
#define CHECK_TIMEOUT_INTERVAL 5000  

int current_boards = 2; 

// --- Firebase Paths ---
const char* FB_BASE_PATH = "/parkingLots/mainStreetGarage/spots";
const char* FB_CONFIG_MAX_DEVICE = "/parkingLots/mainStreetGarage/lotInfo/MaxDevice";
const char* FB_CONFIG_MAC_ROOT = "/parkingLots/mainStreetGarage/lotInfo/BoardMac"; 

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

// --- MAC Addresses ---
uint8_t board_macs[MAX_BOARDS][6] = {
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 
  // ... Init others to FF
};

// --- Connection Tracking ---
unsigned long boards_last_seen[MAX_BOARDS] = {0}; 
bool boards_online_state[MAX_BOARDS] = {false};   

// --- Display ---
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
  int id; // We will ignore this ID now
  int status; 
  char CarLicense[11];
  int readingId;
} struct_message;

// --- NEW: Wrapper Struct for Queue ---
typedef struct GatewayMessage {
  struct_message payload;
  uint8_t senderMac[6];
} GatewayMessage;

typedef struct led_message {
  int id;
  bool state;
} led_message;

struct ParkingSpot {
  int last_status = 0;
  unsigned long start_time = 0;
};
ParkingSpot spots[MAX_BOARDS];

// --- Firebase Objects ---
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long lastReadMillis = 0;
unsigned long lastTimeoutCheck = 0; 

// --- GUI Objects ---
static lv_obj_t * info_tables[MAX_BOARDS];
static lv_obj_t * led_buttons[MAX_BOARDS];
static lv_obj_t * conn_table; 

// ================= HELPERS =================

String getSpotPath(int board_id) {
  return String(FB_BASE_PATH) + "/A0" + String(board_id);
}

String getShortMac(const uint8_t *macAddr) {
  char macStr[10];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X",
           macAddr[3], macAddr[4], macAddr[5]);
  return String(macStr);
}

String formatMacAddress(const uint8_t *macAddr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  return String(macStr);
}

void stringToMac(String macStr, uint8_t* targetMac) {
  int values[6];
  if (6 == sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", 
                  &values[0], &values[1], &values[2], &values[3], &values[4], &values[5])) {
    for (int i = 0; i < 6; i++) targetMac[i] = (uint8_t)values[i];
  }
}

// ================= UPDATED CALLBACK =================

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  GatewayMessage msg;
  // 1. Copy Data
  memcpy(&msg.payload, incomingData, sizeof(struct_message));
  // 2. Copy Sender MAC
  memcpy(msg.senderMac, recv_info->src_addr, 6);
  // 3. Send Wrapper to Queue
  xQueueSendFromISR(esp_now_queue, &msg, NULL);
}

// ================= GUI LOGIC =================

void update_led_button_ui(int board_id, String state) {
  int board_index = board_id - 1;
  if (board_index < 0 || board_index >= current_boards) return;
  if (led_buttons[board_index]) {
    if (state == FB_LED_ON) lv_obj_add_state(led_buttons[board_index], LV_STATE_CHECKED);
    else lv_obj_clear_state(led_buttons[board_index], LV_STATE_CHECKED);
  }
}

void readDataFromFirebase(int board_id) {
  if (!Firebase.ready()) return;
  String ledPath = getSpotPath(board_id) + "/led_status";
  if (Firebase.RTDB.getString(&fbdo, ledPath)) {
    String led_state_str = fbdo.stringData();
    update_led_button_ui(board_id, led_state_str);
    
    int board_index = board_id - 1; 
    if (board_index >= 0 && board_index < current_boards) {
      led_message msg; msg.id = board_id; msg.state = (led_state_str == FB_LED_ON);
      esp_now_send(board_macs[board_index], (uint8_t *) &msg, sizeof(msg));
    }
  }
}

static void led_button_event_handler(lv_event_t * e) {
  lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
  int board_id = (int)lv_event_get_user_data(e);
  int board_index = board_id - 1;
  bool led_state = lv_obj_has_state(btn, LV_STATE_CHECKED);

  if (board_index >= 0 && board_index < current_boards) {
    led_message msg; msg.id = board_id; msg.state = led_state;
    esp_now_send(board_macs[board_index], (uint8_t *) &msg, sizeof(msg));
  }
  if (Firebase.ready()) {
    String path = getSpotPath(board_id) + "/led_status";
    Firebase.RTDB.setString(&fbdo, path, led_state ? FB_LED_ON : FB_LED_OFF);
  }
}

void update_table_values(int board_id, struct_message *myData) {
  int board_index = board_id - 1;
  if (board_index < 0 || board_index >= current_boards) return;
  lv_obj_t* target_table = info_tables[board_index];
  if (target_table) {
    lv_table_set_cell_value(target_table, 0, 1, (myData->status == 1) ? FB_STATUS_OCCUPIED : FB_STATUS_AVAILABLE);
    lv_table_set_cell_value(target_table, 1, 1, (myData->status == 1) ? myData->CarLicense : "--");
    lv_table_set_cell_value(target_table, 2, 1, String(myData->readingId).c_str());
  }
}

// --- Display Driver ---
void log_print(lv_log_level_t level, const char * buf) { LV_UNUSED(level); Serial.println(buf); Serial.flush(); }
void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();
    float alpha_x = -0.000, beta_x = 0.090, delta_x = -33.771;
    float alpha_y = 0.066, beta_y = 0.000, delta_y = -14.632;
    x = alpha_y * p.x + beta_y * p.y + delta_y;
    x = max(0, min(SCREEN_WIDTH - 1, x));
    y = alpha_x * p.x + beta_x * p.y + delta_x;
    y = max(0, min(SCREEN_HEIGHT - 1, y));
    data->state = LV_INDEV_STATE_PRESSED; data->point.x = x; data->point.y = y;
  } else { data->state = LV_INDEV_STATE_RELEASED; }
}

// --- GUI Builders ---
static void create_info_tab(lv_obj_t * parent_tab, int board_index) {
  lv_obj_t * table = lv_table_create(parent_tab);
  lv_table_set_cell_value(table, 0, 0, "Status"); lv_table_set_cell_value(table, 0, 1, "--");
  lv_table_set_cell_value(table, 1, 0, "License"); lv_table_set_cell_value(table, 1, 1, "--");
  lv_table_set_cell_value(table, 2, 0, "Read ID"); lv_table_set_cell_value(table, 2, 1, "--");
  lv_obj_center(table);
  info_tables[board_index] = table;
}

static void create_led_button(lv_obj_t * parent_cont, int board_id) {
  int board_index = board_id - 1;
  lv_obj_t* btn = lv_button_create(parent_cont);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn, 200, 40);
  lv_obj_add_event_cb(btn, led_button_event_handler, LV_EVENT_VALUE_CHANGED, (void*)board_id);
  lv_obj_t * label = lv_label_create(btn);
  lv_label_set_text(label, ("Board " + String(board_id) + " LED").c_str());
  lv_obj_center(label);
  led_buttons[board_index] = btn;
}

void lv_create_main_gui(void) {
  lv_obj_t * tabview = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_size(tabview, 30);

  for (int i = 0; i < current_boards; i++) {
    lv_obj_t * tab = lv_tabview_add_tab(tabview, ("B" + String(i + 1)).c_str());
    create_info_tab(tab, i);
  }

  lv_obj_t * tab_control = lv_tabview_add_tab(tabview, "LED");
  lv_obj_set_flex_flow(tab_control, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(tab_control, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  for (int i = 0; i < current_boards; i++) create_led_button(tab_control, i + 1);

  lv_obj_t * tab_conn = lv_tabview_add_tab(tabview, "Conn");
  conn_table = lv_table_create(tab_conn);
  lv_table_set_col_width(conn_table, 0, 60);
  lv_table_set_col_width(conn_table, 1, 100);
  lv_table_set_col_width(conn_table, 2, 100);
  lv_obj_set_size(conn_table, 260, 150);
  lv_obj_center(conn_table);
  lv_table_set_cell_value(conn_table, 0, 0, "ID");
  lv_table_set_cell_value(conn_table, 0, 1, "MAC");
  lv_table_set_cell_value(conn_table, 0, 2, "State");
  for (int i = 0; i < current_boards; i++) {
    lv_table_set_cell_value(conn_table, i + 1, 0, String(i + 1).c_str());
    lv_table_set_cell_value(conn_table, i + 1, 1, getShortMac(board_macs[i]).c_str());
    lv_table_set_cell_value(conn_table, i + 1, 2, "Offline");
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial.println("System Initializing...");

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifi < 10000) { delay(100); }

  config.api_key = WEB_API_KEY; auth.user.email = USER_EMAIL; auth.user.password = USER_PASS;
  config.database_url = DATABASE_URL; config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth); Firebase.reconnectWiFi(true);

  Serial.println("Fetching Config from Firebase...");
  unsigned long startWait = millis();
  while (!Firebase.ready() && millis() - startWait < 10000) { delay(100); }

  if (Firebase.ready()) {
    if (Firebase.RTDB.getInt(&fbdo, FB_CONFIG_MAX_DEVICE)) {
      int val = fbdo.intData();
      if (val > 0 && val <= MAX_BOARDS) current_boards = val;
    }
    Serial.printf(">> Configured: %d Boards.\n", current_boards);
    for (int i = 0; i < current_boards; i++) {
      String macPath = String(FB_CONFIG_MAC_ROOT) + "/" + String(i);
      if (Firebase.RTDB.getString(&fbdo, macPath)) {
        String macStr = fbdo.stringData();
        stringToMac(macStr, board_macs[i]);
      }
    }
  }

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = WiFi.channel(); peerInfo.encrypt = false;
  
  for (int i = 0; i < current_boards; i++) {
    memcpy(peerInfo.peer_addr, board_macs[i], 6);
    esp_now_add_peer(&peerInfo);
  }
  
  // --- Update Queue to hold Wrapper Struct ---
  esp_now_queue = xQueueCreate(10, sizeof(GatewayMessage)); 

  lv_init();
  lv_log_register_print_cb(log_print);
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI); touchscreen.setRotation(2);
  lv_display_t * disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);
  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchscreen_read);
  
  lv_create_main_gui(); 

  if (Firebase.ready()) {
    for (int i = 0; i < current_boards; i++) readDataFromFirebase(i + 1);
  }
}

// ================= LOOP =================
void loop() {
  lv_task_handler(); lv_tick_inc(5); delay(5);

  // --- HANDLE ESP-NOW DATA ---
  GatewayMessage msg; // Use wrapper
  if (xQueueReceive(esp_now_queue, &msg, 0) == pdTRUE) {
    
    // 1. Identify Board by MAC
    int calculated_id = -1;
    int board_index = -1;
    for (int i = 0; i < current_boards; i++) {
      if (memcmp(board_macs[i], msg.senderMac, 6) == 0) {
        board_index = i;
        calculated_id = i + 1;
        break;
      }
    }

    // 2. Only proceed if ID matches a Firebase MAC
    if (calculated_id != -1) {
      struct_message receivedData = msg.payload;
      receivedData.id = calculated_id; // Enforce correct ID

      // --- Heartbeat Logic ---
      boards_last_seen[board_index] = millis(); 
      if (!boards_online_state[board_index]) {
        boards_online_state[board_index] = true;
        if (conn_table) lv_table_set_cell_value(conn_table, board_index + 1, 2, "Online");
        if (Firebase.ready()) {
           Firebase.RTDB.setString(&fbdo, getSpotPath(calculated_id) + "/connection_status", "online");
        }
      }

      // --- Parking Logic ---
      ParkingSpot* spot = &spots[board_index];
      FirebaseJson json;
      String spotPath = getSpotPath(calculated_id);

      if (receivedData.status == 1 && spot->last_status != 1) {
        spot->start_time = millis();
        if (Firebase.ready()) {
          json.set("status", FB_STATUS_OCCUPIED);
          json.set("currentVehicle/licensePlate", String(receivedData.CarLicense));
          json.set("duration", "null"); json.set("revenue", "null");
          Firebase.RTDB.updateNode(&fbdo, spotPath, &json);
        }
      } else if (receivedData.status == 0 && spot->last_status == 1) {
        float duration_h = (millis() - spot->start_time) / 3600000.0f;
        if (Firebase.ready()) {
          json.set("status", FB_STATUS_AVAILABLE);
          json.set("currentVehicle", "null");
          json.set("duration", duration_h);
          json.set("revenue", duration_h * HOURLY_RATE);
          Firebase.RTDB.updateNode(&fbdo, spotPath, &json);
        }
      }
      spot->last_status = receivedData.status;
      update_table_values(calculated_id, &receivedData);

    } else {
      // Optional: Print ignored MACs for debugging
      Serial.print("Ignored Unknown MAC: ");
      Serial.println(formatMacAddress(msg.senderMac));
    }
  }

  // --- TIMEOUT CHECK ---
  if (millis() - lastTimeoutCheck > CHECK_TIMEOUT_INTERVAL) {
    lastTimeoutCheck = millis();
    for (int i = 0; i < current_boards; i++) {
      if (boards_online_state[i] && (millis() - boards_last_seen[i] > CONNECTION_TIMEOUT_MS)) {
        boards_online_state[i] = false; 
        if (conn_table) lv_table_set_cell_value(conn_table, i + 1, 2, "Offline");
        if (Firebase.ready()) {
           Firebase.RTDB.setString(&fbdo, getSpotPath(i + 1) + "/connection_status", "offline");
        }
      }
    }
  }

  // --- FIREBASE READ ---
  if (millis() - lastReadMillis >= FIREBASE_READ_INTERVAL) {
    lastReadMillis = millis();
    if (Firebase.ready()) {
      for (int i = 0; i < current_boards; i++) readDataFromFirebase(i + 1);
    }
  }
}
