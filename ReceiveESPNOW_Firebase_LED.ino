/*********
 * Combined code for an ESP32 Gateway.
 * This device:
 * 1. Receives data from other ESP32 boards via ESP-NOW.
 * 2. Displays the received data on a local TFT screen using LVGL.
 * 3. Forwards the data to a Firebase Realtime Database.
 * 4. NEW: Calculates the duration when a board's status is '1' and sends it
 * to Firebase when the status changes back.
 *
 * All necessary configurations and credentials are included in this single file.
*********/
#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
// --- Wi-Fi Credentials ---
// Replace with your network SSID (name) and password
#define WIFI_SSID "Bubuchacha"
#define WIFI_PASSWORD "umbalaxibua"

// --- Firebase Project Credentials ---
// Replace with your Firebase project's credentials.
#define WEB_API_KEY "AIzaSyC58kY22AMwBzdzzOfp66BRBzOZG9Kl8xo"
#define DATABASE_URL "https://esp-project-5cd9d-default-rtdb.asia-southeast1.firebasedatabase.app/" // e.g., "https://your-project-id-default-rtdb.firebaseio.com/"

// --- Firebase User Authentication ---
// Replace with the email and password of a user created in Firebase Authentication.
#define USER_EMAIL "starsrising8888@gmail.com"
#define USER_PASS "kuroba12"


// --- Core Libraries ---
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <image.h>
#include <XPT2046_Touchscreen.h>
#include <esp_now.h>
#include <WiFi.h>
#include <freertos/queue.h>

// --- Firebase Libraries ---
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>

// --- MAC Addresses of Remote Boards ---
// TODO: Replace with the MAC addresses of your remote ESP32 boards
uint8_t board1_mac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01};
uint8_t board2_mac[] = {0x24, 0x6F, 0x28, 0x45, 0x53, 0xDC};

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

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

// --- ESP-NOW Configuration ---
QueueHandle_t esp_now_queue;

// Structure to receive data (must match the sender's structure)
typedef struct struct_message {
  int id;          // Board ID (e.g., 1 or 2)
  int status;      // Application-defined status
  char CarLicense[10]; // Null-terminated string for car license
  int readingId;   // A unique or sequential ID for the reading
} struct_message;

// Structure for sending LED control commands
typedef struct led_message {
  int id;
  bool state; // true for ON, false for OFF
} led_message;

// --- Firebase Components ---
void processData(AsyncResult &aResult); // Forward declaration
UserAuth user_auth(WEB_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
RealtimeDatabase Database;

// --- LVGL GUI Objects ---
static lv_obj_t * table1;
static lv_obj_t * table2;

// --- NEW: Variables for Duration Tracking ---
int last_status_board1 = 0;
int last_status_board2 = 0;
unsigned long start_time_board1 = 0;
unsigned long start_time_board2 = 0;


// --- Callback Functions ---

// Callback function executed when ESP-NOW data is received
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  struct_message myData;
  memcpy(&myData, incomingData, sizeof(myData));
  // Send received data to a queue to be processed in the main loop
  xQueueSendFromISR(esp_now_queue, &myData, NULL);
}

// Firebase async result processing callback
void processData(AsyncResult &aResult) {
  if (aResult.isError()) {
    Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
  }
}

// --- GUI Functions ---
// LVGL: Event handler for the LED control buttons
static void led_button_event_handler(lv_event_t * e) {
    lv_obj_t * btn = (lv_obj_t *)lv_event_get_target(e);
    int board_id = (int)lv_event_get_user_data(e);
    bool led_state = lv_obj_has_state(btn, LV_STATE_CHECKED);

    Serial.printf("Board %d LED button toggled. New state: %s\n", board_id, led_state ? "ON" : "OFF");

    // 1. Send ESP-NOW command to the remote board
    led_message msg;
    msg.id = board_id;
    msg.state = led_state;
    
    uint8_t *target_mac = (board_id == 1) ? board1_mac : board2_mac;
    esp_err_t result = esp_now_send(target_mac, (uint8_t *) &msg, sizeof(msg));

    if (result == ESP_OK) {
        Serial.println("ESP-NOW command sent successfully.");
    } else {
        Serial.println("Error sending ESP-NOW command.");
    }

    // 2. Update Firebase with the new LED state
    if (app.ready()) {
        String path = "/board" + String(board_id) + "/led_status";
        Database.set<bool>(aClient, path, led_state, processData, "UpdateLEDStatus");
    }
}

// Function to update the tables on the LVGL display
void update_table_values(struct_message *myData) {
  if (myData->id == 1) {
    lv_table_set_cell_value(table1, 0, 1, String(myData->status).c_str());
    lv_table_set_cell_value(table1, 1, 1, myData->CarLicense);
    lv_table_set_cell_value(table1, 2, 1, String(myData->readingId).c_str());
  } else if (myData->id == 2) {
    lv_table_set_cell_value(table2, 0, 1, String(myData->status).c_str());
    lv_table_set_cell_value(table2, 1, 1, myData->CarLicense);
    lv_table_set_cell_value(table2, 2, 1, String(myData->readingId).c_str());
  }
}

// --- Firebase Data Sending Function ---
void sendDataToFirebase(struct_message *myData) {
    if (!app.ready()) {
        Serial.println("Firebase not ready, skipping send.");
        return;
    }

    // Create a dynamic path based on the board ID
    String basePath = "/board" + String(myData->id);

    Serial.printf("Sending data for Board %d to Firebase...\n", myData->id);

    // Send status
    Database.set<int>(aClient, basePath + "/status", myData->status, processData, "SendStatus");

    // Send Car License
    Database.set<String>(aClient, basePath + "/CarLicense", String(myData->CarLicense), processData, "SendLicense");

    // Send Reading ID
    Database.set<int>(aClient, basePath + "/readingId", myData->readingId, processData, "SendReadingID");
}


// --- Touchscreen and Display Driver Functions ---
// (These are standard setup functions for the display and LVGL)

void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

void draw_image(void) {
  LV_IMAGE_DECLARE(my_image);
  lv_obj_t * img1 = lv_image_create(lv_screen_active());
  lv_image_set_src(img1, &my_image);
  lv_img_set_zoom(img1, 128);
  lv_obj_align(img1, LV_ALIGN_TOP_LEFT, 0, 0);
}


// Get the Touchscreen data
void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z)
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();

    // Advanced Touchscreen calibration, LEARN MORE » https://RandomNerdTutorials.com/touchscreen-calibration/
    float alpha_x, beta_x, alpha_y, beta_y, delta_x, delta_y;

    // REPLACE WITH YOUR OWN CALIBRATION VALUES » https://RandomNerdTutorials.com/touchscreen-calibration/
    alpha_x = -0.000;
    beta_x = 0.090;
    delta_x = -33.771;
    alpha_y = 0.066;
    beta_y = 0.000;
    delta_y = -14.632;

    x = alpha_y * p.x + beta_y * p.y + delta_y;
    // clamp x between 0 and SCREEN_WIDTH - 1
    x = max(0, x);
    x = min(SCREEN_WIDTH - 1, x);

    y = alpha_x * p.x + beta_x * p.y + delta_x;
    // clamp y between 0 and SCREEN_HEIGHT - 1
    y = max(0, y);
    y = min(SCREEN_HEIGHT - 1, y);

    // Basic Touchscreen calibration points with map function to the correct width and height
    //x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    //y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);

    z = p.z;

    data->state = LV_INDEV_STATE_PRESSED;

    // Set the coordinates
    data->point.x = x;
    data->point.y = y;


  }
  else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void lv_create_main_gui(void) {
  LV_IMAGE_DECLARE(my_image);
  lv_obj_t * tabview = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_size(tabview, 40);
  
  lv_obj_t * tab1 = lv_tabview_add_tab(tabview, "BOARD #1");
  lv_obj_t * tab2 = lv_tabview_add_tab(tabview, "BOARD #2");
  lv_obj_t * tab_control = lv_tabview_add_tab(tabview, "LED Control");

  //config tab1
  lv_obj_t * img1 = lv_image_create(tab1);
  lv_image_set_src(img1, &my_image);
  lv_obj_align(img1, LV_ALIGN_TOP_LEFT, 0, 0);

  table1 = lv_table_create(tab1);
  lv_table_set_cell_value(table1, 0, 0, "Status");
  lv_table_set_cell_value(table1, 1, 0, "Car License");
  lv_table_set_cell_value(table1, 2, 0, "Reading ID");
  lv_table_set_cell_value(table1, 0, 1, "--");
  lv_table_set_cell_value(table1, 1, 1, "--");
  lv_table_set_cell_value(table1, 2, 1, "--");
  lv_obj_center(table1);
  
  //config tab2
  lv_obj_t * img2 = lv_image_create(tab2);
  lv_image_set_src(img2, &my_image);
  lv_obj_align(img2, LV_ALIGN_TOP_LEFT, 0, 0);

  table2 = lv_table_create(tab2);
  lv_table_set_cell_value(table2, 0, 0, "Status");
  lv_table_set_cell_value(table2, 1, 0, "Car License");
  lv_table_set_cell_value(table2, 2, 0, "Reading ID");
  lv_table_set_cell_value(table2, 0, 1, "--");
  lv_table_set_cell_value(table2, 1, 1, "--");
  lv_table_set_cell_value(table2, 2, 1, "--");
  lv_obj_center(table2);


  // LED Control Tab Content
  lv_obj_set_flex_flow(tab_control, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(tab_control, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  // Button for Board 1 LED
  lv_obj_t * btn1 = lv_button_create(tab_control);
  lv_obj_add_flag(btn1, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn1, 200, 50);
  lv_obj_add_event_cb(btn1, led_button_event_handler, LV_EVENT_VALUE_CHANGED, (void*)1);
  lv_obj_t * label1 = lv_label_create(btn1);
  lv_label_set_text(label1, "Board 1 LED");
  lv_obj_center(label1);

  // Button for Board 2 LED
  lv_obj_t * btn2 = lv_button_create(tab_control);
  lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn2, 200, 50);
  lv_obj_add_event_cb(btn2, led_button_event_handler, LV_EVENT_VALUE_CHANGED, (void*)2);
  lv_obj_t * label2 = lv_label_create(btn2);
  lv_label_set_text(label2, "Board 2 LED");
  lv_obj_center(label2);

  // Center the tables
  //lv_obj_center(table1);
  //lv_obj_center(table2);
}


// --- Main Setup ---
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Gateway Initializing...");

  // --- Wi-Fi Connection ---
  WiFi.mode(WIFI_STA);
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

  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = WiFi.channel(); // Use the channel we are connected on
  peerInfo.encrypt = false;          // No encryption
  
  // Add Board 1 as a peer
  memcpy(peerInfo.peer_addr, board1_mac, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 1");
    return;
  }
  
  // Add Board 2 as a peer
  memcpy(peerInfo.peer_addr, board2_mac, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 2");
    return;
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
  ssl_client.setInsecure();
  ssl_client.setConnectionTimeout(1000);
  ssl_client.setHandshakeTimeout(5);
  initializeApp(aClient, app, getAuth(user_auth), processData, "🔐 authTask");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  Serial.println("Initialization complete. Waiting for data...");
}

// --- Main Loop ---
void loop() {
  // Handle LVGL tasks
  lv_task_handler();
  lv_tick_inc(5);
  delay(5);

  // Handle Firebase async tasks
  app.loop();

  // Check queue for incoming ESP-NOW messages
  struct_message receivedData;
  if (xQueueReceive(esp_now_queue, &receivedData, 0) == pdTRUE) {
    Serial.printf("Processing data from Board ID: %d\n", receivedData.id);
    
    // --- NEW: Duration Calculation Logic ---
    if (receivedData.id == 1) {
        // Event starts: status changes to 1
        if (receivedData.status == 1 && last_status_board1 != 1) {
            start_time_board1 = millis();
            Serial.println("Board 1: Event started, timer initiated.");
        } 
        // Event ends: status changes from 1 to something else
        else if (receivedData.status != 1 && last_status_board1 == 1) {
            unsigned long duration_ms = millis() - start_time_board1;
            unsigned long duration_s = duration_ms / 1000; // Convert to seconds
            Serial.printf("Board 1: Event ended. Duration: %lu seconds.\n", duration_s);

            // Send the duration to Firebase
            if (app.ready()) {
                String path = "/board1/duration_seconds";
                Database.set<unsigned long>(aClient, path, duration_s, processData, "SendDuration1");
            }
        }
        // Update the last known status for board 1
        last_status_board1 = receivedData.status;

    } else if (receivedData.id == 2) {
        // Event starts: status changes to 1
        if (receivedData.status == 1 && last_status_board2 != 1) {
            start_time_board2 = millis();
            Serial.println("Board 2: Event started, timer initiated.");
        }
        // Event ends: status changes from 1 to something else
        else if (receivedData.status != 1 && last_status_board2 == 1) {
            unsigned long duration_ms = millis() - start_time_board2;
            unsigned long duration_s = duration_ms / 1000; // Convert to seconds
            Serial.printf("Board 2: Event ended. Duration: %lu seconds.\n", duration_s);

            // Send the duration to Firebase
            if (app.ready()) {
                String path = "/board2/duration_seconds";
                Database.set<unsigned long>(aClient, path, duration_s, processData, "SendDuration2");
            }
        }
        // Update the last known status for board 2
        last_status_board2 = receivedData.status;
    }

    // 1. Update the local display
    update_table_values(&receivedData);

    // 2. Forward the other data to Firebase
    sendDataToFirebase(&receivedData);
  }
}
