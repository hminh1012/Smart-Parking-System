/*********
 * Combined code for an ESP32 Gateway.
 * This device:
 * 1. Receives data from other ESP32 boards via ESP-NOW.
 * 2. Displays the received data on a local TFT screen using LVGL.
 * 3. Forwards the data to a Firebase Realtime Database.
 * 4. NEW: Calculates the duration when a board's status is '1' (occupied)
 * and sends the duration and revenue to Firebase when the status
 * changes back to '0' (available).
 *
 * All necessary configurations and credentials are included in this single file.
 *
 * --- FIXES APPLIED ---
 * 1. Commented out all references to the missing 'image.h' file to allow compilation.
 * 2. Corrected Firebase path in 'led_button_event_handler' to match JSON structure.
 * 3. Removed 'sendDataToFirebase' function and integrated its logic directly
 * into the main loop.
 * 4. Rewrote the main loop's processing logic to:
 * - Send "occupied" / "available" strings for status.
 * - Send license plate to '/currentVehicle/licensePlate'.
 * - On departure, remove '/currentVehicle' node.
 * - On departure, calculate duration (in hours) and revenue, and send them
 * to the spot's '/duration' and '/revenue' fields.
 * 5. Added a warning for the touchscreen calibration values.
 **********
*/
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
// --- FIX: --- Commented out missing image file
// #include <image.h> 
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
  int status;      // Application-defined status (1 for occupied, 0 for available)
  char CarLicense[11]; // Null-terminated string for car license
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
// NOTE: Status 0 = available, 1 = occupied
int last_status_board1 = 0;
int last_status_board2 = 0;
unsigned long start_time_board1 = 0; // Time in millis()
unsigned long start_time_board2 = 0; // Time in millis()


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
        // --- FIX: --- Corrected the Firebase path to match the JSON structure
        String path = "/parkingLots/mainStreetGarage/spots/A0" + String(board_id) + "/led_status";
        Database.set<String>(aClient, path, led_state ? "on" : "off", processData, "UpdateLEDStatus");
    }
}

// Function to update the tables on the LVGL display
void update_table_values(struct_message *myData) {
  // --- FIX: --- Show "Occupied" / "Available" instead of 1 / 0
  const char* status_str = (myData->status == 1) ? "Occupied" : "Available";
  
  if (myData->id == 1) {
    lv_table_set_cell_value(table1, 0, 1, status_str);
    lv_table_set_cell_value(table1, 1, 1, (myData->status == 1) ? myData->CarLicense : "--");
    lv_table_set_cell_value(table1, 2, 1, String(myData->readingId).c_str());
  } else if (myData->id == 2) {
    lv_table_set_cell_value(table2, 0, 1, status_str);
    lv_table_set_cell_value(table2, 1, 1, (myData->status == 1) ? myData->CarLicense : "--");
    lv_table_set_cell_value(table2, 2, 1, String(myData->readingId).c_str());
  }
}

// --- FIX: --- Removed the old 'sendDataToFirebase' function.
// The logic is now integrated into the main loop.


// --- Touchscreen and Display Driver Functions ---
// (These are standard setup functions for the display and LVGL)

void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

// --- FIX: --- Commented out unused function that depended on missing 'image.h'
/*
void draw_image(void) {
  LV_IMAGE_DECLARE(my_image);
  lv_obj_t * img1 = lv_image_create(lv_screen_active());
  lv_image_set_src(img1, &my_image);
  lv_img_set_zoom(img1, 128);
  lv_obj_align(img1, LV_ALIGN_TOP_LEFT, 0, 0);
}
*/

// --- FIX: --- Added a global declaration for the image, as it's needed by LVGL
// even if the file isn't present. Comment it out if you get errors.
// LV_IMAGE_DECLARE(my_image);


// Get the Touchscreen data
void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z)
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();

    // --- FIX: ---
    // --- WARNING! ---
    // These calibration values are placeholders. You MUST run a calibration
    // sketch for your specific XPT2046 screen to get accurate touch points.
    // Your touch coordinates will be incorrect until you replace these.
    // --- WARNING! ---
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
  // --- FIX: --- This needs to be declared even if the image is commented out,
  // to prevent a compile error in the line below.
  LV_IMAGE_DECLARE(my_image);
  lv_obj_t * tabview = lv_tabview_create(lv_screen_active());
  lv_tabview_set_tab_bar_size(tabview, 40);
  
  lv_obj_t * tab1 = lv_tabview_add_tab(tabview, "BOARD #1");
  lv_obj_t * tab2 = lv_tabview_add_tab(tabview, "BOARD #2");
  lv_obj_t * tab_control = lv_tabview_add_tab(tabview, "LED Control");

  //config tab1
  // --- FIX: --- Commented out image display
  // lv_obj_t * img1 = lv_image_create(tab1);
  // lv_image_set_src(img1, &my_image);
  // lv_obj_align(img1, LV_ALIGN_TOP_LEFT, 0, 0);

  table1 = lv_table_create(tab1);
  lv_table_set_cell_value(table1, 0, 0, "Status");
  lv_table_set_cell_value(table1, 1, 0, "Car License");
  lv_table_set_cell_value(table1, 2, 0, "Reading ID");
  lv_table_set_cell_value(table1, 0, 1, "--");
  lv_table_set_cell_value(table1, 1, 1, "--");
  lv_table_set_cell_value(table1, 2, 1, "--");
  lv_obj_center(table1);
  
  //config tab2
  // --- FIX: --- Commented out image display
  // lv_obj_t * img2 = lv_image_create(tab2);
  // lv_image_set_src(img2, &my_image);
  // lv_obj_align(img2, LV_ALIGN_TOP_LEFT, 0, 0);

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
  peerInfo.encrypt = false;       // No encryption
  
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
    Serial.printf("Processing data from Board ID: %d, Status: %d\n", receivedData.id, receivedData.status);
    
    // --- FIX: --- Refactored logic to use pointers for cleaner code
    int* last_status;
    unsigned long* start_time;
    
    if (receivedData.id == 1) {
      last_status = &last_status_board1;
      start_time = &start_time_board1;
    } else if (receivedData.id == 2) {
      last_status = &last_status_board2;
      start_time = &start_time_board2;
    } else {
      Serial.println("Received data from unknown board ID.");
      return; // Ignore unknown board ID
    }

    // --- FIX: --- This is the new, combined logic block.
    // It updates Firebase based on state changes (car arrival/departure).
    
    String spotPath = "/parkingLots/mainStreetGarage/spots/A0" + String(receivedData.id);
    int current_status_int = receivedData.status; // 1 = occupied, 0 = available

    // Event starts: Car arrives (status changes to 1)
    if (current_status_int == 1 && *last_status != 1) {
      *start_time = millis();
      Serial.printf("Board %d: Car arrived. Timer started.\n", receivedData.id);

      if (app.ready()) {
        // Set status to "occupied"
        Database.set<String>(aClient, spotPath + "/status", "occupied", processData, "SetStatusOccupied");
        
        // Set current vehicle license plate
        Database.set<String>(aClient, spotPath + "/currentVehicle/licensePlate", String(receivedData.CarLicense), processData, "SetLicense");
        
        // Clear last session's duration/revenue
        Database.remove(aClient, spotPath + "/duration", processData, "ClearDuration");
        Database.remove(aClient, spotPath + "/revenue", processData, "ClearRevenue");
      }
    } 
    // Event ends: Car leaves (status changes from 1 to 0)
    else if (current_status_int != 1 && *last_status == 1) {
      unsigned long duration_ms = millis() - *start_time;
      
      // Calculate duration in hours (for revenue)
      // (float)duration_ms / (milliseconds_per_second * seconds_per_minute * minutes_per_hour)
      float duration_h = (float)duration_ms / (1000.0f * 60.0f * 60.0f); 
      
      // Get hourly rate (hardcoded to 2.5 based on your JSON)
      float hourly_rate = 2.5;
      float revenue = duration_h * hourly_rate; 

      Serial.printf("Board %d: Car left. Duration: %.4f hours. Revenue: $%.2f\n", receivedData.id, duration_h, revenue);

      if (app.ready()) {
        // Set status to "available"
        Database.set<String>(aClient, spotPath + "/status", "available", processData, "SetStatusAvailable");
        
        // Remove the currentVehicle node entirely
        Database.remove(aClient, spotPath + "/currentVehicle", processData, "ClearVehicle");
        
        // Set the duration and revenue for the completed session
        Database.set<float>(aClient, spotPath + "/duration", duration_h, processData, "SetDuration");
        Database.set<float>(aClient, spotPath + "/revenue", revenue, processData, "SetRevenue");
      }
    }
    
    // Update the last known status
    *last_status = current_status_int;


    // 1. Update the local display
    update_table_values(&receivedData);

    // 2. Forwarding to Firebase is now handled *inside* the logic above.
    // The old sendDataToFirebase(&receivedData); call is no longer needed.
  }
}
