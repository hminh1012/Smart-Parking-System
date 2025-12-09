#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <vector>
#include <finalProject_inferencing.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <queue>
using std::vector;
using std::queue;
#include <utility>
#include <climits>  // For INT_MAX
#include <algorithm>  // For std::min, std::max
#include "board_config.h"
// ===================== Include file ảnh đã tạo =====================
#include "input_image.h"

// --- ESP-NOW & PERIPHERALS ---
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>

#define WAKEUP_PIN 13          // PIR Motion Sensor (GPIO 13)
#define BUZZZER_PIN 15         // Piezo Buzzer (GPIO 15)
#define PIN_WS2812B 14         // NeoPixel LED (GPIO 14)
#define NUM_PIXELS 8           // Number of NeoPixels
#define boardId 2              // ID of this board

// --- ESP-NOW DATA STRUCTURES ---
typedef struct {
  int id;
  int status;
  char CarLicense[11];
  int readingId;
} struct_message;

typedef struct led_message {
  int id;
  bool state;
} led_message;

// --- GLOBALS ---
struct_message myData;
led_message incomingData;
esp_now_peer_info_t peerInfo;

Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

uint8_t peerAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Default to Broadcast until Master found
std::vector<std::vector<uint8_t>> masters;
unsigned int readingId = 0;
bool master_found = false;


// ===================== WiFi Config =====================
const char* ssid = "BZz";
const char* password = "123456789";
String serverName = "buu.pythonanywhere.com";
String serverPath = "/upload_plate";
const int serverPort = 80;

// Server firebase 
String FIREBASE_URL = "https://parking-violation-app-default-rtdb.asia-southeast1.firebasedatabase.app/violations/A2.json";

String recognizedText;

WiFiClient client;
uint8_t* jpg_buf = NULL;
size_t jpg_buf_len = 0;

// ===================== BIẾN TOÀN CỤC CHO XỬ LÝ BIỂN SỐ =====================
int plateStartX = 0, plateStartY = 0, plateEndX = 0, plateEndY = 0;
int IMG_WIDTH = 0, IMG_HEIGHT = 0;
uint8_t* plate_gray_image = nullptr;
static bool debug_nn = true;

// ===================== CẤU TRÚC CHO KÝ TỰ ĐÃ RESIZE =====================
struct ResizedCharacter {
    uint8_t pixels[784]; // 28*28
    int originalWidth;
    int originalHeight;
    String methodName;
};

// ===================== CẤU TRÚC KẾT QUẢ DỰ ĐOÁN =====================
struct PredictionResult {
    bool success;
    float confidence;
    char predictedChar;
    int labelIndex;
    String methodName;
};

// Cấu trúc hỗ trợ
struct Point {
    int x, y;
};

struct Rect {
    int minX, minY, maxX, maxY;
};

// Struct để trả về kết quả phát hiện viền đen
struct BlackEdgeResult {
    int firstPos;
    int count;
};
// ===================== KHAI BÁO HÀM =====================
std::pair<int, int> findCharacterVerticalBounds(int startX, int endX);
bool resizeCharacterTo28x28_Nearest(int startX, int endX, ResizedCharacter& result);
PredictionResult predictCharacterFromImage(uint8_t* image_data, int img_width, int img_height, const String& methodName);
PredictionResult predictResizedCharacter(const ResizedCharacter& character);
String recognizeCharactersFromPlate();
std::vector<std::pair<int, int>> filterCharactersBySize(const std::vector<std::pair<int, int>>& characters);
void findContour(uint8_t* edge, uint8_t* visited, int w, int h,
                 int startX, int startY, vector<Point>& contour);

float calculateContourArea(const vector<Point>& contour);

Rect boundingRect(const vector<Point>& points);

// ===================== HÀM XỬ LÝ BIỂN SỐ =====================

uint8_t getPixel(int x, int y) {
    if(x < 0 || x >= IMG_WIDTH || y < 0 || y >= IMG_HEIGHT) return 0;
    return plate_gray_image[y * IMG_WIDTH + x];
}

bool isBlackPixel(int x, int y) {
    return getPixel(x, y) < 130;
}

bool isWhitePixel(int x, int y) {
    return getPixel(x, y) > 130;
}

int min_val(int a, int b) { return (a < b) ? a : b; }
int max_val(int a, int b) { return (a > b) ? a : b; }

// =============================================
// HÀM RESIZE VÀ AI PREDICTION
// =============================================

// ===================== HÀM LỌC KÝ TỰ THEO KÍCH THƯỚC =====================
std::vector<std::pair<int, int>> filterCharactersBySize(const std::vector<std::pair<int, int>>& characters) {
    std::vector<std::pair<int, int>> filteredCharacters;
    
    // Ngưỡng kích thước
    const int MIN_WIDTH = 20;   // Chiều rộng tối thiểu
    const int MIN_HEIGHT = 30;  // Chiều cao tối thiểu
    const int MAX_WIDTH = 80;   // Chiều rộng tối đa
    const int MAX_HEIGHT = 100; // Chiều cao tối đa
    
    Serial.println("\n=== LỌC KÝ TỰ THEO KÍCH THƯỚC ===");
    Serial.printf("Ngưỡng: Rộng[%d-%d], Cao[%d-%d]\n", MIN_WIDTH, MAX_WIDTH, MIN_HEIGHT, MAX_HEIGHT);
    
    for (int i = 0; i < characters.size(); i++) {
        auto region = characters[i];
        int startX = region.first;
        int endX = region.second;
        
        // Tìm chiều cao thực của ký tự
        auto verticalBounds = findCharacterVerticalBounds(startX, endX);
        int charStartY = verticalBounds.first;
        int charEndY = verticalBounds.second;
        
        int charWidth = endX - startX + 1;
        int charHeight = charEndY - charStartY + 1;
        
        Serial.printf("Ký tự %d: %dx%d pixels", i+1, charWidth, charHeight);
        
        // Kiểm tra kích thước
        if (charWidth >= MIN_WIDTH && charWidth <= MAX_WIDTH && 
            charHeight >= MIN_HEIGHT && charHeight <= MAX_HEIGHT) {
            filteredCharacters.push_back(region);
            Serial.println(" ✅ GIỮ LẠI");
        } else {
            if (charWidth < MIN_WIDTH) Serial.print(" (RỘNG QUÁ NHỎ)");
            if (charWidth > MAX_WIDTH) Serial.print(" (RỘNG QUÁ LỚN)");
            if (charHeight < MIN_HEIGHT) Serial.print(" (CAO QUÁ NHỎ)");
            if (charHeight > MAX_HEIGHT) Serial.print(" (CAO QUÁ LỚN)");
            Serial.println(" ❌ LOẠI BỎ");
        }
    }
    
    Serial.printf("Sau khi lọc: %d/%d ký tự được giữ lại\n", filteredCharacters.size(), characters.size());
    return filteredCharacters;
}

std::pair<int, int> findCharacterVerticalBounds(int startX, int endX) {
    int minY = IMG_HEIGHT, maxY = 0;
    for (int y = plateStartY; y <= plateEndY; y++) {
        for (int x = startX; x <= endX; x++) {
            if (isBlackPixel(x, y)) {
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }
    }
    if (minY > maxY) {
        return {plateStartY, plateEndY};
    }
    return {minY, maxY};
}

// ===================== HÀM PHÁT HIỆN KÝ TỰ VỚI BỘ LỌC KÍCH THƯỚC =====================
std::vector<std::pair<int, int>> findCharactersByVerticalProjection() {
    std::vector<std::pair<int, int>> characters;
    
    std::vector<int> horizontalProjection(IMG_WIDTH, 0);
    for (int x = plateStartX; x <= plateEndX; x++) {
        int blackCount = 0;
        for (int y = plateStartY; y <= plateEndY; y++) {
            if (isBlackPixel(x, y)) blackCount++;
        }
        horizontalProjection[x - plateStartX] = blackCount;
    }
    
    const int MIN_VERTICAL_DENSITY = 2;
    const int MIN_CHAR_WIDTH = 5;   // Giảm ngưỡng tối thiểu ban đầu
    const int MAX_CHAR_WIDTH = 150; // Tăng ngưỡng tối đa ban đầu
    
    int charStart = -1;
    bool inCharacter = false;
    
    for (int x = 0; x < IMG_WIDTH; x++) {
        if (horizontalProjection[x] >= MIN_VERTICAL_DENSITY) {
            if (!inCharacter) {
                charStart = x + plateStartX;
                inCharacter = true;
            }
        } else {
            if (inCharacter) {
                int charEnd = x - 1 + plateStartX;
                int width = charEnd - charStart + 1;
                
                // Chỉ lọc theo chiều rộng cơ bản trước
                if (width >= MIN_CHAR_WIDTH && width <= MAX_CHAR_WIDTH) {
                    characters.push_back({charStart, charEnd});
                }
                inCharacter = false;
            }
        }
    }
    
    if (inCharacter) {
        int charEnd = plateEndX;
        int width = charEnd - charStart + 1;
        if (width >= MIN_CHAR_WIDTH && width <= MAX_CHAR_WIDTH) {
            characters.push_back({charStart, charEnd});
        }
    }
    
    Serial.printf("Phát hiện %d ký tự (trước khi lọc)\n", characters.size());
    
    // Áp dụng bộ lọc kích thước chi tiết
    characters = filterCharactersBySize(characters);
    
    return characters;
}

bool resizeCharacterTo28x28_Nearest(int startX, int endX, ResizedCharacter& result) {
    auto verticalBounds = findCharacterVerticalBounds(startX, endX);
    int charStartY = verticalBounds.first;
    int charEndY = verticalBounds.second;
    
    int originalWidth = endX - startX + 1;
    int originalHeight = charEndY - charStartY + 1;
    
    if (originalWidth <= 0 || originalHeight <= 0) {
        return false;
    }
    
    result.originalWidth = originalWidth;
    result.originalHeight = originalHeight;
    result.methodName = "Nearest";
    
    memset(result.pixels, 255, 784);
    
    float scaleX = (float)originalWidth / 28.0f;
    float scaleY = (float)originalHeight / 28.0f;
    
    for (int targetY = 0; targetY < 28; targetY++) {
        for (int targetX = 0; targetX < 28; targetX++) {
            int srcX = startX + (int)(targetX * scaleX);
            int srcY = charStartY + (int)(targetY * scaleY);
            
            srcX = max_val(startX, min_val(endX, srcX));
            srcY = max_val(charStartY, min_val(charEndY, srcY));
            
            result.pixels[targetY * 28 + targetX] = getPixel(srcX, srcY);
        }
    }
    
    return true;
}

// ===================== Hàm AI Prediction =====================
static uint8_t* current_image_data = nullptr;

static int get_image_data(size_t offset, size_t length, float *out_ptr) {
    if (current_image_data == nullptr) {
        return -1;
    }
    
    size_t pixel_ix = offset;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        uint8_t gray = current_image_data[pixel_ix];
        out_ptr[out_ptr_ix] = (float)((gray << 16) + (gray << 8) + gray);

        out_ptr_ix++;
        pixel_ix++;
        pixels_left--;
    }
    return 0;
}

PredictionResult predictCharacterFromImage(uint8_t* image_data, int img_width, int img_height, const String& methodName) {
    PredictionResult result;
    result.success = false;
    result.confidence = 0.0f;
    result.predictedChar = '?';
    result.labelIndex = 0;
    result.methodName = methodName;

    if (EI_CLASSIFIER_RAW_SAMPLE_COUNT != img_width * img_height) {
        return result;
    }

    if (image_data == nullptr) {
        return result;
    }

    current_image_data = image_data;

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &get_image_data;

    ei_impulse_result_t ei_result = { 0 };

    EI_IMPULSE_ERROR res = run_classifier(&signal, &ei_result, debug_nn);
    if (res != EI_IMPULSE_OK) {
        current_image_data = nullptr;
        return result;
    }

    float max_prob = 0.0f;
    uint16_t max_idx = 0;
    
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float prob = ei_result.classification[i].value;
        if (prob > max_prob) {
            max_prob = prob;
            max_idx = i;
        }
    }

    const char* predicted_label = ei_classifier_inferencing_categories[max_idx];
    
    if (strlen(predicted_label) > 0) {
        result.predictedChar = predicted_label[0];
    } else {
        result.predictedChar = '?';
    }
    
    result.confidence = max_prob;
    result.labelIndex = max_idx;
    result.success = true;

    current_image_data = nullptr;
    return result;
}

PredictionResult predictResizedCharacter(const ResizedCharacter& character) {
    return predictCharacterFromImage((uint8_t*)character.pixels, 28, 28, character.methodName);
}

// ===================== HIỂN THỊ THÔNG TIN KÝ TỰ CHI TIẾT =====================
void displayCharacterDetails(const std::vector<std::pair<int, int>>& characters) {
    Serial.println("\n=== THÔNG TIN CHI TIẾT CÁC KÝ TỰ ===");
    
    for (int i = 0; i < characters.size(); i++) {
        auto region = characters[i];
        auto verticalBounds = findCharacterVerticalBounds(region.first, region.second);
        
        int width = region.second - region.first + 1;
        int height = verticalBounds.second - verticalBounds.first + 1;
        float aspectRatio = (float)width / height;
        
        Serial.printf("Ký tự %d: ", i + 1);
        Serial.printf("X[%d-%d] Y[%d-%d] ", 
                     region.first, region.second, 
                     verticalBounds.first, verticalBounds.second);
        Serial.printf("Size: %dx%d ", width, height);
        Serial.printf("Tỷ lệ: %.2f\n", aspectRatio);
    }
}

// ===================== Nhận dạng ký tự từ biển số =====================
String recognizeCharactersFromPlate() {
    Serial.println("\n=== BẮT ĐẦU NHẬN DẠNG KÝ TỰ ===");
    
    if (plate_gray_image == nullptr) {
        Serial.println("❌ Không có ảnh biển số để xử lý");
        return "";
    }
    
    // Tìm các ký tự (đã được lọc kích thước)
    std::vector<std::pair<int, int>> characters = findCharactersByVerticalProjection();
    
    if (characters.empty()) {
        Serial.println("❌ Không tìm thấy ký tự nào sau khi lọc");
        return "";
    }
    
    // Hiển thị thông tin chi tiết các ký tự
    displayCharacterDetails(characters);
    
    String recognizedText = "";
    
    // Xử lý từng ký tự
    for (int i = 0; i < characters.size(); i++) {
        auto region = characters[i];
        
        // Tính kích thước thực để hiển thị
        auto verticalBounds = findCharacterVerticalBounds(region.first, region.second);
        int charHeight = verticalBounds.second - verticalBounds.first + 1;
        int charWidth = region.second - region.first + 1;
        
        Serial.printf("\nXử lý ký tự %d: X[%d-%d] (Kích thước: %dx%d)\n", 
                     i+1, region.first, region.second, charWidth, charHeight);
        
        ResizedCharacter resizedChar;
        if (resizeCharacterTo28x28_Nearest(region.first, region.second, resizedChar)) {
            PredictionResult result = predictResizedCharacter(resizedChar);
            
            if (result.success) {
                Serial.printf("  ✅ Nhận dạng: %c (độ tin cậy: %.3f)\n", 
                             result.predictedChar, result.confidence);
                recognizedText += result.predictedChar;
            } else {
                Serial.println("  ❌ Không thể nhận dạng");
                recognizedText += "?";
            }
        } else {
            Serial.println("  ❌ Lỗi resize ký tự");
            recognizedText += "?";
        }
    }
    
    Serial.printf("\n🎯 === KẾT QUẢ NHẬN DẠNG: %s ===\n", recognizedText.c_str());
    return recognizedText;
}

// =============================================
// CÁC HÀM XỬ LÝ BIỂN SỐ GỐC
// =============================================

void calculateAutoThreshold() {
    Serial.println("\n=== TÍNH NGƯỠNG TỰ ĐỘNG ===");
    
    int cornerSamples[8] = {
        getPixel(0, 0), getPixel(IMG_WIDTH-1, 0),
        getPixel(0, IMG_HEIGHT-1), getPixel(IMG_WIDTH-1, IMG_HEIGHT-1),
        getPixel(10, 10), getPixel(IMG_WIDTH-11, 10),
        getPixel(10, IMG_HEIGHT-11), getPixel(IMG_WIDTH-11, IMG_HEIGHT-11)
    };
    
    int borderAvg = 0;
    for(int i = 0; i < 8; i++) borderAvg += cornerSamples[i];
    borderAvg /= 8;
    
    int centerX = IMG_WIDTH / 2, centerY = IMG_HEIGHT / 2;
    int centerSamples[9];
    for(int i = -1; i <= 1; i++) {
        for(int j = -1; j <= 1; j++) {
            centerSamples[(i+1)*3 + (j+1)] = getPixel(centerX + i, centerY + j);
        }
    }
    
    int centerAvg = 0;
    for(int i = 0; i < 9; i++) centerAvg += centerSamples[i];
    centerAvg /= 9;
    
    Serial.printf("→ Giá trị trung bình viền: %d\n", borderAvg);
    Serial.printf("→ Giá trị trung bình trung tâm: %d\n", centerAvg);
    Serial.printf("→ Độ chênh lệch: %d\n", abs(centerAvg - borderAvg));
}

void analyzeRegionHistogram(int startX, int endX, int startY, int endY, const char* regionName) {
    Serial.printf("\n=== PHÂN TÍCH HISTOGRAM %s ===\n", regionName);
    
    int histogram[256] = {0};
    int totalPixels = (endX - startX + 1) * (endY - startY + 1);
    
    for(int y = startY; y <= endY; y++) {
        for(int x = startX; x <= endX; x++) {
            histogram[getPixel(x, y)]++;
        }
    }
    
    int maxCount = 0, peakValue = 0;
    for(int i = 0; i < 256; i++) {
        if(histogram[i] > maxCount) {
            maxCount = histogram[i];
            peakValue = i;
        }
    }
    
    Serial.printf("→ Peak: giá trị %d (xuất hiện %d lần, chiếm %.1f%%)\n", 
                 peakValue, maxCount, (maxCount * 100.0) / totalPixels);
}

// =============================================
// HÀM QUÉT VÀ CẮT VIỀN ĐEN NÂNG CAO
// =============================================

void advancedBlackBorderRemoval() {
    Serial.println("\n=== QUÉT VIỀN ĐEN NÂNG CAO ===");
    
    const float BLACK_THRESHOLD_LOW = 0.6f;   // Ngưỡng thấp: 20%
    const float BLACK_THRESHOLD_HIGH = 0.7f;  // Ngưỡng cao: 40%
    const int SCAN_DEPTH = 40; // Quét sâu hơn: 40 pixel
    const int MIN_BLACK_STREAK = 3; // Yêu cầu ít nhất 3 hàng đen liên tiếp

    // HÀM QUÉT MỘT CẠNH VỚI NGƯỠNG LINH HOẠT
    auto scanEdgeAdvanced = [&](bool isTop, bool isHorizontal) -> int {
        int totalBlackRows = 0;
        int currentBlackStreak = 0;
        int maxBlackStreak = 0;
        bool inBlackZone = false;
        
        int start, end;
        int fixedStart, fixedEnd;
        
        if (isHorizontal) {
            start = isTop ? plateStartY : plateEndY;
            end = isTop ? min_val(plateStartY + SCAN_DEPTH, plateEndY) 
                       : max_val(plateEndY - SCAN_DEPTH, plateStartY);
            fixedStart = plateStartX;
            fixedEnd = plateEndX;
        } else {
            start = isTop ? plateStartX : plateEndX;
            end = isTop ? min_val(plateStartX + SCAN_DEPTH, plateEndX)
                       : max_val(plateEndX - SCAN_DEPTH, plateStartX);
            fixedStart = plateStartY;
            fixedEnd = plateEndY;
        }

        int step = isTop ? 1 : -1;
        int current = start;

        // DEBUG: In thông tin quét
        Serial.printf("   Quét %s từ %d đến %d:\n", 
                     isHorizontal ? (isTop ? "TRÊN" : "DƯỚI") : (isTop ? "TRÁI" : "PHẢI"),
                     start, end);

        // Quét toàn bộ vùng
        while ((isTop && current <= end) || (!isTop && current >= end)) {
            int blackCount = 0;
            int totalPixels = 0;
            
            // Đếm pixel đen
            if (isHorizontal) {
                for (int x = fixedStart; x <= fixedEnd; x++) {
                    if (!isWhitePixel(x, current)) blackCount++;
                    totalPixels++;
                }
            } else {
                for (int y = fixedStart; y <= fixedEnd; y++) {
                    if (!isWhitePixel(current, y)) blackCount++;
                    totalPixels++;
                }
            }
            
            float blackRatio = (float)blackCount / totalPixels;
            
            // Sử dụng ngưỡng thấp cho việc phát hiện
            if (blackRatio >= BLACK_THRESHOLD_LOW) {
                if (!inBlackZone) {
                    inBlackZone = true;
                }
                currentBlackStreak++;
                totalBlackRows++;
                
                // Cập nhật streak dài nhất
                if (currentBlackStreak > maxBlackStreak) {
                    maxBlackStreak = currentBlackStreak;
                }
                
                // DEBUG chi tiết
                Serial.printf("      %s %d: đen=%.1f%% ✓\n", 
                             isHorizontal ? "Hàng" : "Cột", current, blackRatio * 100);
            } else {
                if (inBlackZone) {
                    // Chỉ reset nếu streak quá ngắn
                    if (currentBlackStreak < MIN_BLACK_STREAK) {
                        totalBlackRows -= currentBlackStreak; // Không tính streak ngắn
                    }
                    currentBlackStreak = 0;
                    inBlackZone = false;
                }
                
                // DEBUG
                Serial.printf("      %s %d: đen=%.1f%% ✗\n", 
                             isHorizontal ? "Hàng" : "Cột", current, blackRatio * 100);
            }
            
            current += step;
        }

        // Chỉ trả về kết quả nếu có streak đủ dài
        if (maxBlackStreak >= MIN_BLACK_STREAK) {
            Serial.printf("   → Tổng: %d hàng đen, streak dài nhất: %d\n", totalBlackRows, maxBlackStreak);
            return totalBlackRows;
        } else {
            Serial.printf("   → Không có viền đen đáng kể (streak dài nhất: %d)\n", maxBlackStreak);
            return 0;
        }
    };

    // QUÉT VÀ CẮT CẠNH TRÊN
    Serial.println("1. Quét cạnh TRÊN:");
    int topCut = scanEdgeAdvanced(true, true);
    if (topCut > 0) {
        int newStartY = plateStartY + topCut;
        if (newStartY <= plateEndY) {
            plateStartY = newStartY;
            Serial.printf("   → Đã cắt %d hàng -> Y=%d\n", topCut, plateStartY);
        }
    }

    // QUÉT VÀ CẮT CẠNH DƯỚI - QUAN TRỌNG
    Serial.println("2. Quét cạnh DƯỚI:");
    int bottomCut = scanEdgeAdvanced(false, true);
    if (bottomCut > 0) {
        int newEndY = plateEndY - bottomCut;
        if (newEndY >= plateStartY) {
            plateEndY = newEndY;
            Serial.printf("   → Đã cắt %d hàng -> Y=%d\n", bottomCut, plateEndY);
        } else {
            Serial.println("   → Cảnh báo: Không thể cắt vì vùng quá nhỏ");
        }
    }

    // QUÉT VÀ CẮT CẠNH TRÁI
    Serial.println("3. Quét cạnh TRÁI:");
    int leftCut = scanEdgeAdvanced(true, false);
    if (leftCut > 0) {
        int newStartX = plateStartX + leftCut;
        if (newStartX <= plateEndX) {
            plateStartX = newStartX;
            Serial.printf("   → Đã cắt %d cột -> X=%d\n", leftCut, plateStartX);
        }
    }

    // QUÉT VÀ CẮT CẠNH PHẢI
    Serial.println("4. Quét cạnh PHẢI:");
    int rightCut = scanEdgeAdvanced(false, false);
    if (rightCut > 0) {
        int newEndX = plateEndX - rightCut;
        if (newEndX >= plateStartX) {
            plateEndX = newEndX;
            Serial.printf("   → Đã cắt %d cột -> X=%d\n", rightCut, plateEndX);
        }
    }
}

// =============================================
// CẬP NHẬT HÀM DEEP CROP BORDER
// =============================================

void deepCropBorder() {
    Serial.println("\n=== XÓA VIỀN SÂU NÂNG CAO ===");
    
    const int CONFIDENCE_SIDES = 85;
    const int CONFIDENCE_TOP_BOTTOM = 90;
    const int SCAN_STEP = 1;
    const int EXPAND_MARGIN = 2;
    
    // Lưu giá trị gốc
    int originalStartX = plateStartX;
    int originalStartY = plateStartY;
    int originalEndX = plateEndX;
    int originalEndY = plateEndY;
    
    // BƯỚC 1: CẮT BAN ĐẦU THEO CONFIDENCE
    Serial.println("\n--- BƯỚC 1: CẮT BAN ĐẦU ---");
    
    // VIỀN TRÁI
    Serial.println("1. XÓA VIỀN TRÁI:");
    plateStartX = 0;
    for(int x = 0; x < IMG_WIDTH; x += SCAN_STEP) {
        int whiteCount = 0;
        for(int y = 0; y < IMG_HEIGHT; y += 1) {
            if(isWhitePixel(x, y)) whiteCount++;
        }
        float whiteRatio = (whiteCount * 100.0) / IMG_HEIGHT;
        
        if(whiteRatio >= CONFIDENCE_SIDES) {
            plateStartX = min_val(IMG_WIDTH-1, x + EXPAND_MARGIN);
            Serial.printf("   → Biên trái tại X=%d (tỷ lệ trắng: %.1f%%)\n", plateStartX, whiteRatio);
            break;
        }
    }
    
    // VIỀN PHẢI
    Serial.println("2. XÓA VIỀN PHẢI:");
    plateEndX = IMG_WIDTH - 1;
    for(int x = IMG_WIDTH - 1; x >= 0; x -= SCAN_STEP) {
        int whiteCount = 0;
        for(int y = 0; y < IMG_HEIGHT; y += 1) {
            if(isWhitePixel(x, y)) whiteCount++;
        }
        float whiteRatio = (whiteCount * 100.0) / IMG_HEIGHT;
        
        if(whiteRatio >= CONFIDENCE_SIDES) {
            plateEndX = max_val(0, x - EXPAND_MARGIN);
            Serial.printf("   → Biên phải tại X=%d (tỷ lệ trắng: %.1f%%)\n", plateEndX, whiteRatio);
            break;
        }
    }
    
    // VIỀN TRÊN
    Serial.println("3. XÓA VIỀN TRÊN:");
    plateStartY = 0;
    for(int y = 0; y < IMG_HEIGHT; y += SCAN_STEP) {
        int whiteCount = 0;
        for(int x = plateStartX; x <= plateEndX; x += 1) {
            if(isWhitePixel(x, y)) whiteCount++;
        }
        float whiteRatio = (whiteCount * 100.0) / (plateEndX - plateStartX + 1);
        
        if(whiteRatio >= CONFIDENCE_TOP_BOTTOM) {
            plateStartY = min_val(IMG_HEIGHT-1, y + EXPAND_MARGIN);
            Serial.printf("   → Biên trên tại Y=%d (tỷ lệ trắng: %.1f%%)\n", plateStartY, whiteRatio);
            break;
        }
    }
    
    // VIỀN DƯỚI
    Serial.println("4. XÓA VIỀN DƯỚI:");
    plateEndY = IMG_HEIGHT - 1;
    for(int y = IMG_HEIGHT - 1; y >= 0; y -= SCAN_STEP) {
        int whiteCount = 0;
        for(int x = plateStartX; x <= plateEndX; x += 1) {
            if(isWhitePixel(x, y)) whiteCount++;
        }
        float whiteRatio = (whiteCount * 100.0) / (plateEndX - plateStartX + 1);
        
        if(whiteRatio >= CONFIDENCE_TOP_BOTTOM) {
            plateEndY = max_val(0, y - EXPAND_MARGIN);
            Serial.printf("   → Biên dưới tại Y=%d (tỷ lệ trắng: %.1f%%)\n", plateEndY, whiteRatio);
            break;
        }
    }

    // BƯỚC 2: QUÉT NÂNG CAO VIỀN ĐEN
    advancedBlackBorderRemoval();

    // BƯỚC 3: KIỂM TRA VÀ HIỆU CHỈNH
    Serial.println("\n--- HIỆU CHỈNH CUỐI ---");
    
    // Đảm bảo không cắt quá nhiều
    if (plateEndY - plateStartY < 30) {
        Serial.println("⚠️  Cảnh báo: Vùng quá nhỏ, khôi phục một phần");
        plateStartY = max(originalStartY, plateStartY - 10);
        plateEndY = min(originalEndY, plateEndY + 10);
    }

    bool isValidCrop = (plateEndX - plateStartX > 50) && 
                      (plateEndY - plateStartY > 20) &&
                      (plateStartX < plateEndX) && 
                      (plateStartY < plateEndY);
    
    Serial.println("=== KẾT QUẢ ===");
    Serial.printf("→ Kích thước: %d × %d pixel\n", 
                 plateEndX - plateStartX + 1, plateEndY - plateStartY + 1);
    Serial.printf("→ Tính hợp lệ: %s\n", isValidCrop ? "HỢP LỆ" : "CÓ VẤN ĐỀ");
    
    if(!isValidCrop) {
        Serial.println("⚠️  KHÔI PHỤC KÍCH THƯỚC GỐC");
        plateStartX = originalStartX;
        plateStartY = originalStartY;
        plateEndX = originalEndX;
        plateEndY = originalEndY;
    }
    
    analyzeRegionHistogram(plateStartX, plateEndX, plateStartY, plateEndY, "KẾT QUẢ CUỐI");
}

std::vector<std::pair<int, int>> findCharactersByRelativeWhiteGaps() {
    std::vector<std::pair<int, int>> characters;
    
    std::vector<int> blackDensity;
    for(int x = plateStartX; x <= plateEndX; x++) {
        int blackCount = 0;
        for(int y = plateStartY; y <= plateEndY; y++) {
            if(isBlackPixel(x, y)) blackCount++;
        }
        blackDensity.push_back(blackCount);
    }
    
    const int LOW_DENSITY_THRESHOLD = 3;
    const int MIN_GAP_WIDTH = 2;
    
    std::vector<std::pair<int, int>> gapRegions;
    int gapStart = -1;
    
    for(int i = 0; i < blackDensity.size(); i++) {
        int x = plateStartX + i;
        if(blackDensity[i] <= LOW_DENSITY_THRESHOLD) {
            if(gapStart == -1) gapStart = x;
        } else {
            if(gapStart != -1) {
                int gapEnd = x - 1;
                if(gapEnd - gapStart + 1 >= MIN_GAP_WIDTH) {
                    gapRegions.push_back({gapStart, gapEnd});
                }
                gapStart = -1;
            }
        }
    }
    
    // Xử lý gap cuối cùng
    if(gapStart != -1) {
        int gapEnd = plateEndX;
        if(gapEnd - gapStart + 1 >= MIN_GAP_WIDTH) {
            gapRegions.push_back({gapStart, gapEnd});
        }
    }
    
    // Phân đoạn ký tự
    if(gapRegions.empty()) {
        characters.push_back({plateStartX, plateEndX});
    } else {
        if(gapRegions[0].first > plateStartX) {
            characters.push_back({plateStartX, gapRegions[0].first - 1});
        }
        for(int i = 0; i < gapRegions.size() - 1; i++) {
            int charStart = gapRegions[i].second + 1;
            int charEnd = gapRegions[i+1].first - 1;
            if(charEnd >= charStart) {
                characters.push_back({charStart, charEnd});
            }
        }
        if(gapRegions.back().second < plateEndX) {
            characters.push_back({gapRegions.back().second + 1, plateEndX});
        }
    }
    
    Serial.printf("→ Phát hiện %d ký tự\n", characters.size());
    
    // Áp dụng bộ lọc kích thước
    characters = filterCharactersBySize(characters);
    
    return characters;
}

void displayCroppedPlate() {
    Serial.println("\n=== BIỂN SỐ SAU KHI XÓA VIỀN ===");
    
    for(int y = plateStartY; y <= plateEndY; y++) {
        for(int x = plateStartX; x <= plateEndX; x++) {
            Serial.print(isBlackPixel(x, y) ? "#" : ".");
        }
        Serial.println();
    }
}

void displayCharacter(int charNum, int startX, int endX) {
    int width = endX - startX + 1;
    
    Serial.printf("\n┌── KÝ TỰ %d (Rộng: %d pixel) ──┐\n", charNum, width);
    
    for(int y = plateStartY; y <= plateEndY; y++) {
        Serial.print("│ ");
        for(int x = startX; x <= endX; x++) {
            Serial.print(isBlackPixel(x, y) ? "█" : " ");
        }
        Serial.println(" │");
    }
    Serial.printf("└%*s┘\n", width + 2, "");
}

void processLicensePlate() {
    // PHÂN TÍCH NÂNG CAO
    calculateAutoThreshold();
    analyzeRegionHistogram(0, IMG_WIDTH-1, 0, IMG_HEIGHT-1, "TOÀN ẢNH");
    
    // XÓA VIỀN SÂU
    deepCropBorder();
    
    // PHÂN TÍCH VÙNG ĐÃ CROP
    analyzeRegionHistogram(plateStartX, plateEndX, plateStartY, plateEndY, "SAU CROP");
    
    // HIỂN THỊ VÀ PHÂN TÁCH
    displayCroppedPlate();
    
    // Tìm ký tự với bộ lọc kích thước
    std::vector<std::pair<int, int>> characters = findCharactersByRelativeWhiteGaps();
    
    Serial.printf("\n=== KẾT QUẢ: %d KÝ TỰ ===\n", characters.size());
    
    for(int i = 0; i < characters.size(); i++) {
        displayCharacter(i + 1, characters[i].first, characters[i].second);
    }

    // ===================== NHẬN DẠNG KÝ TỰ BẰNG AI =====================
    String recognizedText = recognizeCharactersFromPlate();
    Serial.printf("\n🎯 BIỂN SỐ NHẬN DẠNG: %s\n", recognizedText.c_str());
}

// ===================== CÁC HÀM CHỤP ẢNH VÀ XỬ LÝ ẢNH GỐC =====================

void bgr2rgb(uint8_t *buf, size_t pixels) {
    for (size_t i = 0; i < pixels; i++) {
        uint8_t tmp = buf[3 * i];
        buf[3 * i] = buf[3 * i + 2];
        buf[3 * i + 2] = tmp;
    }
}

uint8_t* cropLicensePlate(const uint8_t* rgb, int srcWidth, int srcHeight, 
                         int minX, int minY, int maxX, int maxY, 
                         int* cropWidth, int* cropHeight) {
    *cropWidth = maxX - minX + 1;
    *cropHeight = maxY - minY + 1;
    
    if (minX < 0) minX = 0;
    if (minY < 0) minY = 0;
    if (maxX >= srcWidth) maxX = srcWidth - 1;
    if (maxY >= srcHeight) maxY = srcHeight - 1;
    
    *cropWidth = maxX - minX + 1;
    *cropHeight = maxY - minY + 1;
    
    if (*cropWidth <= 0 || *cropHeight <= 0) {
        Serial.println("❌ Invalid crop dimensions");
        return nullptr;
    }
    
    uint8_t* cropped = (uint8_t*)malloc(*cropWidth * *cropHeight * 3);
    if (!cropped) {
        Serial.println("❌ Failed to allocate memory for cropped image");
        return nullptr;
    }
    
    for (int y = 0; y < *cropHeight; y++) {
        for (int x = 0; x < *cropWidth; x++) {
            int srcX = minX + x;
            int srcY = minY + y;
            
            int srcIndex = (srcY * srcWidth + srcX) * 3;
            int dstIndex = (y * *cropWidth + x) * 3;
            
            cropped[dstIndex] = rgb[srcIndex];
            cropped[dstIndex + 1] = rgb[srcIndex + 1];
            cropped[dstIndex + 2] = rgb[srcIndex + 2];
        }
    }
    
    Serial.printf("✅ Cropped license plate: %dx%d pixels\n", *cropWidth, *cropHeight);
    return cropped;
}

uint8_t* convertToGrayscale(const uint8_t* rgb, int width, int height) {
    size_t pixels = (size_t)width * height;
    uint8_t* gray = (uint8_t*)malloc(pixels);
    if (!gray) {
        Serial.println("❌ Failed to allocate memory for grayscale image");
        return nullptr;
    }
    
    for (size_t i = 0; i < pixels; i++) {
        uint8_t r = rgb[i * 3];
        uint8_t g = rgb[i * 3 + 1];
        uint8_t b = rgb[i * 3 + 2];
        gray[i] = (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
    }
    
    Serial.printf("✅ Converted to grayscale: %dx%d pixels\n", width, height);
    return gray;
}


// Helper function to clamp values
int clamp(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

Rect boundingRect(const std::vector<Point>& points) {
    if (points.empty()) {
        return {0, 0, -1, -1};  // Invalid rect if no points
    }
    Rect rect = {INT_MAX, INT_MAX, INT_MIN, INT_MIN};
    for (const auto& p : points) {
        if (p.x < rect.minX) rect.minX = p.x;
        if (p.y < rect.minY) rect.minY = p.y;
        if (p.x > rect.maxX) rect.maxX = p.x;
        if (p.y > rect.maxY) rect.maxY = p.y;
    }
    return rect;
}
void refinePlate(uint8_t *gray, int w, int h, int &minX, int &minY, int &maxX, int &maxY, int expand = 15) {
    // Mở rộng vùng ước lượng
    int expandedMinX = max_val(0, minX - expand);
    int expandedMinY = max_val(0, minY - expand);
    int expandedMaxX = min_val(w-1, maxX + expand);
    int expandedMaxY = min_val(h-1, maxY + expand);

    // Ngưỡng trắng: có thể điều chỉnh
    const uint8_t whiteThreshold = 200;
    const float whiteRatioThreshold = 0.8f; // 80% pixel trong hàng/cột phải là trắng

    // Tìm biên trái: quét từ trái sang
    for (int x = expandedMinX; x <= expandedMaxX; x++) {
        int whiteCount = 0;
        int total = 0;
        for (int y = expandedMinY; y <= expandedMaxY; y++) {
            if (gray[y * w + x] >= whiteThreshold) {
                whiteCount++;
            }
            total++;
        }
        float ratio = (float)whiteCount / total;
        if (ratio >= whiteRatioThreshold) {
            minX = x;
            break;
        }
    }

    // Tìm biên phải: quét từ phải sang trái
    for (int x = expandedMaxX; x >= expandedMinX; x--) {
        int whiteCount = 0;
        int total = 0;
        for (int y = expandedMinY; y <= expandedMaxY; y++) {
            if (gray[y * w + x] >= whiteThreshold) {
                whiteCount++;
            }
            total++;
        }
        float ratio = (float)whiteCount / total;
        if (ratio >= whiteRatioThreshold) {
            maxX = x;
            break;
        }
    }

    // Tìm biên trên: quét từ trên xuống
    for (int y = expandedMinY; y <= expandedMaxY; y++) {
        int whiteCount = 0;
        int total = 0;
        for (int x = expandedMinX; x <= expandedMaxX; x++) {
            if (gray[y * w + x] >= whiteThreshold) {
                whiteCount++;
            }
            total++;
        }
        float ratio = (float)whiteCount / total;
        if (ratio >= whiteRatioThreshold) {
            minY = y;
            break;
        }
    }

    // Tìm biên dưới: quét từ dưới lên
    for (int y = expandedMaxY; y >= expandedMinY; y--) {
        int whiteCount = 0;
        int total = 0;
        for (int x = expandedMinX; x <= expandedMaxX; x++) {
            if (gray[y * w + x] >= whiteThreshold) {
                whiteCount++;
            }
            total++;
        }
        float ratio = (float)whiteCount / total;
        if (ratio >= whiteRatioThreshold) {
            maxY = y;
            break;
        }
    }
}

// ===================== Optimized Detect License Plate =====================
void detectPlate(uint8_t *gray, int w, int h, int &minX, int &minY, int &maxX, int &maxY) {
    minX = w; minY = h; maxX = 0; maxY = 0;
    if (!gray || w <= 8 || h <= 8) {
        minX = w / 4; minY = h / 3; maxX = 3 * w / 4; maxY = 2 * h / 3;
        return;
    }

    size_t imgSize = (size_t)w * h;
    uint8_t *norm = (uint8_t*) malloc(imgSize);
    if (!norm) return;

    // === Improved Adaptive brightness normalization ===
    uint8_t minv = 255, maxv = 0;
    for (size_t i = 0; i < imgSize; i++) {
        if (gray[i] < minv) minv = gray[i];
        if (gray[i] > maxv) maxv = gray[i];
    }
    
    // Contrast stretching with saturation
    float contrast_factor = 1.5f;
    uint8_t range = max(1, (int)(maxv - minv));
    for (size_t i = 0; i < imgSize; i++) {
        int stretched = (int)((gray[i] - minv) * 255 / range * contrast_factor);
        norm[i] = (uint8_t)((stretched < 0) ? 0 : (stretched > 255) ? 255 : stretched);
    }

    uint8_t *edge = (uint8_t*) malloc(imgSize);
    if (!edge) { free(norm); return; }
    memset(edge, 0, imgSize);

    // === Enhanced Vertical Edge Detection ===
    for (int y = 1; y < h - 1; y++) {
        int yw = y * w;
        int y1w = (y - 1) * w;
        int y2w = (y + 1) * w;
        for (int x = 1; x < w - 1; x++) {
            int gx = norm[y1w + (x + 1)] - norm[y1w + (x - 1)];
            gx += 2 * (norm[yw + (x + 1)] - norm[yw + (x - 1)]);
            gx += norm[y2w + (x + 1)] - norm[y2w + (x - 1)];

            int gy = norm[y2w + (x - 1)] - norm[y1w + (x - 1)];
            gy += 2 * (norm[y2w + x] - norm[y1w + x]);
            gy += norm[y2w + (x + 1)] - norm[y1w + (x + 1)];

            int mag = abs(gx) * 2 + abs(gy);
            if (mag > 255) mag = 255;
            
            uint8_t local_avg = (norm[y1w + x] + norm[yw + x] + norm[y2w + x] + 
                                norm[yw + (x-1)] + norm[yw + (x+1)]) / 5;
            if (mag > max(30, local_avg / 3)) {
                edge[yw + x] = (uint8_t)mag;
            }
        }
    }

    // === Morphological Operations ===
    uint8_t *dilated = (uint8_t*) malloc(imgSize);
    if (!dilated) { free(edge); free(norm); return; }
    memcpy(dilated, edge, imgSize);
    
    // Vertical dilation to connect broken vertical edges
    for (int y = 1; y < h - 1; y++) {
        int yw = y * w;
        for (int x = 1; x < w - 1; x++) {
            if (edge[yw + x] > 0) {
                for (int dy = -1; dy <= 1; dy++) {
                    int ny = y + dy;
                    if (ny >= 0 && ny < h) {
                        dilated[ny * w + x] = max(dilated[ny * w + x], edge[yw + x]);
                    }
                }
            }
        }
    }

    // === NEW: Find and filter white regions by aspect ratio ===
    uint8_t *visited = (uint8_t*) calloc(imgSize, sizeof(uint8_t));
    if (!visited) { free(edge); free(dilated); free(norm); return; }

    #define MAX_REGIONS 50
    struct Region {
        int minX, minY, maxX, maxY;
        int area;
        float aspectRatio;
    };
    Region regions[MAX_REGIONS];
    int regionCount = 0;

    // Simple connected component analysis for white regions
    for (int y = 1; y < h - 1 && regionCount < MAX_REGIONS; y++) {
        for (int x = 1; x < w - 1 && regionCount < MAX_REGIONS; x++) {
            int idx = y * w + x;
            if (norm[idx] > 200 && !visited[idx]) { // Bright white regions
                // Flood fill or BFS for connected component
                int stack[1000];
                int stackTop = 0;
                stack[stackTop++] = idx;
                visited[idx] = 1;

                Region reg = {x, y, x, y, 0, 0.0f};

                while (stackTop > 0) {
                    int curIdx = stack[--stackTop];
                    int curX = curIdx % w;
                    int curY = curIdx / w;

                    // Update region bounds
                    if (curX < reg.minX) reg.minX = curX;
                    if (curX > reg.maxX) reg.maxX = curX;
                    if (curY < reg.minY) reg.minY = curY;
                    if (curY > reg.maxY) reg.maxY = curY;
                    reg.area++;

                    // Check 4 neighbors
                    int neighbors[4] = {curIdx - 1, curIdx + 1, curIdx - w, curIdx + w};
                    for (int i = 0; i < 4; i++) {
                        int nidx = neighbors[i];
                        if (nidx >= 0 && nidx < imgSize && !visited[nidx] && norm[nidx] > 200) {
                            visited[nidx] = 1;
                            if (stackTop < 1000 - 1) {
                                stack[stackTop++] = nidx;
                            }
                        }
                    }
                }

                // Calculate aspect ratio and filter
                int width = reg.maxX - reg.minX + 1;
                int height = reg.maxY - reg.minY + 1;
                reg.aspectRatio = (height > 0) ? (float)width / height : 0;

                // Filter: License plate aspect ratio ~4.7, area should be reasonable
                // Also filter out too thin regions (likely the white strips you mentioned)
                if (reg.area > 100 && reg.area < (w * h / 4) && 
                    reg.aspectRatio > 3.0f && reg.aspectRatio < 6.5f &&
                    width > 30 && height > 8) { // Minimum size constraints
                    regions[regionCount++] = reg;
                }
            }
        }
    }

    // Find the best candidate region (largest area with correct aspect ratio)
    int bestRegion = -1;
    float bestScore = 0.0f;
    for (int i = 0; i < regionCount; i++) {
        Region &reg = regions[i];
        
        // Score based on area and how close aspect ratio is to 4.7
        float ratioScore = 1.0f - fabs(reg.aspectRatio - 4.7f) / 4.7f;
        float areaScore = (float)reg.area / (w * h);
        float score = ratioScore * 0.6f + areaScore * 0.4f;
        
        if (score > bestScore) {
            bestScore = score;
            bestRegion = i;
        }
    }

    if (bestRegion != -1) {
        // Use the best white region as license plate candidate
        Region &best = regions[bestRegion];
        minX = best.minX;
        minY = best.minY;
        maxX = best.maxX;
        maxY = best.maxY;
    } else {
        // === FALLBACK: Original Edge-based Projection Method ===
        uint32_t *colSum = (uint32_t*) calloc(w, sizeof(uint32_t));
        uint32_t *rowSum = (uint32_t*) calloc(h, sizeof(uint32_t));
        if (!colSum || !rowSum) {
            free(visited); free(edge); free(dilated); free(norm);
            return;
        }

        // Use dilated image for projection
        for (int y = 0; y < h; y++) {
            int yw = y * w;
            for (int x = 0; x < w; x++) {
                uint8_t v = dilated[yw + x];
                colSum[x] += v;
                rowSum[y] += v;
            }
        }

        // === Improved Column Detection with Multiple Peaks ===
        uint32_t maxCol = 0;
        for (int x = 0; x < w; x++) if (colSum[x] > maxCol) maxCol = colSum[x];
        
        // Adaptive threshold với hysteresis
        uint32_t colThr_high = (uint32_t)(maxCol * 0.20f);
        uint32_t colThr_low = (uint32_t)(maxCol * 0.12f);

        // Thay thế vector bằng mảng cố định
        #define MAX_SEGMENTS 20
        int segments_start[MAX_SEGMENTS];
        int segments_end[MAX_SEGMENTS];
        int segmentCount = 0;
        
        bool inSegment = false;
        int segmentStart = 0;
        int curLen = 0;
        
        for (int x = 0; x < w && segmentCount < MAX_SEGMENTS; x++) {
            if (colSum[x] > colThr_high || (inSegment && colSum[x] > colThr_low)) {
                if (!inSegment) {
                    segmentStart = x;
                    inSegment = true;
                    curLen = 1;
                } else {
                    curLen++;
                }
            } else {
                if (inSegment) {
                    if (curLen >= w/20) { // Chỉ lưu segments đủ dài
                        segments_start[segmentCount] = segmentStart;
                        segments_end[segmentCount] = x - 1;
                        segmentCount++;
                    }
                    inSegment = false;
                }
            }
        }
        
        // Xử lý segment cuối cùng
        if (inSegment && segmentCount < MAX_SEGMENTS && curLen >= w/20) {
            segments_start[segmentCount] = segmentStart;
            segments_end[segmentCount] = w - 1;
            segmentCount++;
        }

        // Chọn segment tốt nhất dựa trên độ dài và tổng gradient
        int bestStart = 0, bestEnd = 0;
        float bestScore = 0;
        
        for (int i = 0; i < segmentCount; i++) {
            int start = segments_start[i];
            int end = segments_end[i];
            int length = end - start + 1;
            
            uint32_t segSum = 0;
            for (int x = start; x <= end; x++) {
                segSum += colSum[x];
            }
            
            float score = (float)segSum / length; // Trung bình gradient trên mỗi pixel
            if (score > bestScore) {
                bestScore = score;
                bestStart = start;
                bestEnd = end;
            }
        }

        int x0 = bestStart;
        int x1 = bestEnd;
        if (bestScore == 0) { 
            x0 = w / 8; 
            x1 = 7 * w / 8; 
        }

        // === Improved Row Detection ===
        uint32_t maxRow = 0;
        for (int y = 0; y < h; y++) {
            uint32_t s = 0;
            int yw = y * w;
            for (int x = (x0-5 > 0) ? x0-5 : 0; x <= ((x1+5 < w-1) ? x1+5 : w-1); x++) { // Mở rộng vùng tìm kiếm
                s += dilated[yw + x];
            }
            rowSum[y] = s;
            if (s > maxRow) maxRow = s;
        }

        uint32_t rowThr = (uint32_t)(maxRow * 0.15f); // Giảm ngưỡng cho hàng
        
        // Thay thế vector cho row segments
        int row_segments_start[MAX_SEGMENTS];
        int row_segments_end[MAX_SEGMENTS];
        int rowSegmentCount = 0;
        
        inSegment = false;
        segmentStart = 0;
        curLen = 0;
        
        for (int y = 0; y < h && rowSegmentCount < MAX_SEGMENTS; y++) {
            if (rowSum[y] > rowThr || (inSegment && rowSum[y] > rowThr * 0.7f)) {
                if (!inSegment) {
                    segmentStart = y;
                    inSegment = true;
                    curLen = 1;
                } else {
                    curLen++;
                }
            } else {
                if (inSegment) {
                    if (curLen >= h/15) { // Chỉ lưu segments đủ cao
                        row_segments_start[rowSegmentCount] = segmentStart;
                        row_segments_end[rowSegmentCount] = y - 1;
                        rowSegmentCount++;
                    }
                    inSegment = false;
                }
            }
        }
        
        // Xử lý segment cuối cùng
        if (inSegment && rowSegmentCount < MAX_SEGMENTS && curLen >= h/15) {
            row_segments_start[rowSegmentCount] = segmentStart;
            row_segments_end[rowSegmentCount] = h - 1;
            rowSegmentCount++;
        }

        // Chọn row segment tốt nhất
        int bestY0 = 0, bestY1 = 0;
        bestScore = 0;
        
        for (int i = 0; i < rowSegmentCount; i++) {
            int start = row_segments_start[i];
            int end = row_segments_end[i];
            int height = end - start + 1;
            
            uint32_t segSum = 0;
            for (int y = start; y <= end; y++) {
                segSum += rowSum[y];
            }
            
            float score = (float)segSum / height;
            if (score > bestScore) {
                bestScore = score;
                bestY0 = start;
                bestY1 = end;
            }
        }

        if (bestScore == 0) {
            bestY0 = h / 3; 
            bestY1 = 2 * h / 3;
        }

        minX = x0;
        maxX = x1;
        minY = bestY0;
        maxY = bestY1;

        free(colSum);
        free(rowSum);
    }

    // Final validation and expansion
    int width = maxX - minX;
    int height = maxY - minY;
    float ratio = (height > 0) ? (float)width / height : 0;
    
    // Ensure the detected region has proper license plate ratio
    if (ratio < 3.5f || ratio > 6.0f) {
        // Adjust to maintain proper aspect ratio
        int targetWidth = (int)(height * 4.7f);
        int expand_x = (targetWidth - width) / 2;
        if (expand_x > 0) {
            minX = (minX - expand_x > 0) ? minX - expand_x : 0;
            maxX = (maxX + expand_x < w-1) ? maxX + expand_x : w-1;
        }
    }

    // Add small margin
    int expand_margin_x = max(5, w / 40);
    int expand_margin_y = max(3, h / 50);
    
    minX = (minX - expand_margin_x > 0) ? minX - expand_margin_x : 0;
    maxX = (maxX + expand_margin_x < w - 1) ? maxX + expand_margin_x : w - 1;
    minY = (minY - expand_margin_y > 0) ? minY - expand_margin_y : 0;
    maxY = (maxY + expand_margin_y < h - 1) ? maxY + expand_margin_y : h - 1;

    free(visited); free(edge); free(dilated); free(norm);
}

// Gửi biến số lên Firebase 

// 🔥 HÀM GỬI LÊN FIREBASE VỚI CẢ BIỂN SỐ VÀ URL ẢNH
void updateSlotOwner(String license, String imageUrl ) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(FIREBASE_URL);
    http.addHeader("Content-Type", "application/json");

    // 🔥 TẠO JSON CHỨA CẢ BIỂN SỐ VÀ URL ẢNH
    String jsonData;
    if (imageUrl.length() > 0) {
      jsonData = "{\"licensePlate\":\"" + license + 
                "\",\"URL_image\":\"" + imageUrl + 
                 + "\"}";
    } else {
      jsonData = "{\"licensePlate\":\"" + license + 
                + "\"}";
    }

    Serial.printf("📤 Gửi dữ liệu lên Firebase: %s\n", jsonData.c_str());

    // Gửi PATCH thay vì PUT
    int httpResponseCode = http.sendRequest("PATCH", jsonData);

    if (httpResponseCode > 0) {
      Serial.printf("✅ Gửi Firebase thành công, mã: %d\n", httpResponseCode);
      String response = http.getString();
      Serial.printf("📨 Phản hồi: %s\n", response.c_str());
    } else {
      Serial.printf("❌ Gửi Firebase thất bại: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();
  } else {
    Serial.println("❌ Không kết nối WiFi");
  }
}

// 🔥 HÀM LẤY THỜI GIAN HIỆN TẠI
String getCurrentTime() {
  // Nếu bạn có RTC hoặc NTP, hãy thay thế phần này
  unsigned long currentTime = millis();
  
  // Chuyển đổi sang định dạng dễ đọc
  unsigned long seconds = currentTime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  
  seconds = seconds % 60;
  minutes = minutes % 60;
  hours = hours % 24;
  
  char timeString[20];
  snprintf(timeString, sizeof(timeString), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  
  return String(timeString);
}

// ===================== Gửi ảnh và kết quả lên server - TRẢ VỀ URL =====================
bool sendImageToServer(uint8_t* jpg_buf, size_t jpg_len, int imageNumber, String& imageUrl, const char* imageType = "full", const String& recognizedText = "") {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi not connected");
        imageUrl = ""; // Trả về URL rỗng
        return false;
    }

    HTTPClient http;
    String url = "http://" + serverName + serverPath;
    
    Serial.printf("🌐 [Image %d - %s] Connecting to server: %s\n", imageNumber, imageType, url.c_str());
    
    http.begin(client, url);
    
    http.addHeader("Content-Type", "image/jpeg");
    http.addHeader("Connection", "close");
    http.addHeader("X-Image-Number", String(imageNumber));
    http.addHeader("X-Image-Type", imageType);
    
    // Sử dụng header đúng với server
    if (recognizedText.length() > 0) {
        http.addHeader("Plate-Text", recognizedText);
        http.addHeader("Confidence", "0.95");
    }

    Serial.printf("📤 [Image %d - %s] Sending image - Size: %d bytes\n", imageNumber, imageType, jpg_len);
    if (recognizedText.length() > 0) {
        Serial.printf("📝 Recognized text: %s\n", recognizedText.c_str());
    }

    int httpResponseCode = http.POST(jpg_buf, jpg_len);
    Serial.printf("📩 [Image %d - %s] HTTP Response code: %d\n", imageNumber, imageType, httpResponseCode);

    bool success = false;
    imageUrl = ""; // Khởi tạo URL rỗng

    if (httpResponseCode == 200) {
        String response = http.getString();
        Serial.printf("✅ [Image %d - %s] Upload successful\n", imageNumber, imageType);
        
        // 🔥 PHÂN TÍCH JSON THỦ CÔNG - KHÔNG CẦN THƯ VIỆN
        Serial.println("📄 Server response: " + response);
        
        // Tìm image_url trong response
        imageUrl = extractJsonValue(response, "image_url");
        String filename = extractJsonValue(response, "filename");
        String plateText = extractJsonValue(response, "plate_text");
        
        if (imageUrl.length() > 0) {
            Serial.printf("📸 IMAGE URL: %s\n", imageUrl.c_str());
            Serial.printf("🎯 Direct link: %s\n", imageUrl.c_str());
        } else {
            Serial.println("❌ No image_url found in response");
        }
        
        if (filename.length() > 0) {
            Serial.printf("📁 Filename: %s\n", filename.c_str());
        }
        
        if (plateText.length() > 0) {
            Serial.printf("🚗 Plate: %s\n", plateText.c_str());
        }
        
        success = true;
    } else {
        Serial.printf("❌ [Image %d - %s] Error: %s\n", imageNumber, imageType, http.errorToString(httpResponseCode).c_str());
        // In response chi tiết khi lỗi
        String response = http.getString();
        if (response.length() > 0) {
            Serial.println("📄 Error response: " + response);
        }
    }

    http.end();
    return success;
}

// 🔥 HÀM TRÍCH XUẤT GIÁ TRỊ TỪ JSON - KHÔNG CẦN THƯ VIỆN
String extractJsonValue(const String& json, const String& key) {
    String searchPattern = "\"" + key + "\":\"";
    int startIndex = json.indexOf(searchPattern);
    if (startIndex == -1) {
        // Thử tìm không có dấu ngoặc kép
        searchPattern = "\"" + key + "\":";
        startIndex = json.indexOf(searchPattern);
        if (startIndex == -1) return "";
        
        startIndex += searchPattern.length();
        int endIndex = json.indexOf(",", startIndex);
        if (endIndex == -1) endIndex = json.indexOf("}", startIndex);
        if (endIndex == -1) return "";
        
        return json.substring(startIndex, endIndex);
    }
    
    startIndex += searchPattern.length();
    int endIndex = json.indexOf("\"", startIndex);
    if (endIndex == -1) return "";
    
    return json.substring(startIndex, endIndex);
}



// --- HELPER FUNCTIONS ---
void setStripColor(uint32_t color) {
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {
    WS2812B.setPixelColor(pixel, color);
  }
  WS2812B.show();
}

// --- CALLBACKS ---
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingDataBytes, int len) {
  // Check for broadcast message (Discovery)
  if (recv_info->des_addr[0] == 0xFF) { // Simple check for broadcast
    String msg = "";
    for(int i=0; i<len; i++) msg += (char)incomingDataBytes[i];

    if (msg.startsWith("DISCOVER_MASTER")) {
      Serial.println(">> Broadcast Received: " + msg);
      
      bool known = false;
      for (const auto& master : masters) {
        if (memcmp(master.data(), recv_info->src_addr, 6) == 0) {
          known = true; break;
        }
      }

      if (!known) {
        Serial.println("New Master detected! Registering...");
        std::vector<uint8_t> newMaster(recv_info->src_addr, recv_info->src_addr + 6);
        masters.push_back(newMaster);

        // Register as peer
        esp_now_peer_info_t newPeerInfo = {};
        memcpy(newPeerInfo.peer_addr, recv_info->src_addr, 6);
        newPeerInfo.channel = WiFi.channel(); // Use current WiFi channel
        newPeerInfo.encrypt = false;

        if (esp_now_add_peer(&newPeerInfo) == ESP_OK) {
          Serial.println("Master registered successfully.");
          memcpy(peerAddress, recv_info->src_addr, 6); // Update target
          master_found = true;
        } else {
          Serial.println("Failed to register Master.");
        }
      }
    }
    return;
  }

  // Handle LED Control Message
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));
    Serial.printf("Received Data from ID %d: LED %s\n", incomingData.id, incomingData.state ? "ON" : "OFF");

    if (incomingData.state) {
      setStripColor(WS2812B.Color(255, 0, 0)); // RED
    } else {
      setStripColor(WS2812B.Color(0, 255, 0)); // GREEN
    }
  }
}

void setup() {
Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // ===========================
  // CẤU HÌNH CAMERA - GIỐNG 100% CODE MẪU
  // ===========================
  camera_config_t config;
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Tối ưu hóa khi có PSRAM
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // Khởi tạo camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Điều chỉnh sensor - GIỐNG CODE MẪU
  sensor_t *s = esp_camera_sensor_get();
   if (s){
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
     }
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  
  // Giảm độ phân giải để tăng frame rate ban đầu
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_VGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif



  // ===========================
  // KẾT NỐI WiFi
  // ===========================
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Chụp ảnh test ngay sau khi khởi động
  delay(2000);

  // --- INIT PINS & WS2812B ---
  pinMode(WAKEUP_PIN, INPUT);
  pinMode(BUZZZER_PIN, OUTPUT);
  digitalWrite(BUZZZER_PIN, LOW);
  
  WS2812B.begin();
  WS2812B.clear();
  setStripColor(WS2812B.Color(0, 255, 0)); // Default GREEN

  // --- INIT ESP-NOW ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  } else {
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    
    // Attempt to register peer (if address known or for testing)
    // If master is not found yet, we wait for discovery packet in OnDataRecv
    // However, if we want to support sending even without discovery (e.g. hardcoded):
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = WiFi.channel();  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer (might be broadcast or already exists)");
    }
  }

}

void loop() {
    static unsigned long last = 0;
    static int current_image = 0;  // Sử dụng current_image thay vì imageCounter
    
    // --- PIR TRIGGER LOGIC ---
    int reading = digitalRead(WAKEUP_PIN);
    if (reading == LOW || (millis() - last < 5000)) { // Wait for High AND 5s Debounce
       delay(10); 
       return;
    }
    last = millis();
    
    Serial.println(">>> MOTION DETECTED! <<<");
    digitalWrite(BUZZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZZER_PIN, LOW);

    Serial.println("📸 Capturing image...");
  
    // Lấy frame từ camera
    camera_fb_t *fb = esp_camera_fb_get();
    
    if (!fb) {
        Serial.println("❌ Camera capture failed");
        return;
    }

    Serial.printf("✅ Captured image - Size: %d bytes, Width: %d, Height: %d\n", 
                  fb->len, fb->width, fb->height);

    // LƯU TRỮ THÔNG TIN TRƯỚC KHI GIẢI PHÓNG
    int w = fb->width;
    int h = fb->height;
    size_t jpeg_len = fb->len;
    uint8_t* jpeg_data = (uint8_t*)malloc(jpeg_len);
    if (!jpeg_data) {
        Serial.println("❌ Failed to allocate memory for jpeg data");
        esp_camera_fb_return(fb);
        return;
    }
    memcpy(jpeg_data, fb->buf, jpeg_len);  // Sao chép dữ liệu
    
    // 🔥 Gửi ảnh gốc lên server
    String tempUrl;
    if (sendImageToServer(fb->buf, fb->len, current_image, tempUrl, "full")) {
        current_image++;
    }
    
    // Giải phóng frame buffer
    esp_camera_fb_return(fb);

    Serial.printf("🖼️ Processing image, size: %d bytes, dimensions: %dx%d\n", jpeg_len, w, h);

    size_t pixels = (size_t)w * h;
    uint8_t *rgb = (uint8_t*)malloc(pixels * 3);
    if (!rgb) {
        Serial.println("❌ malloc rgb failed");
        free(jpeg_data);
        return;
    }

    bool converted = fmt2rgb888(jpeg_data, jpeg_len, PIXFORMAT_JPEG, rgb);
    free(jpeg_data);  // Giải phóng dữ liệu JPEG đã sao chép
    
    if (!converted) {
        Serial.println("❌ JPEG to RGB conversion failed");
        free(rgb);
        return;
    }
    
    bgr2rgb(rgb, pixels);

    uint8_t *gray = (uint8_t*)malloc(pixels);
    if (!gray) { 
        free(rgb); 
        return; 
    }
    
    for (size_t i = 0; i < pixels; i++) {
        uint8_t r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        gray[i] = (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
    }

    int minX, minY, maxX, maxY;
    detectPlate(gray, w, h, minX, minY, maxX, maxY);
    Serial.printf("📦 Detected Region: x=%d, y=%d, w=%d, h=%d\n", 
                  minX, minY, maxX - minX, maxY - minY);

    int cropWidth, cropHeight;
    uint8_t* croppedRGB = cropLicensePlate(rgb, w, h, minX, minY, maxX, maxY, &cropWidth, &cropHeight);
    
    free(rgb);
    free(gray);
    
    if (croppedRGB) {
        Serial.println("\n🎯 BẮT ĐẦU XỬ LÝ BIỂN SỐ VÀ PHÂN TÁCH KÝ TỰ");
        
        if (plate_gray_image) {
            free(plate_gray_image);
            plate_gray_image = nullptr;
        }
        
        plate_gray_image = convertToGrayscale(croppedRGB, cropWidth, cropHeight);
        
        if (plate_gray_image) {
            IMG_WIDTH = cropWidth;
            IMG_HEIGHT = cropHeight;
            
            plateStartX = 0; plateStartY = 0;
            plateEndX = IMG_WIDTH-1; plateEndY = IMG_HEIGHT-1;
            
            processLicensePlate();
            
            free(plate_gray_image);
            plate_gray_image = nullptr;
        }
        
        // Gửi ảnh biển số lên server
        uint8_t* plate_jpg_buf = NULL;
        size_t plate_jpg_buf_len = 0;
        
        bool plate_ok = fmt2jpg(croppedRGB, cropWidth * cropHeight * 3, 
                               cropWidth, cropHeight, PIXFORMAT_RGB888, 
                               90, &plate_jpg_buf, &plate_jpg_buf_len);
        free(croppedRGB);
    
        if (plate_ok) {
            String imageUrl;
            bool plateSuccess = sendImageToServer(plate_jpg_buf, plate_jpg_buf_len, 
                                                 current_image, imageUrl, "plate", recognizedText);
            
            // 🔥 SỬ DỤNG URL ẢNH ĐỂ GỬI LÊN FIREBASE
            if (plateSuccess && imageUrl.length() > 0) {
                Serial.printf("🚀 Gửi lên Firebase: Biển số=%s, URL ảnh=%s\n", 
                            recognizedText.c_str(), imageUrl.c_str());
                updateSlotOwner(recognizedText, imageUrl);
            } else if (plateSuccess) {
                Serial.printf("🚀 Gửi lên Firebase: Biển số=%s (không có ảnh)\n", 
                            recognizedText.c_str());
                updateSlotOwner(recognizedText, "");
            }
            
            free(plate_jpg_buf);
        }
    }

    // --- SEND ESP-NOW DATA ---
    Serial.println("📡 Preparing ESP-NOW Packet...");
    myData.id = boardId;
    memset(myData.CarLicense, 0, sizeof(myData.CarLicense));
    if (recognizedText.length() > 0) {
       snprintf(myData.CarLicense, 10, "%s", recognizedText.c_str());
    } else {
       strcpy(myData.CarLicense, "UNKNOWN");
    }
    myData.readingId = readingId++;
    myData.status = 1;

    esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &myData, sizeof(myData));
    Serial.print("ESP-NOW Send Result: ");
    Serial.println(result == ESP_OK ? "Success" : "Fail");

    Serial.println("✅ Done processing!\n");
    // Removed delay(10000) to allow PIR polling (debounce handled at start)
}
