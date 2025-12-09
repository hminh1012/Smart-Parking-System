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

// ===================== Include file ảnh đã tạo =====================
#include "input_image.h"


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

void setup() {
    Serial.begin(115200);
}

void loop() {
    static unsigned long last = 0;
    static int current_image = 0;
    
    if (millis() - last < 5000) return;
    last = millis();

    // THAY THẾ PHẦN NÀY BẰNG CODE CHỤP ẢNH THỰC TẾ
    const uint8_t* jpeg_data = img_jpeg_0;
    size_t jpeg_len = img_jpeg_0_len;
    int w = img_jpeg_0_width;
    int h = img_jpeg_0_height;

    Serial.printf("🖼️ Processing image, size: %d bytes, dimensions: %dx%d\n", jpeg_len, w, h);

    size_t pixels = (size_t)w * h;
    uint8_t *rgb = (uint8_t*) malloc(pixels * 3);
    if (!rgb) {
        Serial.println("❌ malloc rgb failed");
        return;
    }

    bool converted = fmt2rgb888(jpeg_data, jpeg_len, PIXFORMAT_JPEG, rgb);
    if (!converted) {
        Serial.println("❌ JPEG to RGB conversion failed");
        free(rgb);
        return;
    }

    bgr2rgb(rgb, pixels);

    uint8_t *gray = (uint8_t*) malloc(pixels);
    if (!gray) { free(rgb); return; }
    for (size_t i = 0; i < pixels; i++) {
        uint8_t r = rgb[i * 3], g = rgb[i * 3 + 1], b = rgb[i * 3 + 2];
        gray[i] = (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
    }

    int minX, minY, maxX, maxY;
    detectPlate(gray, w, h, minX, minY, maxX, maxY);
    Serial.printf("📦 Detected Region: x=%d, y=%d, w=%d, h=%d\n", minX, minY, maxX - minX, maxY - minY);

    int cropWidth, cropHeight;
    uint8_t* croppedRGB = cropLicensePlate(rgb, w, h, minX, minY, maxX, maxY, &cropWidth, &cropHeight);
    
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
    }

    free(rgb);
    free(gray);

    Serial.println("✅ Done processing!\n");
    delay(10000);
}

