#include "TFT.h"

//Khởi tạo SPI vs panel TFT
void TFTDistance::begin() {
  SPI.begin(pins_.sck, pins_.miso, pins_.mosi, pins_.cs); //remap
  tft_.initR(INITR_BLACKTAB);           // nếu màu lệch thử GREENTAB/REDTAB
  drawStatic(); //vẽ khung label + thanh mức
}

// full-scale cho thanh mức (mặc định max 30.0)
void TFTDistance::setMaxRangeMeters(float m) {
  if (m < 1.0f) m = 1.0f; //chặn dưới 1m
  maxRange_m_ = m;
}

// Tracking 
void TFTDistance::setSmoothing(float alpha) {
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  alpha_ = alpha;
}

// Giới hạn thời gian dữ liệu cũ
void TFTDistance::setStaleTimeoutMs(uint32_t ms) { staleTimeoutMs_ = ms; }

// Cập nhật số đo (cm) -> (m) + trạng thái hợp lệ
void TFTDistance::updateDistanceCm(uint16_t dist_cm, bool valid) {
  if(valid){ //Nếu dữ liệu hợp lệ
    float m = dist_cm / 100.0f;
    if(isnan(distEMA_m_)){
        distEMA_m_ = m;
    }else{                   
        distEMA_m_ = alpha_ * m + (1.0f - alpha_) * distEMA_m_;
    }
    lastUpdateMs_ = millis();
  }
  paintDistanceUI();
}

// ---------------- UI ----------------

void TFTDistance::drawStatic() {
  tft_.fillScreen(C_BLACK); //clear

  // Header
  tft_.drawRect(0, 0, 128, 28, C_CYAN);
  tft_.setCursor(4, 4);
  tft_.setTextColor(C_ORANGE, C_BLACK);
  tft_.setTextSize(1);
  tft_.print("DISTANCE(m)");

  // Thanh mức
  tft_.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, C_ORANGE);

  // Nhãn 0 & max
  tft_.setTextColor(C_WHITE, C_BLACK);
  tft_.setTextSize(1);
  tft_.setCursor(4, 48); tft_.print("0");
  String sMax = String((int)maxRange_m_);
  tft_.setCursor(128 - (int)sMax.length()*6 - 2, 48); tft_.print(sMax);

  // Clear vùng số lần đầu
  tft_.fillRect(2, 14, 124, 12, C_BLACK);
}

//In số lớn
void TFTDistance::printDistanceValue(const String& s, uint16_t color) {
  tft_.fillRect(2, 14, 124, 12, C_BLACK);     // xóa số cũ
  tft_.setTextColor(color, C_BLACK);
  tft_.setTextSize(2);
  int x = (s.length() <= 5) ? 20 : 4;         // canh tương đối
  tft_.setCursor(x, 12);
  tft_.print(s);
}

// Vẽ số + thanh mức theo trạng thái hiện tại (hiển thị stale khi mất dữ liệu)
void TFTDistance::paintDistanceUI() {
  bool stale = (millis() - lastUpdateMs_) > staleTimeoutMs_;

  if (isnan(distEMA_m_) || stale) {
    printDistanceValue(String("---.- m"), C_RED);
    tft_.fillRect(BAR_X+1, BAR_Y+1, BAR_W-2, BAR_H-2, C_BLACK);
    tft_.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, C_ORANGE);
    return;
  }

  String s = String(distEMA_m_, 1) + " m";
  printDistanceValue(s, C_YELLOW);

  float r = distEMA_m_ / maxRange_m_;
  if (r < 0) r = 0; if (r > 1) r = 1;
  int fillW = (int)(r * (BAR_W - 2));

  tft_.fillRect(BAR_X+1, BAR_Y+1, BAR_W-2, BAR_H-2, C_BLACK);
  tft_.fillRect(BAR_X+1, BAR_Y+1, fillW,      BAR_H-2, C_GREEN);
  tft_.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, (distEMA_m_ >= maxRange_m_) ? C_RED : C_ORANGE);
}
