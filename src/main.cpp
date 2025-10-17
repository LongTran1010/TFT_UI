#include "TFT.h"
#include "MeasurementTypes.h"  // MeasStatus, Measurement
//#include "TF02-Pro.h"        // TF02ProDriver
#include "TC22.h"          // TC22Driver

// TFT
TFTPins pins{5,19,21,18,23,-1};  // CS=5, DC=2, RST=21, SCK=18, MOSI=23, MISO=-1
TFTDistance display(pins);

#if defined(USE_TC22)
// TC22trên UART2 (ESP32): RX2=16, TX2=17
HardwareSerial& LRF = Serial2; 
TC22Driver lrf(LRF, /*RX*/16, /*TX*/17);
#endif

// Tính FPS dựa trên các mẫu OK liên tiếp
static float computeFPS() {
  static uint32_t lastOk = 0;
  static float fps = 0.0f;
  uint32_t now = millis();
  if (lastOk != 0) {
    float dt = (now - lastOk) / 1000.0f;
    if (dt > 0.001f && dt < 2.0f) fps = 1.0f / dt;
  }
  lastOk = now;
  return fps;
}

void setup() {
  Serial.begin(115200);

  // TFT UI
  display.begin();
  display.setMaxRangeMeters(30.0f);
  display.setSmoothing(0.25f); // Set EMA ở 0.25
  display.setStaleTimeoutMs(1000);

// #if defined(USE_TF02_PRO)
//   // TF02-Pro
//   lrf.begin(115200);          // cấu hình UART + reset parser
//   lrf.setTimeoutMs(120);      // timeout khung dở
//   lrf.setStrengthMin(30);     // ngưỡng tín hiệu yếu
// #endif
// }

#if defined(USE_TC22)
  // TC22
  lrf.begin(115200);          // cấu hình UART + reset parser
  delay(50);     // timeout khung dở
  lrf.startContinuous();     // ngưỡng tín hiệu yếu
#endif
}

void loop() {
// #if defined(USE_TF02_PRO)
//   static uint32_t lastAny = millis(); // để báo TIMEOUT khi lâu không có mẫu

//   Measurement m;
//   if (lrf.poll(m)) {
//     lastAny = millis();

//     if (m.status == MEAS_OK) {
//       float fps = computeFPS();
//       display.update(m, fps);           // cập nhật số đo + overlay FPS/Status
//     } else {
//       // Không đưa mẫu lỗi vào EMA; chỉ cập nhật overlay
//       display.setStatus(m.status);
//       display.setFPS(0.0f);
//     }
//   } else {
//     // Không nhận được mẫu hoàn chỉnh trong một khoảng ngắn -> overlay TIMEOUT
//     if (millis() - lastAny > 300) {
//       display.setStatus(MEAS_TIMEOUT);
//       display.setFPS(0.0f);
//       // Vùng số sẽ tự về "---.- m" khi stale > 1000 ms (đã cấu hình)
//     }
//   }
#if defined(USE_TC22)
  static uint32_t lastAny = millis();
  Measurement m;
  if (lrf.poll(m)) {
    lastAny = millis();
    if (m.status == MEAS_OK) {
      float fps = computeFPS();
      Serial.printf("Start parse frame,%lu\n", (unsigned long)m.t_ms); //Điểm bắt đầu tính độ trễ end--to-end (in trước khi lọc)
      display.update(m, fps);           // cập nhật số đo + overlay FPS/Status
      Serial.printf("%lu,%.3f,%.3f,%u,%.1f\n",
        (unsigned long)m.t_ms, 
        m.dist_m, 
        display.getFilter(), 
        -1, //ko trả strength
        fps);
      Serial.printf("render TFT,%lu\n", (unsigned long)millis()); //Điểm kết thúc tính độ trễ end--to-end (in ngay sau khi vẽ xong TFT)
    } else {
      // Không đưa mẫu lỗi vào EMA; chỉ cập nhật overlay
      display.setStatus(m.status);
      display.setFPS(0.0f);
    }
  } else {
    // Không nhận được mẫu hoàn chỉnh trong một khoảng ngắn -> overlay TIMEOUT
    if (millis() - lastAny > 300) {
      display.setStatus(MEAS_TIMEOUT);
      display.setFPS(0.0f);
      // Vùng số sẽ tự về "---.- m" khi stale > 1000 ms (đã cấu hình)
    }
  }
#else
  // ----------- FAKE DEMO (không cần TF02) -----------
  static uint32_t last = 0;
  static float mval = 0.0f;
  static float step = 0.6f;

  if (millis() - last > 120) {        // ~8.3 Hz
    last = millis();
    mval += step;
    if (mval >= 30.0f) { mval = 30.0f; step = -step; }
    if (mval <= 0.0f)  { mval = 0.0f;  step = -step; }

    Measurement meas{};
    meas.t_ms     = millis();
    meas.dist_m   = mval;
    meas.strength = 100;
    meas.temp_C   = NAN;
    meas.status   = MEAS_OK;

    display.update(meas, 8.3f);
  }
#endif
}
