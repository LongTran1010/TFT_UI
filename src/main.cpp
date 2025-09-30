//#define USE_TF02_PRO

#include "TFT.h"
TFTPins pins{5,19,21,18,23,-1};  // CS=5, DC=19, RST=21, SCK=18, MOSI=23
TFTDistance display(pins);

#if defined(USE_TF02_PRO)
// UART TF02 Pro
HardwareSerial& LRF = Serial2;
const int PIN_RX2 = 16; // TF02 TX -> RX2(16)
const int PIN_TX2 = 17; // TF02 RX -> TX2(17)

// Parser state (giữ trong task đọc)
static uint8_t buf9[9];
static int st = 0, idx = 0;
#endif

// ====== RTOS glue ======
struct LidarMsg {
  uint16_t cm;
  uint16_t strength;
  int16_t  temp_x8;
  bool     valid;
  uint32_t ts_ms;
};

// Queue: dùng 1 phần tử (luôn chỉ cần bản mới nhất)
static QueueHandle_t qLidar;

#if defined(USE_TF02_PRO)
static bool parseTF02(uint16_t &cm, uint16_t &strength, int16_t &temp_x8) {
  bool any = false;
  while (LRF.available()) {
    any = true;
    uint8_t b = LRF.read();
    switch (st) {
      case 0: if (b == 0x59) { buf9[0] = b; st = 1; } break;
      case 1: if (b == 0x59) { buf9[1] = b; idx = 2; st = 2; } else st = 0; break;
      case 2:
        buf9[idx++] = b;
        if (idx >= 9) {
          uint8_t sum = 0; for (int i = 0; i < 8; ++i) sum += buf9[i];
          bool ok = (sum == buf9[8]);
          st = 0; idx = 0;
          if (ok) {
            cm       = (uint16_t)buf9[2] | ((uint16_t)buf9[3] << 8);
            strength = (uint16_t)buf9[4] | ((uint16_t)buf9[5] << 8);
            temp_x8  = (int16_t)((uint16_t)buf9[6] | ((uint16_t)buf9[7] << 8));
            return true; // trả ngay frame hợp lệ mới nhất
          }
        }
        break;
    }
  }
  (void)any;
  return false;
}
#endif

// --------- TASKS ----------
void TaskLRF(void*){
#if defined(USE_TF02_PRO)
  const uint16_t MIN_STRENGTH = 30;
  for(;;){
    uint16_t cm=0, strength=0; int16_t temp_x8=0;
    bool got = parseTF02(cm, strength, temp_x8);
    if (got) {
      LidarMsg m;
      m.cm = cm;
      m.strength = strength;
      m.temp_x8 = temp_x8;
      m.valid = (strength >= MIN_STRENGTH) && (cm > 0);
      m.ts_ms = millis();
      // ghi đè bản mới nhất để UI luôn thấy dữ liệu “current”
      xQueueOverwrite(qLidar, &m);
    }
    // nhường CPU, vẫn đọc rất nhanh
    vTaskDelay(pdMS_TO_TICKS(2));
  }
#else
  // Chế độ giả lập (không TF02)
  float m = 0.0f; float step = 0.6f;
  for(;;){
    m += step;
    if (m >= 30.0f) { m = 30.0f; step = -step; }
    if (m <= 0.0f)  { m = 0.0f;  step = -step; }
    LidarMsg msg{ (uint16_t)(m*100.0f), 100, 25*8, true, millis() };
    xQueueOverwrite(qLidar, &msg);
    vTaskDelay(pdMS_TO_TICKS(120));
  }
#endif
}

void TaskUI(void*){
  // Nhịp cập nhật UI (mượt mà, không quá dày): 50–100ms
  const TickType_t period = pdMS_TO_TICKS(80);
  TickType_t last = xTaskGetTickCount();

  LidarMsg latest{}; bool has = false;

  for(;;){
    // Hút hết bản tin có sẵn (nếu nhiều, giữ cái cuối)
    while (xQueueReceive(qLidar, &latest, 0) == pdTRUE) has = true;

    if (has) {
      display.updateDistanceCm(latest.cm, latest.valid);
    } else {
      // Không có mới → vẫn gọi update để cơ chế stale tự kích hoạt
      display.updateDistanceCm(0, /*valid=*/false);
    }

    vTaskDelayUntil(&last, period);
  }
}

// --------- Arduino setup/loop ----------
void setup() {
  Serial.begin(115200);

  // Màn hình
  display.begin();
  display.setMaxRangeMeters(30.0f);
  display.setSmoothing(0.25f);
  display.setStaleTimeoutMs(1000);

#if defined(USE_TF02_PRO)
  // UART LiDAR
  LRF.begin(115200, SERIAL_8N1, PIN_RX2, PIN_TX2);
#endif

  // Queue 1 phần tử (xQueueOverwrite dùng được)
  qLidar = xQueueCreate(1, sizeof(LidarMsg));

  // Tạo task (có thể pin vào core nếu muốn)
  xTaskCreatePinnedToCore(TaskLRF, "LRF", 4096, nullptr, 3, nullptr, 1); // ưu tiên cao hơn UI
  xTaskCreatePinnedToCore(TaskUI , "UI" , 4096, nullptr, 2, nullptr, 1);
}

void loop() {
  // Không dùng nữa; FreeRTOS lo lịch
  vTaskDelay(pdMS_TO_TICKS(1000));
}
