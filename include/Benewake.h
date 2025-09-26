#pragma once
#include <Arduino.h>

class BenewakeTF {
public:
  struct Frame {
    uint16_t dist_cm = 0;
    uint16_t strength = 0;
    int16_t  temp_x8 = 0; // nhiệt độ * 8
    bool     valid = false;
  };

  // Cho phép map Serial & chân tùy biến
  bool begin(HardwareSerial& serial, int rxPin, int txPin, uint32_t baud = 115200) {
    ser_ = &serial;
    ser_->begin(baud, SERIAL_8N1, rxPin, txPin);
    return true;
  }

  // Gọi thường xuyên trong loop; trả về true khi bắt được 1 frame hợp lệ
  bool read(Frame& out) {
    while (ser_ && ser_->available()) {
      uint8_t b = ser_->read();
      switch (state_) {
        case 0: if (b == 0x59) { buf_[0] = b; state_ = 1; } break;
        case 1: if (b == 0x59) { buf_[1] = b; idx_ = 2; state_ = 2; } else state_ = 0; break;
        case 2:
          buf_[idx_++] = b;
          if (idx_ >= 9) {
            uint8_t sum = 0; for (int i = 0; i < 8; ++i) sum += buf_[i];
            if (sum == buf_[8]) {
              out.dist_cm  = (uint16_t)buf_[2] | ((uint16_t)buf_[3] << 8);
              out.strength = (uint16_t)buf_[4] | ((uint16_t)buf_[5] << 8);
              out.temp_x8  = (int16_t)((uint16_t)buf_[6] | ((uint16_t)buf_[7] << 8));
              out.valid = true;
            } else {
              out.valid = false;
            }
            state_ = 0; idx_ = 0;
            return out.valid;
          }
          break;
      }
    }
    return false;
  }

private:
  HardwareSerial* ser_ = nullptr;
  uint8_t buf_[9];
  int state_ = 0, idx_ = 0;
};
