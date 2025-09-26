//#define USE_TF02_PRO  // Sử dụng TF02 Pro

#include "TFT.h"

TFTPins pins{5,19,21,18,23,-1};  // CS=5, DC=19, RST=21, SCK=18, MOSI=23
TFTDistance display(pins);


#if defined(USE_TF02_PRO)
//---------Module TF02 Pro (UART) ---------
HardwareSerial& LRF = Serial2;
const int PIN_RX2 = 16; // TF02 TX -> RX2(16)
const int PIN_TX2 = 17; // TF02 RX -> TX2(17)
uint8_t buf9[9]; int st=0, idx=0;

bool readTF02(uint16_t &cm, uint16_t &strength, int16_t &temp_x8){
  while (LRF.available()){
    uint8_t b = LRF.read();
    switch (st){
      case 0: if (b==0x59){ buf9[0]=b; st=1; } break;
      case 1: if (b==0x59){ buf9[1]=b; idx=2; st=2; } else st=0; break;
      case 2:
        buf9[idx++] = b;
        if (idx>=9){
          uint8_t sum=0; for(int i=0;i<8;++i) sum+=buf9[i];
          bool ok = (sum==buf9[8]); st=0; idx=0;
          if (ok){
            cm       = (uint16_t)buf9[2] | ((uint16_t)buf9[3]<<8);
            strength = (uint16_t)buf9[4] | ((uint16_t)buf9[5]<<8);
            temp_x8  = (int16_t)((uint16_t)buf9[6] | ((uint16_t)buf9[7]<<8));
            return true;
          }
        }
        break;
    }
  }
  return false;
}
#endif

void setup() {
  Serial.begin(115200);
  display.begin();
  display.setMaxRangeMeters(30.0f);
  display.setSmoothing(0.25f);
  display.setStaleTimeoutMs(1000);

#if defined(USE_TF02_PRO)
  LRF.begin(115200, SERIAL_8N1, PIN_RX2, PIN_TX2);
#endif
}

void loop(){
#if defined(USE_TF02_PRO)
//------------Đọc TF02 Pro + hiển thị -------------
  uint16_t cm, strength; int16_t temp_x8;
  bool got = readTF02(cm, strength, temp_x8);
  const uint16_t MIN_STRENGTH = 30;          // lọc cơ bản
  bool valid = got && (strength >= MIN_STRENGTH) && (cm > 0);
  display.updateDistanceCm(cm, valid);
#else
//------------Generic test (không dùng TF02) -------------
  static float m = 0.0f, step = 0.6f;
  static uint32_t last = 0;
  if (millis() - last > 120) {
    last = millis();
    m += step;
    if (m >= 30.0f) { m = 30.0f; step = -step; }
    if (m <= 0.0f)  { m = 0.0f;  step = -step; }
    uint16_t cm = (uint16_t)(m * 100.0f);
    display.updateDistanceCm(cm, /*valid=*/true);
  }
#endif
}