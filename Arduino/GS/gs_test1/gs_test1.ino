#include <SPI.h>
#include <RH_RF95.h>

#define RFM_CS 5
#define RFM_INT 4
#define RFM_RST 14

RH_RF95 rf95(RFM_CS, RFM_INT);
struct __attribute__((packed)) Packet { uint8_t seq; float w, x, y, z; };

void setup() {
  Serial.begin(115200);
  pinMode(RFM_RST, OUTPUT);
  digitalWrite(RFM_RST, LOW);  delay(10);
  digitalWrite(RFM_RST, HIGH); delay(10);

  if (!rf95.init()) { while (1); }
  rf95.setFrequency(915.0);
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
}

void loop() {
  if (rf95.available()) {
    Packet p;
    uint8_t len = sizeof(p);
    if (rf95.recv((uint8_t*)&p, &len) && len == sizeof(p)) {
      Serial.printf("%.4f,%.4f,%.4f,%.4f\n", p.w, p.x, p.y, p.z);
    }
  }
}