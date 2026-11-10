#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BNO08x.h>

#define RFM_CS 5
#define RFM_INT 4
#define RFM_RST 14


RH_RF95 rf95(RFM_CS, RFM_INT);
Adafruit_BNO08x bno(-1);
sh2_SensorValue_t val;


struct __attribute__((packed)) Packet { uint8_t seq; float w, x, y, z; };
Packet pkt = {0, 1, 0, 0, 0};
uint32_t lastSend = 0;

void enableReports() {
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 10 ms
}

void setup() {
  Serial.begin(115200);
  pinMode(RFM_RST, OUTPUT);
  digitalWrite(RFM_RST, LOW);  delay(10);
  digitalWrite(RFM_RST, HIGH); delay(10);

  if (!rf95.init()) { Serial.println("RFM9X fail"); while (1); }
  rf95.setFrequency(915.0);
  rf95.setTxPower(13, false);
  rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);

  if (!bno.begin_I2C()) { Serial.println("BNO085 fail"); while (1); }
  enableReports();
}

void loop() {
  if (bno.wasReset()) enableReports();

  if (bno.getSensorEvent(&val) && val.sensorId == SH2_GAME_ROTATION_VECTOR) {
    pkt.w = val.un.gameRotationVector.real;
    pkt.x = val.un.gameRotationVector.i;
    pkt.y = val.un.gameRotationVector.j;
    pkt.z = val.un.gameRotationVector.k;
  }

  if (millis() - lastSend >= 30) {   // ~33 Hz
    lastSend = millis();
    pkt.seq++;
    rf95.send((uint8_t*)&pkt, sizeof(pkt));
    rf95.waitPacketSent();
  }
}
