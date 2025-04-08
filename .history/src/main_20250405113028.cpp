// Sender发送端
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("i am serial 0");

}

void loop() {
  mySerial2.println("i am serial 2ww");
  delay(1000);
}
