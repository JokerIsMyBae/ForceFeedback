#include <Arduino.h>
#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(0, 1);

void setup() {
  Serial.begin(9600);
}

long position  = 999;

void loop() {
  long newLeft;
  newLeft = myEnc.read();
  if (newLeft != position) {
    Serial.print("Position = ");
    Serial.print(newLeft);
    Serial.println();
    position = newLeft;
  }
}