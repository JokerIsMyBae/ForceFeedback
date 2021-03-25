#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define EnA 2
#define In1 3
#define In2 4

Adafruit_INA219 ina219;

int motorspeed = 0;

void setup() {
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  analogWriteFrequency(EnA,22000);
  motorspeed = 250;
  Serial.begin(115200);
  while (!Serial){
    delay(1);
  }
  if(!ina219.begin()){
    Serial.println("failed to find in219");
    delay(10);
  }
 // ina219.setCalibration_32V_1A();
}

void loop() {
  float current_mA = 0;

  // turn 1 way // rechts
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnA, motorspeed); // turn motor 1 way
  delay(500);
  current_mA = ina219.getCurrent_mA();
  Serial.print("current_mA = ");Serial.print(current_mA);
  delay(500);
  current_mA = ina219.getCurrent_mA();
  Serial.print("current_mA = ");Serial.print(current_mA);
  analogWrite(EnA, 0); // turn of motor
  // turn other way // links
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(EnA, motorspeed); // turn motor other way
  delay(1000);
  analogWrite(EnA, 0); // turn of motor 
}