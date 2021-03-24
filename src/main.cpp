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
  Serial.begin(9600);
  ina219.begin();
}

void loop() {
  float current = 0;

  // turn 1 way // rechts
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnA, motorspeed); // turn motor 1 way
  delay(500);
  current = ina219.getCurrent_mA();
  Serial.println(current);
  delay(500);
  analogWrite(EnA, 0); // turn of motor
  // turn other way // links
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(EnA, motorspeed); // turn motor other way
  delay(1000);
  analogWrite(EnA, 0); // turn of motor 
}