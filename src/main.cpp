#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define EnA 2
#define In1 3
#define In2 4

int motorspeed = 0;

void setup() {
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  analogWriteFrequency(EnA,22000);
  motorspeed = 100;
}

void loop() {

  // turn 1 way // rechts
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnA, motorspeed); // turn motor 1 way
  delay(1000);
  analogWrite(EnA, 0); // turn of motor
  // turn other way // links
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(EnA, motorspeed); // turn motor other way
  delay(1000);
  analogWrite(EnA, 0); // turn of motor 
}