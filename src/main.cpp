#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define EnA 2
#define In1 3
#define In2 4

Adafruit_INA219 ina219;
/////////////////////////////////////check polarity ina!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
int motorspeed = 0;

void setup() {
  pinMode(EnA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  analogWriteFrequency(EnA,22000);
  motorspeed = 250;
  //////////////////////////////////////////////////////check polarity ina!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Serial.begin(9600);
  while (!Serial){
    delay(1);
  }
  if(!ina219.begin()){
    Serial.println("failed to find in219");
     while (1) { delay(10); }
  }
 // ina219.setCalibration_32V_1A();
}
  //////////////////////////////////////////////////////check polarity ina!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void loop() {
  float current_mA = 0;
  float shuntvoltage = 0;
  float busvoltage = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  // turn 1 way // rechts
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(EnA, motorspeed); // turn motor 1 way
  delay(500);
  current_mA = ina219.getCurrent_mA();
  Serial.print("current_mA = ");Serial.print(current_mA);
  delay(500);
    //////////////////////////////////////////////////////check polarity ina!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  current_mA = ina219.getCurrent_mA();
  // extra info, wss niet nodig -> testen waarom dit vorige keer niet werkte
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
// einde van extra testcode

  Serial.print("current_mA = ");Serial.print(current_mA);
  analogWrite(EnA, 0); // turn of motor
  // turn other way // links
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
    //////////////////////////////////////////////////////check polarity ina!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  analogWrite(EnA, motorspeed); // turn motor other way
  delay(1000);
  analogWrite(EnA, 0); // turn of motor 
}