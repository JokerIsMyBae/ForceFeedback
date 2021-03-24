#include <Arduino.h>
#include <iostream>

/*
 Pins:
 Yellow = GND
 Green = Vcc (5v or 3.3V)
 Black = Channel 1
 White = Channel 2
*/

enum PinAssignments {
    encoderPinA = 1,
    encoderPinB = 0
};

volatile int encoderPos = 0;
int lastEncoderPos = 1;
static bool rotating = false;

bool A_set = false;
bool B_set = false;

void channelA();
void channelB();

void setup() {
    Serial.begin(9600);

    pinMode (encoderPinA, INPUT);
    pinMode (encoderPinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPinA), channelA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), channelB, CHANGE);

}

void loop() {
    rotating = true;
}

void channelA() {
    if (rotating)
        delay (1);
    
    if (digitalRead(encoderPinA) != A_set) {
        A_set = !A_set;

        if (A_set != B_set) {
            encoderPos++;
            Serial.println("rechts");
        }
        else {
            Serial.println("links");
            encoderPos--;
        }

        rotating = false;
    }

} 

void channelB() {
    if (rotating)
        delay (1);

    if (digitalRead(encoderPinB) != B_set) {
        B_set = !B_set;

        if (B_set != A_set) {
            Serial.println("links");
            encoderPos--;
        }
        else {
            Serial.println("rechts");
            encoderPos++;          
        }
        
        rotating = false;
    }
}