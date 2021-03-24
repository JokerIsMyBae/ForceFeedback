#include <Arduino.h>

/*
 Pins:
 Yellow = GND
 Green = Vcc (5v or 3.3V)
 Black = Channel 1/A
 White = Channel 2/B
*/

enum PinAssignments {
    encoderPinA = 1,
    encoderPinB = 0
};

volatile int encoderPos = 0; // teller voor postitie van het stuur
static bool rotating = false; //voor debouncing

bool A_set = false;  //staat van pinA (0 of 1)
bool B_set = false;  //staat van pinB (0 of 1)

//functies uitgevoerd bij interrupts
void channelA();
void channelB();

void setup() 
{
    Serial.begin(9600);

    pinMode(encoderPinA, INPUT); 
    pinMode(encoderPinB, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPinA), channelA, CHANGE); //zet interrupt op pin 1
    attachInterrupt(digitalPinToInterrupt(encoderPinB), channelB, CHANGE); //zet interrupt op pin 0
}

void loop() {
    rotating = true; //reset voor debouncer
}

void channelA() 
{
    if (rotating)
        delay (1); //wacht tot bounce gedaan is
    
    // test of er effectief iets veranderd is
    if (digitalRead(encoderPinA) != A_set) { // nog eens een debounce
        A_set = !A_set;

        if (A_set != B_set) { //als a niet gelijk aan b dan loopt a voor op b -> +
            encoderPos++;
        }
        else { //als a gelijk aan b loopt b voor op a -> -
            encoderPos--;
        }

        rotating = false; //niet meer debouncen tot loop opnieuw begint
    }
} 
//zelfde commentaar als channelA()
void channelB() 
{
    if (rotating)
        delay (1);

    if (digitalRead(encoderPinB) != B_set) { 
        B_set = !B_set;

        if (B_set != A_set) { //als b niet gelijk aan a dan loopt b voor op a -> -
            encoderPos--;
        }
        else { //als b gelijk aan a dan loopt a voor op b -> +
            encoderPos++;          
        }
        
        rotating = false;
    }
}