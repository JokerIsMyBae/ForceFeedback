#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

//De connecties: zie bedradinsschema
  //stroommeter

  //H brug
#define motorEnA 2        //enable (digitaal)
#define motorIn1 3        // In 1 en 2 (digitaal), 1 HIGH en 1 LOW, omdraaien om van richting te veranderen
#define motorIn2 4        //de motor draait naar rechts bij IN1= LOW, IN2 = HIGH
int motorspeed = 0;       //overgenomen van thijs, gaat van 0 tot 255
  
  //encoder
#define encA  1           //pinA is pin 1 op Teensy
#define encB  0           //pinB is pin 0 op Teensy
bool encAstate = false;   //staat van pinA (0 of 1)
bool encBstate = false;   //staat van pinB (0 of 1)

//volatile int encA= 0;     // Pin A van encoder (digitaal)
//volatile int encB= 1;     // pin B van encoder (digitaal) 
//int encAstate=LOW;        //huidige staat van A
//int encAlastState =encAstate; // laatst gelezen staat van A 


  //extra variablen
static bool rotating = false; //voor debouncing
int rotCounter=9999;         //gaat omhoog naar links, omlaag naar rechts (uiterst rechts is dus 0, uiterst links = maxRot)
//int rotDirection=0;       // 0 voor kloksgewijs, 1 voor tegen de klok (wordt voorlopig niet gebruikt)
int maxRot=0;             //voorlopig 0, wordt overschreven na callibratie
int tempCounter=0;
boolean isCalibrated = false;
  //functies
void channelA() 
{
      
    // test of er effectief iets veranderd is
    if (digitalRead(encA) != encAstate) { // nog eens een debounce
        encAstate = !encAstate;

        if (encAstate != encBstate) { //als a niet gelijk aan b dan loopt a voor op b -> +
            rotCounter--;
        }
        else { //als a gelijk aan b loopt b voor op a -> -
            rotCounter++;
        }

        rotating = false; //niet meer debouncen tot loop opnieuw begint
    }
} 
void channelB() 
{
    if (digitalRead(encB) != encBstate) { 
        encBstate = !encBstate;

        if (encBstate != encAstate) { //als b niet gelijk aan a dan loopt b voor op a -> -
            rotCounter++;
        }
        else { //als b gelijk aan a dan loopt a voor op b -> +
            rotCounter--;          
        }
        
        rotating = false;
    }
}
boolean isTurning()       
{
  //stroommeting enzo?
  if (abs(rotCounter-tempCounter)>5)
  {
    
    Serial.printf("rotCounter= %d, tempCounter= %d dus ", rotCounter, tempCounter);
    Serial.println(" isTurning = true");
    tempCounter= rotCounter;
    delay(200); 
    return true;            //het stuur draait
  }else
  {
    Serial.printf("rotCounter= %d, tempCounter= %d dus ", rotCounter, tempCounter);
    Serial.println(" isTurning = false");
    tempCounter= rotCounter;
    delay(200); 
    return false;           //het stuur draait niet meer
  }
  
}
/*void updateEncoder()
{
  encAstate = digitalRead(encA);          //lees de staat van Pin A  
  if ((encAlastState == LOW) && (encAstate == HIGH)) // er is min 1 tick gedraaid
  {    
    if (digitalRead(encB) == HIGH)        //A= HIGH en B= HIGH is:  rotatie naar LINKS
    {       
      rotCounter--;                       //rotcounter gaat omhoog bij rotatie naar links 
    } else                                //A= HIGH en B= LOW is:  rotatie naar RECHTS
    {
      rotCounter++;                       //rotcounter gaat omhoog bij rotatie naar links 
    }    
  }  
  encAlastState = encAstate;              //update lastState met huidige staat
}*/

 void calibrate()
 {
    //calibratie
    Serial.println("Begin Calibratie");
    Serial.println("Draai naar rechts... ");
    digitalWrite(motorIn1, LOW);        //laat de motor naar rechts draaien tot het stuur niet meer beweegt
    digitalWrite(motorIn2, HIGH); 
    analogWrite(motorEnA, motorspeed);
    delay(20);
    while( isTurning() );               //zolang encoder zegt dat het stuur draait 
    {
      //delay(200);                        //om de 100 milliseconden checken
    }
    analogWrite(motorEnA, 0);           //stuur draait niet meer verder: stop de motor 
    Serial.println("Uiterst rechts bereikt, draai naar links... ");
    rotCounter=0;                       //uiterst rechts is het nulpunt

    //laat de motor naar links draaien tot het stuur niet meer beweegt en tel ondertussen de ticks van de encoder
    digitalWrite(motorIn1, HIGH);       //In1 en 2 worden omgedraaid HIGH <-> LOW
    digitalWrite(motorIn2, LOW);
    analogWrite(motorEnA, motorspeed); 
    delay(20);
    while( isTurning() );               //zolang encoder zegt dat het stuur draait 
    {
      //delay(200);                        //om de 100 milliseconden checken
    }
    analogWrite(motorEnA, 0);           //stop de motor
    maxRot=rotCounter;                  //uiterst links = max rotatia
    //nu de maxRotatie gekend is, weten we dat het midden bereikt is als rotCounter= maxRot/2
    Serial.printf("Uiterst links bereikt, maxRot = %d ",maxRot);
    Serial.println(" callibratie beindingd");
    isCalibrated = true;
 }
void setup() {
  Serial.begin(9600);//seriele monitor
  Serial.println("Begin declaratie");
  //declarenen van in/uitgangen
  //H brug
  pinMode(motorEnA, OUTPUT);
  pinMode(motorIn1, OUTPUT);           
  pinMode(motorIn2, OUTPUT);
  motorspeed = 220;
  //encoder
  pinMode(encA, INPUT); 
  pinMode(encB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA), channelA, CHANGE); //zet interrupt op pin 1
  attachInterrupt(digitalPinToInterrupt(encB), channelB, CHANGE); //zet interrupt op pin 0
  //pinMode(encA,INPUT);
  //pinMode(encB,INPUT);
  //als encA verandert (CHANGE), wordt de functie updateEncoder() uitgevoerd
  //attachInterrupt(digitalPinToInterrupt(encA),updateEncoder,CHANGE);

  
}

void loop() {
  
  if (!isCalibrated)
  {
    Serial.println("isCalibrated = false ");
    calibrate();
    Serial.println("gedaan met de pret");
  }
  
  }