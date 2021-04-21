#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Encoder.h>

//De connecties: zie bedradinsschema
  //stroommeter

  //H brug
#define motorEnA 2          //enable (digitaal)
#define motorIn1 3          // In 1 en 2 (digitaal), 1 HIGH en 1 LOW, omdraaien om van richting te veranderen
#define motorIn2 4          //de motor draait naar rechts bij IN1= LOW, IN2 = HIGH
int motorspeed = 0;         //gaat van 0 tot 255
  
  //encoder --> dit is een hardwareteller die de rotatie bijhoudt t.o.v. de positie die hij had bij activatie
Encoder rotEncoder(0, 1);   //de encoder telt op naar rechts 

//#define encA  1           //pinA is pin 1 op Teensy
//#define encB  0           //pinB is pin 0 op Teensy
//bool encAstate = false;   //staat van pinA (0 of 1)
//bool encBstate = false;   //staat van pinB (0 of 1)

//volatile int encA= 0;     // Pin A van encoder (digitaal)
//volatile int encB= 1;     // pin B van encoder (digitaal) 
//int encAstate=LOW;        //huidige staat van A
//int encAlastState =encAstate; // laatst gelezen staat van A 


  //extra variablen
//static bool rotating = false; //voor debouncing
int rotCount=9999;          //gaat omhoog naar links, omlaag naar rechts (uiterst rechts is dus 0, uiterst links = maxRot)
int maxRot=0;               //voorlopig 0, wordt overschreven na callibratie
int tempRotCount=0;
int rotOffset=0;            //de positie waarin de encoder de 1ste keer wordt gelezen zal hij als pos 0 beschouwen, dit is niet gelijk aan ons startpunt
boolean isCalibrated = false;

//functies
void updateEncoder()
{
  rotCount=-1*(rotEncoder.read()-rotOffset);
}


boolean isTurning()       
{
  updateEncoder();        //update rotCount
  if (abs(rotCount-tempRotCount)>5)
  {
    Serial.printf("rotCount= %d, tempRotCount= %d dus ", rotCount, tempRotCount);
    Serial.println(" isTurning = true");
    tempRotCount= rotCount;
    delay(200); 
    return true;            //het stuur draait
  }else
  {
    Serial.printf("rotCount= %d, tempRotCount= %d dus ", rotCount, tempRotCount);
    Serial.println(" isTurning = false");
    tempRotCount= rotCount;
    delay(200); 
    return false;           //het stuur draait niet meer
  }  
}

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
  Serial.println("Uiterst rechts bereikt, stop de motor ");
  updateEncoder();
  Serial.printf( "De encoder geeft nu als rotatie %d = de rotOffset",rotCount);
  Serial.println("");// een enter want \n werkt precies niet
  rotOffset=rotEncoder.read(); 
  updateEncoder();
  Serial.printf("Na instellen van offset is onze rotCounter= %d (zou 0 moeten zijn) ",rotCount);
  Serial.println("");// een enter want \n werkt precies niet
  Serial.println("Draai naar links...");  
 // rotCount=0;                       //uiterst rechts is het nulpunt
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
  maxRot=rotCount;                  //uiterst links = max rotatia
  //nu de maxRotatie gekend is, weten we dat het midden bereikt is als rotCount= maxRot/2
  Serial.printf("Uiterst links bereikt, maxRot = %d ",maxRot);
  Serial.println(" callibratie beeindigd");
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
  analogWriteFrequency(motorEnA,22000);
  //encoder
  /*pinMode(encA, INPUT); 
  pinMode(encB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA), channelA, CHANGE); //zet interrupt op pin 1
  attachInterrupt(digitalPinToInterrupt(encB), channelB, CHANGE); //zet interrupt op pin 0
  */
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

  /*int i=0;
  Serial.println("Test");
  for (i < 4)
  {
    Serial.println("zet het stuur manueel in een positie.");
    delay(5000);
    Serial.println("De hoek is %d graden",(rotCount/maxRot)*450);
    delay(2000);
    i++;
  }*/
  int i;
  Serial.println("Test");
  for (i=0; i < 4; i++)
  {
    Serial.println("zet het stuur manueel in een positie.");
    delay(5000);
    rotCount=-1*(rotEncoder.read()-rotOffset);
    float hoek = float(rotCount)/maxRot * 900;
    Serial.println(hoek);
    delay(2000);
  }
  
  }