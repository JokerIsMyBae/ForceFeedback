#include <Arduino.h> //dit moet wss weg

//De connecties: zie bedradinsschema
  //stroommeter

  //H brug
#define motorEnA 2        //enable (digitaal)
#define motorIn1 3        // In 1 en 2 (digitaal), 1 hoog en 1 laag, omdraaien om van richting te veranderen
#define motorIn2 4
int motorspeed = 0;       //overgenomen van thijs
  //encoder
volatile int encA= 0;     // Pin A van encoder (digitaal)
volatile int encB= 1;     // pin B van encoder (digitaal)
int encAstate=LOW;        //huidige staat van A
int encAlastState =encAstate; // laatst gelezen staat van A 

  //extra variablen
int rotCounter=0;         //gaat omhoog naar links, omlaag naar rechts (uiterst rechts is dus 0, uiterst links = maxRot)
int rotDirection=0;       // 0 voor kloksgewijs, 1 voor tegen de klok
int maxRot=0;             //voorlopig 0, wordt overschreven na callibratie

  //functies
boolean isTurning()       //nog te schrijven
{
  //stroommeting enzo
  return true;            //de motor draait
}
void updateEncoder()
{
  encAstate = digitalRead(encA);          //lees de staat van Pin A  
  if ((encAlastState == LOW) && (encAstate == HIGH)) // er is min 1 tick gedraaid
  {    
    if (digitalRead(encB) == HIGH)        //A= HIGH en B= HIGH is:  rotatie naar LINKS
    {       
      rotCounter++;                       //rotcounter gaat omhoog bij rotatie naar links 
    } else                                //A= HIGH en B= LOW is:  rotatie naar RECHTS
    {
      rotCounter--;                       //rotcounter gaat omhoog bij rotatie naar links 
    }    
  }  
  encAlastState = encAstate;              //update lastState met huidige staat
}


void setup() {
  Serial.begin(9600);//seriele monitor

  //declarenen van in/uitgangen
  //H brug
  pinMode(motorEnA, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  motorspeed = 150;
  //encoder
  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
  //als encA verandert (CHANGE), wordt de functie updateEncoder() uitgevoerd
  attachInterrupt(digitalPinToInterrupt(encA),updateEncoder,CHANGE);

  //callibratie
  Serial.printf("Begin Callibratie /n");
  Serial.printf("Draai naar rechts... /n");
  digitalWrite(motorIn1, LOW);        //laat de motor naar rechts draaien tot het stuur niet meer beweegt
  digitalWrite(motorIn2, HIGH); 
  analogWrite(motorEnA, motorspeed);
  while( isTurning() );               //zolang encoder zegt dat het stuur draait 
  {
    delay(10);                        //om de 10 milliseconden checken
  }
  analogWrite(motorEnA, 0);           //stuur draait niet meer verder: stop de motor 
  Serial.printf("Uiterst rechts bereikt, draai naar links... /n");
  rotCounter=0;                       //uiterst rechts is het nulpunt

  //laat de motor naar links draaien tot het stuur niet meer beweegt en tel ondertussen de ticks van de encoder
  digitalWrite(motorIn1, HIGH);       //In1 en 2 worden omgedraaid HIGH <-> LOW
  digitalWrite(motorIn2, LOW);
  analogWrite(motorEnA, motorspeed); 
  while( isTurning() );               //zolang encoder zegt dat het stuur draait 
  {
    delay(10);                        //om de 10 milliseconden checken
  }
  analogWrite(motorEnA, 0);           //stop de motor
  maxRot=rotCounter;                  //uiterst links = max rotatia
  //nu de maxRotatie gekend is, weten we dat het midden bereikt is als rotCounter= maxRot/2
  Serial.printf("Uiterst links bereikt, maxRot = %d /n",maxRot);
}

void loop() {
  }