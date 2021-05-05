#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Encoder.h>
#include <StensTimer.h>

#define TICK_ACTION 1

StensTimer* stensTimer = NULL;
int counter = 0;

//PID contstanten
double kp=1;
double ki=1;
double kd=1;

//variabelen

double angle = 0;
//PID:
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double current;             //input
double motorSpeed;          //=voltage = output (0 tot 255)
double setPoint;              //
double sumError, rateError; //integral en derivative


double calculateForce(float angle, double scalar)
{
  double force= angle*scalar;//de kracht is evenredig met de hoekafwijking en wordt groter/kleiner met de snelheid vd auto (scalar)
  return force;
}

double calculatePID(double input)
{
  currentTime=millis();
  elapsedTime=(double)(currentTime-previousTime);

  error = setPoint-input;
  sumError += error*elapsedTime;
  rateError = (error - lastError)/elapsedTime;

  double output= kp*error + ki*sumError + kd*rateError;

  lastError= error;
  previousTime=currentTime;

  return output;
}

void timerCallback(Timer* timer)
{
/* This function is called when a timer has passed */
  setPoint=calculateForce(angle,1);
  motorSpeed=calculatePID(setPoint);
}

void printMenu()
{
  Serial.printf("Menu: typ het getal van de optie die u wilt uitvoeren");
  Serial.printf("1: parameters invoeren \n 2: normale procedure");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setPoint=0;
  /* Save instance of StensTimer to the tensTimer variable*/
stensTimer = StensTimer::getInstance();
/* Tell StensTimer which callback function to use */
stensTimer->setStaticCallback(timerCallback);
Timer* LoopTimer = stensTimer->setInterval(TICK_ACTION, 1000);//1 sec
}

void loop() {
  // put your main code here, to run repeatedly:
  //current=analogRead() //lees stroom
  //angle=...
  /* let StensTimer do it's magic every time loop() is executed */
  printMenu();
  stensTimer->run();
  
}