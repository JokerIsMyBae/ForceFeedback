#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Encoder.h>

//declareren van functies die elkaar oproepen
void menu();
void readInput();
void updateMotor();

//De connecties: zie bedradinsschema
  //stroommeter ina219
float current_mA = 0;
/*float shuntvoltage = 0;
float busvoltage = 0;
float loadvoltage = 0;
float power_mW = 0;*/
Adafruit_INA219 ina219;     //ina object, om stroom te meten: ina219.getCurrent_mw

  //H brug
#define motorEnA 2          //enable (digitaal)
#define motorIn1 3          // In 1 en 2 (digitaal), 1 HIGH en 1 LOW, omdraaien om van richting te veranderen
#define motorIn2 4          //de motor draait naar rechts bij IN1= LOW, IN2 = HIGH
int motorspeed = 0;         //gaat van 0 tot 255
  
  //encoder --> dit is een hardwareteller die de rotatie bijhoudt t.o.v. de positie die hij had bij activatie
Encoder rotEncoder(0, 1);   //de encoder telt op naar rechts 

//de timer

IntervalTimer myTimer;      //IntervalTimer object 

//extra variablen
double angle = 0;           //t.o.v. het middelpunt (waar rotCounter = 450)
double scalar=1;            // een constante om de kracht te tunen
bool highLow=true;          //om de polariteit van de motor te bepalen (slechte naam maar het werkt)
bool testModus = false; 
//PID:
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double current;             //input
double setPoint=0; 
double sumError, rateError; //integral en derivative (derivative wordt voorlopig niet gebruikt maar kan in theorie wel)
//PID contstanten
double kp=1;
double ki=0;                //0.45*kp volgens ziegler-nicholas methode
double kd=0;                //0.8*kp/tp

//calibratie
int rotCount=9999;          //gaat omhoog naar links, omlaag naar rechts (uiterst rechts is dus 0, uiterst links = maxRot)
int maxRot=0;               //voorlopig 0, wordt overschreven na callibratie
int prevRotCount=0;
int rotOffset=0;            //de positie waarin de encoder de 1ste keer wordt gelezen zal hij als pos 0 beschouwen, is niet gelijk aan startpunt
boolean isCalibrated = false;

//functies
void updateEncoder()        //update de rotCount, houdt rekening met beginpositie van encoder-decoder
{
  rotCount=-1*(rotEncoder.read()-rotOffset); 
  angle= (-1)*(float(rotCount)/maxRot * 900)+450; //rare bereking, het werkt
}

boolean isTurning()   //gebruikt bij calibratie, werkt met encoder, niet met stroom    
{
  updateEncoder();          //update rotCount
  if (abs(rotCount-prevRotCount)>5)//threshold van 5 ticks
  {
    Serial.printf("rotCount= %d, prevRotCount= %d dus ", rotCount, prevRotCount);
    Serial.println(" isTurning = true");
    prevRotCount= rotCount;
    delay(200); 
    return true;            //het stuur draait
  }else
  {
    Serial.printf("rotCount= %d, prevRotCount= %d dus ", rotCount, prevRotCount);
    Serial.println(" isTurning = false");
    prevRotCount= rotCount;
    delay(200); 
    return false;           //het stuur draait niet meer
  }  
}

void calibrate()
{
  //calibratie
  motorspeed=150;                     //trage motorspeed tegen het terugbotsen aan uiteinden
  Serial.println("Begin Calibratie");
  Serial.println("Draai naar rechts... ");
  highLow=false;
  updateMotor();
  /*digitalWrite(motorIn1, LOW);        //laat de motor naar rechts draaien tot het stuur niet meer beweegt
  digitalWrite(motorIn2, HIGH); 
  analogWrite(motorEnA, motorspeed);*/
  delay(100);
  while( isTurning() );               //zolang encoder zegt dat het stuur draait 
  { }                                  //isTurning wordt alleen hier gebruikt, roept zelf een delay op
  analogWrite(motorEnA, 0);           //stuur draait niet meer verder: stop de motor
  Serial.println("Uiterst rechts bereikt, stop de motor ");
  updateEncoder();                    //offset is hier nog 0, de functie zal dus alleen het verschil met de beginpositie geven
  Serial.printf( "De encoder geeft nu als rotatie %d = de rotOffset",rotCount);
  Serial.println("");                 //een enter want \n werkt precies niet
  rotOffset=rotEncoder.read(); 
  updateEncoder();                    //nu de offset is ingesteld zou het correct (0) moeten zijn
  Serial.printf("Na instellen van offset is onze rotCounter= %d (zou 0 moeten zijn) ",rotCount);
  Serial.println("");                 // een enter want \n werkt precies niet
  Serial.println("Draai naar links...");  
   //laat de motor naar links draaien tot het stuur niet meer beweegt en tel ondertussen de ticks van de encoder
  highLow=true;
  updateMotor();
  /*digitalWrite(motorIn1, HIGH);       //In1 en 2 worden omgedraaid HIGH <-> LOW
  digitalWrite(motorIn2, LOW);
  analogWrite(motorEnA, motorspeed); */
  delay(100);
  while( isTurning() );               //zolang encoder zegt dat het stuur draait 
  {  }
  analogWrite(motorEnA, 0);           //stop de motor
  maxRot=rotCount;                    //uiterst links = max rotatie
  //nu de maxRotatie gekend is, weten we dat het midden bereikt is als rotCount= maxRot/2
  Serial.printf("Uiterst links bereikt, maxRot = %d ",maxRot);
  Serial.println(" callibratie beeindigd");
  isCalibrated = true;
}

void testRotCount()   //om de graden manueel te testen
{
  int i;
  Serial.println("Test");
  for (i=0; i < 4; i++)
  {
    Serial.println("zet het stuur manueel in een positie.");
    delay(3000);
    rotCount=-1*(rotEncoder.read()-rotOffset);
    float absAngle = float(rotCount)/maxRot * 900;
    Serial.printf("absolute hoek van uiterst rechts is= %f en hoek t.o.v. het midden is %f",absAngle, angle);
    delay(500);
  }
}

void updateMotor()
{
    analogWrite(motorEnA, 0);//tijdelijk op 0 zetten

  if (highLow) //motor draait naar links (tegen de klok)
  {
    digitalWrite(motorIn1, HIGH);       
    digitalWrite(motorIn2, LOW);
  }
  else//de motor draait naar rechts (met de klok)
  {
    digitalWrite(motorIn1, LOW);       
    digitalWrite(motorIn2, HIGH);
  }
  analogWrite(motorEnA, motorspeed);
}

double calcSetPoint(float angle) //berekent kracht voor huidige hoekafwijking
{
  double force= angle*scalar;                     //de kracht is evenredig met de hoekafwijking en wordt groter/kleiner met de snelheid vd auto (scalar)
  Serial.printf("angle= %f",angle);
  return force;
}

double calcPID(double input)//input is stroom, output is motorspeed=voltage
{
  double current255;
  currentTime=millis(); 
  elapsedTime=(double)(currentTime-previousTime);
  current255=(input/750)*255;                     //max current is 750 mA en max motorspeed is 255
  error = setPoint-current255;
  sumError += error*elapsedTime;

  int maxSumError=50;
  if(sumError > maxSumError)
  {
    sumError=maxSumError;
  }else if (sumError<(-1)*maxSumError)
  {
    sumError=(-1)*maxSumError;
  }

  rateError = (error - lastError)/elapsedTime;    //wordt hier niet gebruikt aangezien kd=0

  double output= kp*error + ki*sumError + kd*rateError;

  lastError= error;
  previousTime=currentTime;

  if (output>255)                                  //max motorspeed = 255
  {
    output=255;
  }else if(output<0)                               //motorspeed is positief, IN1,IN2 van hbrug bepalen polen van motor
  {                                                //als output negatief is betekent dit dat de motor naar links wil draaien
    highLow=false;    
    output=abs(output);
  }else
  {
    highLow=true;                                  //motor draait naar rechts
  }
  return output;
}

void timerCallback()
{
  updateEncoder();
  if (!testModus)//normale modus
  {
      setPoint=calcSetPoint(angle);               //de hoek wordt geupdate, hoe groter de hoek hoe groter de kracht
  }
  current_mA = ina219.getCurrent_mA();
  motorspeed=calcPID(current_mA);
  if (testModus)
  {
      Serial.printf(" current = %f \n hoek= %f \n motorspeed= %d \n",current_mA,angle,motorspeed);
  }
  updateMotor();
}

void readInput()
{
  testModus=false;
  int keuze=0;
  double temp=0;
  while (Serial.available() == 0) { }             // Wait for User to Input Data   
  keuze = Serial.parseInt();                      //Read the data the user has input (1 charachter)
  
  switch (keuze)  
  {
    case 1:
        Serial.println("U koos: 1");
        Serial.println("Geef de nieuwe krachtscalair in (tussen 0 en 5): ");
        while (Serial.available() == 0) { }       // Wait for User to Input Data
        temp = Serial.parseInt();                 //Read the data the user has input
        if (temp >=0 and temp<6)                  //min en max voor scalair
        {
          scalar=temp;                            //terug naar menu
          Serial.printf("de nieuwe scalar is: %f \n",scalar);
          delay(500);
          menu();
        }
        else{                                     //ongeldige input
          Serial.println("De kracht scalair moet tussen 0 en 5 zijn");
          delay(500);
          menu();                                 //terug naar menu
        }
        break;

    case 2://centreren
        Serial.println("U koos: 2");
        myTimer.begin(timerCallback, 150000);     //in µ-sec
        break;

    case 3:
        Serial.println("U koos: 3");
        testRotCount();
        menu();
        break;

    case 4:
        Serial.println("Optie 4: ");
        testModus=true;
        setPoint=150;
        myTimer.begin(timerCallback, 150000); //in µ-sec
        break;

    default:
        Serial.println("Gelieve een geldig getal in te geven");
        delay(500);
        menu();
  }
}

void menu()//extra feature
{
  Serial.printf("Menu: typ het getal van de optie die u wilt uitvoeren \n");
  Serial.println("1: scalair voor kracht invoeren");
  Serial.println("2: automatisch centreren");
  Serial.println("3: calibratie testen door stuur manueel te draaien");
  Serial.println("4: testmodus met vaste setpoint=150");  
  readInput();
}

void setup() {
  Serial.begin(9600);//seriele monitor
  while (!Serial){
    delay(1);
  }
  Serial.println("Begin Setup");
  if(!ina219.begin()){
    Serial.println("failed to find in219");
     while (1) { delay(10); }
  }  
  //declarenen van in/uitgangen
  //H brug
  pinMode(motorEnA, OUTPUT);
  pinMode(motorIn1, OUTPUT);           
  pinMode(motorIn2, OUTPUT);
  motorspeed = 0;
  analogWriteFrequency(motorEnA,22000);    //freq aangepast om gepiep te vermijden   
}

void loop() {
  if (!isCalibrated)
  {
    Serial.println("isCalibrated = false ");
    calibrate();
    if (isCalibrated)
    {
    Serial.println("isCalibrated = true, gedaan met de pret");
    menu();    
    }
  }
  //nadat menu wordt gecalled zal het ofwel optie 2 of 4 blijven loopen, ofwel het menu

}