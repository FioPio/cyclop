//LIBRARIES
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#define COUNTSREVOLUTION 1632.67

#define BATTERYPIN   A0     //Battery pin
#define BATTERYPERIOD  250
float batteryVoltage = 0;

#define LMAIN         3     //Left Motor Input A
#define LMBIN         4     //Left Motor Input B
#define LMSPD         5     //Left Motor PWM

#define RMAIN         6     //Right Motor Input A
#define RMBIN         7     //Right Motor Input B
#define RMSPD         8     //Right Motor PWM

//sets the default rotation directions
#define LFORWARDHIGH    false
#define RFORWARDHIGH    true

//Pins for interrupt
#define ENCODRA      22     //Encoder Right A
#define ENCODRB      23     //Encoder Right B
#define ENCODLA      24     //Encoder Left  A
#define ENCODLB      25     //Encoder Left  B

//Counter for the encoder
int countR = 0;
int countL = 0;

// PI constants
#define P     0.21//0.2//163.31
#define I     0.4329//0.1//520
#define PIPERIOD 100

// Variables for PI
double spdL, spdR;
double elapsedT;
double outPIL, outPIR;
double error, errorIL, errorIR;
double out;

//times for the "parrallel tasks"
unsigned long int lastTBattery = 0;
unsigned long int lastTPI = 0;

// ROS
ros::NodeHandle nh;

void doStuffL(const std_msgs::Int32& msg){
  setLMS(msg.data);
}
void doStuffR(const std_msgs::Int32& msg){
  setRMS(msg.data);
}
ros::Subscriber<std_msgs::Int32> subL("inLM", &doStuffL);
ros::Subscriber<std_msgs::Int32> subR("inRM", &doStuffR);


void setup() {
  //Serial
  Serial.begin(9600);
  //Subscribe 
  nh.subscribe(subR);
  nh.subscribe(subL);
  //Sets the motor outputs
  pinMode(LMAIN,  OUTPUT);
  pinMode(LMBIN,  OUTPUT);
  pinMode(LMSPD,  OUTPUT);
  pinMode(RMAIN,  OUTPUT);
  pinMode(RMBIN,  OUTPUT);
  pinMode(RMSPD,  OUTPUT);

  digitalWrite(LMAIN,  LOW);
  digitalWrite(LMBIN,  LOW);
  digitalWrite(LMSPD,  LOW);
  digitalWrite(RMAIN,  LOW);
  digitalWrite(RMBIN,  LOW);
  digitalWrite(RMSPD,  LOW);

  //Defines the interrpts
  attachInterrupt(digitalPinToInterrupt(ENCODRA), encoRCount, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODLA), encoLCount, CHANGE);

  lastTBattery = millis();
}

void loop() {
  // Check Batteryvoltage
  if (millis() - lastTBattery > BATTERYPERIOD){
    lastTBattery = millis();
    batteryVoltage = getBatteryLevel();  
  }
  
  // Set reference speed - SET BY ROS INPUT
  spdL = 100;  // Temporary
  spdR = spdL;  // Temporary
  
  //PI in regular intervals
  elapsedT = millis() - lastTPI;
  if (elapsedT > PIPERIOD){
    lastTPI = millis();
    
    outPIL = getPIL(spdL, countL);
    outPIR = getPIR(spdR, countR);

    setLMS(outPIL);
    setRMS(outPIR);
    
    countL = 0;
    countR = 0;
  }
}

// PI
double getPIL(double ref, double count){
  error = ref - (-count);
  errorIL += error * PIPERIOD/1000.0;
  out = P*error + I*errorIL; 
  if((out > 255)||(out < -255)){
    errorIL -= error * PIPERIOD/1000.0;
  }
  
  return out;
}

double getPIR(double ref, double count){
  error = ref - count;
  errorIR += error * PIPERIOD/1000.0;
  out = P*error + I*errorIR;
  if((out > 255)||(out < -255)){
    errorIR -= error * PIPERIOD/1000.0;
  }
  return out;
}

//Returns the battery voltage in Volts
float getBatteryLevel()
{
  return analogRead(BATTERYPIN)*(10.0/1024);
}


//Sets the PWM signal of the Left Motor
void setLMS(int _spd)
{
  //Setting the PWM limits
  if( _spd >255 )
  {
     _spd = 255;
  }
  if( _spd < -255 )
  {
     _spd = -255;
  }
  //Setting the speed and the direction
  if(_spd==0)
  {
    digitalWrite(LMAIN, LOW);
    digitalWrite(LMBIN, LOW);
  }
  else if(_spd>0)    //forward
  {
    digitalWrite(LMAIN, LFORWARDHIGH);
    digitalWrite(LMBIN, !LFORWARDHIGH);
  }
  else  //backward
  {
    digitalWrite(LMAIN, !LFORWARDHIGH);
    digitalWrite(LMBIN, LFORWARDHIGH);
    _spd=-_spd;
  }
  analogWrite(LMSPD, _spd);
}

//Sets the PWM signal of the Right Motor
void setRMS(int _spd)
{
  //Setting the PWM limits
  if( _spd >255 )
  {
     _spd = 255;
  }
  if( _spd < -255 )
  {
     _spd = -255;
  }
  //Setting the speed and the direction
  if(_spd==0)
  {
    digitalWrite(RMAIN, LOW);
    digitalWrite(RMBIN, LOW);
  }
  else if(_spd>0)    //forward
  {
    digitalWrite(RMAIN, RFORWARDHIGH);
    digitalWrite(RMBIN, !RFORWARDHIGH);
  }
  else  //backward
  {
    digitalWrite(RMAIN, !RFORWARDHIGH);
    digitalWrite(RMBIN, RFORWARDHIGH);
    _spd=-_spd;
  }
  analogWrite(RMSPD, _spd);
}


void encoRCount()
{
  if(digitalRead(ENCODRA))
  {
    if(digitalRead(ENCODRB)) countR--;
    else countR++;
  }
}

void encoLCount()
{
  if(digitalRead(ENCODLA))
  {
    if(digitalRead(ENCODLB)) countL--;
    else countL++;
  }
}
