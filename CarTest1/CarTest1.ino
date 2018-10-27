//Servo Library
#include <Servo.h>
Servo myservo;

//Motor A Pins
const int enableA = 3;  // The On-Board Arduino 
const int pinA1 = 7;   // The On-Board Arduino 
const int pinA2 = 6;  // The On-Board Arduino

//Motor B Pins
const int EnableB = 8;    // The On-Board Arduino 
const int pinB1 = 5;     // The On-Board Arduino 
const int pinB2 = 4;    // The On-Board Arduino 

//Servo Pins
const int servposnum = 0;   // The On-Board Arduino Servo
const int servpos = 0;     // The On-Board Arduino Servo

//Led Pins
const int LED13 = 13;           // The On-Board Arduino LED - @Digitalpin 13
const int LED1 = 1;            // The On-Board Arduino LED - @Digitalpin 1
//Buzzer Pins
const int buzzer = A0;           //Declares Buzzer @Analogpin A0

//UltraSonic Sensor Pins
#define trigPin A4
#define echoPin A3
#define TURN_TIME 175
//Speed Of The Motors
#define ENASpeed 250 
#define ENBSpeed 250
// 4 Infrared Obstacle Pins
#define FRONT_LEFT 10 // pin 2 for front-left sensor
#define FRONT_RIGHT 9 // pin 3 for front-right sensor
#define CENTER_LEFT 12 // pin 4 for center-left sensor
#define CENTER_RIGHT 11 // pin 5 for center-right sensor

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

   //Configure Pin Modes For The Drive Motors
   //PinMode(enableA, OUTPUT);
   pinMode(pinA1, OUTPUT);
   pinMode(pinA2, OUTPUT);

   //PinMode(EnableB, OUTPUT);
   pinMode(pinB1, OUTPUT);
   pinMode(pinB2, OUTPUT); 

   //Configure Pin Modes For The Ultrasonic Sensor
   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);

   //Configure Pin Modes For The Leds
   pinMode(LED13, OUTPUT);    //Sets the  LED as the output
   pinMode(LED1, OUTPUT);    //Sets the  LED as the output

   //Configure Pin Modes For The Buzzer
   pinMode(buzzer,OUTPUT);       //Sets the buzzer as the output
  //Configure 4 Infrared Sensors Pins 
  pinMode(FRONT_LEFT, INPUT);//define front-left input pin
  pinMode(FRONT_RIGHT, INPUT);//define front-right input pin 
  pinMode(CENTER_LEFT, INPUT);//define center-left input pin
  pinMode(CENTER_RIGHT, INPUT);//define center-right input pin 
   
   //Servo Pins
   myservo.attach(2);
   myservo.write(90);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){
  //----- ---- --- Main Code --- ---- -----\\

  //Use analogWrite To Run Motor At Adjusted Speed
  analogWrite(enableA, ENASpeed);
  analogWrite(EnableB, ENBSpeed);

/*  int FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
  int FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor
  int RR = digitalRead(REAR_RIGHT);// read CENTER_RIGHT sensor
  int RL = digitalRead(REAR_LEFT);// read CENTER_LEFT sensor  
*/
  car();
  avoid();
     
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Motor Functions --- -- -
void motorA_Forward() {
 digitalWrite(pinA1, HIGH);
 digitalWrite(pinA2, LOW);
}
void motorB_Forward() {
 digitalWrite(pinB1, LOW);
 digitalWrite(pinB2, HIGH);
}
void motorA_Backward() {
 digitalWrite(pinA1, LOW);
 digitalWrite(pinA2, HIGH);
}
void motorB_Backward() {
 digitalWrite(pinB1, HIGH);
 digitalWrite(pinB2, LOW);
}
void motorA_Stop() {
 digitalWrite(pinA1, HIGH);
 digitalWrite(pinA2, HIGH);
}
void motorB_Stop() {
 digitalWrite(pinB1, HIGH);
 digitalWrite(pinB2, HIGH);
}
void motorA_Coast() {
 digitalWrite(pinA1, LOW);
 digitalWrite(pinA2, LOW);
}
void motorB_Coast() {
 digitalWrite(pinB1, LOW);
 digitalWrite(pinB2, LOW);
}
void motorA_On() {
 digitalWrite(enableA, HIGH);
}
void motorB_On() {
 digitalWrite(EnableB, HIGH);
}
void motorA_Off() {
 digitalWrite(enableA, LOW);
}
void motorB_Off() {
 digitalWrite(EnableB, LOW);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Movement Functions --- -- -
void forward(int duration) {
 motorA_Forward();
 motorB_Forward();
 delay(duration);
}
void backward(int duration) {
 motorA_Backward();
 motorB_Backward();
 delay(duration);
}
void right(int duration) {
 motorA_Backward();
 motorB_Forward();
 delay(duration);
}
void left(int duration) {
 motorA_Forward();
 motorB_Backward();
 delay(duration);
}
void coast(int duration) {
 motorA_Coast();
 motorB_Coast();
 delay(duration);
}
void breakRobot(int duration) {
 motorA_Stop();
 motorB_Stop();
 delay(duration);
}
void disableMotors() {
 motorA_Off();
 motorB_Off();
}
void enableMotors() {
 motorA_On();
 motorB_On();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Recieving The Distance From The UltraSonic Sensor --- -- -
int distance(){
  int duration, distance;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*34300*0.000001)/2 ;
  return distance;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Car Driving Forward --- -- -
void car(){
int distance_0;
int FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
int FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor
int CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor
int CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor 
distance_0 = distance();
delay(50);

  while( (distance_0 > 20 ) && (FR == HIGH) && (FL == HIGH) && (CR == HIGH) && (CL == HIGH) )
  {   
     motorA_On();
     motorB_On();
     forward(250);    
     distance_0 = distance();     
     
     FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
     FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor 
     CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor
     CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor 
  }  
  breakRobot(0); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Car Avoiding Obstacle = Search New Route --- -- -
void avoid(){

  int newdistance = distance();
  bool flag = true;
  bool canturn = false;
  int newangle;
  int count = 1;  
  
  while(flag){      
    flag = findNewRoute();      
  } 
  myservo.write(90); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool findNewRoute(){
  bool flag = true;
  int newDistance =distance() ;
  int newAngle = 160;
  bool newroute = true;
  int FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
  int FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor
  int CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor
  int CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor 
  
  while(flag){
   if(newDistance < 20){    
    if(newAngle > 30){
      myservo.write(newAngle);                 //command to rotate the servo to the specified angle
      delay(50);
      newDistance = distance();
      if(newDistance > 20){
        flag = false;        
        if( (newAngle > 90) ){
          turnLeft();         
        }else{
          turnRight();       
        } 
      }
    }else{
      myservo.write(90);
      flag = false;
      reverse();
      turnRight(); 
      turnRight(); 
      turnRight(); 
    }
    newAngle =newAngle - 40; 
    newroute = false;
    
  }else{
    myservo.write(90);
    if(FR == LOW || CR == LOW){
      flag = false;
      turnRight();
    }
    else if(FL == LOW || CL == LOW){
      flag = false;
      turnLeft();
    }
    else if(CR == LOW || CL == LOW){
      flag = false;
      reverse();
      turnRight();
      delay(50);
      turnRight();
      delay(50);
      turnRight();
    }
    
    FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
    FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor
    CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor
    CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor 
    newroute = false;
  }
 }  
  return newroute; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turnLeft(){
  digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
  left(500);
  backward(500);    
  digitalWrite(LED13, LOW);            //Sets The  LED Duty Ratio OFF      
}

void turnRight(){
  digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
  right(500);
  backward(500);          
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- 
void reverseLeft(){
  digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
  left(30);
  backward(250);
  myservo.write(90);
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- 
void reverseRight(){
  digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
  right(30);
  backward(250);
  myservo.write(90);
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- 
void reverse(){
 digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
 digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
 backward(500);
 myservo.write(90);
 digitalWrite(LED13, LOW);            //Sets The  LED Duty Ratio OFF
 digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- 
void TurnBack(){
  motorA_Backward();
  motorB_Forward();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
