//Servo Library
#include <Servo.h>
Servo myservo;

//Motor A Pins
const int enableA = 3;  // The On-Board Arduino 
const int pinA1 = 7;   // The On-Board Arduino 
const int pinA2 = 6;  // The On-Board Arduino

//Motor B Pins
const int enableA = 8;    // The On-Board Arduino 
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
//
const int startAngle = 160;
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

   //PinMode(enableA, OUTPUT);
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
  analogWrite(enableA, ENBSpeed);

  carMoveForwardMveForward();
  carAvoidObstacle();
     
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Motor Functions --- -- -
// set motor A to forward movement
void motorAForward() {
 digitalWrite(pinA1, HIGH);
 digitalWrite(pinA2, LOW);
}
// set motor B to forward movement
void motorBForward() {
 digitalWrite(pinB1, LOW);
 digitalWrite(pinB2, HIGH);
}
// set motor A to backward movement
void motorABackward() {
 digitalWrite(pinA1, LOW);
 digitalWrite(pinA2, HIGH);
}
//set motor A to backward movement
void motorBBackward() {
 digitalWrite(pinB1, HIGH);
 digitalWrite(pinB2, LOW);
}
//set motor A to Stop 
void motorAStop() {
 digitalWrite(pinA1, HIGH);
 digitalWrite(pinA2, HIGH);
}
//set motor B to Stop 
void motorBStop() {
 digitalWrite(pinB1, HIGH);
 digitalWrite(pinB2, HIGH);
}
//set motor A to ON 
void motorAOn() {
 digitalWrite(enableA, HIGH);
}
//set motor B to ON  
void motorBOn() {
 digitalWrite(enableA, HIGH);
}
//set motor A OFF  
void motorAOff() {
 digitalWrite(enableA, LOW);
}
//set motor B OFF 
void motorBOff() {
 digitalWrite(enableA, LOW);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Movement Functions --- -- -
void moveForward(int duration) {
 motorAForward();
 motorBForward();
 delay(duration);
}
void moveBackward(int duration) {
 motorABackward();
 motorBBackward();
 delay(duration);
}
void right(int duration) {
 motorABackward();
 motorBForward();
 delay(duration);
}
void left(int duration) {
 motorAForward();
 motorBBackward();
 delay(duration);
}

void stopMovement(int duration) {
 motorAStop();
 motorBStop();
 delay(duration);
}
void disableMotors() {
 motorAOff();
 motorBOff();
}
void enableMotors() {
 motorAOn();
 motorBOn();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Recieving The Calculated Distance From The UltraSonic Sensor --- -- -
int calculateDistance(){
  int duration, distance;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*34300*0.000001)/2 ;
  return distance;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Car Moving Forward --- -- -
void carMoveForwardMveForward(){
int distanceCalculated;
int FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
int FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor
int CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor
int CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor 
distanceCalculated = calculateDistance(); //calculating the Distance 1 time 
delay(50);

  while( (distanceCalculated > 20 ) && (FR == HIGH) && (FL == HIGH) && (CR == HIGH) && (CL == HIGH) )
  {   
     enableMotors(); //turn ON motor A and B 
     
     moveForward(250); // car moves forward 
   
     distanceCalculated = calculateDistance(); // calculating the distance again    
     
     FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor again 
     FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor again
     CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor again
     CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor again
  }  
  stopMovement(0); // stop the car 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- Car Avoiding Obstacle = Search New Route --- -- -
void carAvoidObstacle(){

  int newdistance = calculateDistance();
  bool flag = true;
  bool canturn = false; 

  while(flag){      
    flag = searchNewRoute();      
  } 
  myservo.write(90); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool searchNewRoute(){
  bool flag = true;
  int newDistance =calculateDistance() ;
  int newAngle = startAngle;
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
      newDistance = calculateDistance();
      if(newDistance > 20){
        flag = false;        
        if( (newAngle > 90) ){
          moveBackwardLeft();         
        }else{
          moveBackwardRight();       
        } 
      }
    }else{
      myservo.write(90);
      flag = false;
      goReverse();
      moveBackwardRight(); 
      moveBackwardRight(); 
      moveBackwardRight(); 
    }
    newAngle =newAngle - 40; 
    newroute = false;
    
  }else{
    myservo.write(90);
    if(FR == LOW || CR == LOW){
      flag = false;
      moveBackwardRight();
    }
    else if(FL == LOW || CL == LOW){
      flag = false;
      moveBackwardLeft();
    }
    else if(CR == LOW || CL == LOW){
      flag = false;
      goReverse();
      moveBackwardRight();
      delay(50);
      moveBackwardRight();
      delay(50);
      moveBackwardRight();
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
// - -- --- car moves backward and turn Left/Right functions :1----------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward and turn Left -------------------------
void moveBackwardLeft(){
  digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
  left(500);
  moveBackward(500);    
  digitalWrite(LED13, LOW);            //Sets The  LED Duty Ratio OFF      
}
//- -- --- car moves backward and turn Right ----------------------
void moveBackwardRight(){
  digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
  right(500);
  moveBackward(500);          
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// - -- --- car moves backward and turn Left/Right functions :2----------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward and turn Left ---------------------------------------------------------------
void goReverseLeft(){
  digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
  left(30);
  moveBackward(250);
  myservo.write(90);
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward and turn Right -----------------------
void goReverseRight(){
  digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
  right(30);
  moveBackward(250);
  myservo.write(90);
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward ----------------------------------------------------------------------------
void goReverse(){
 digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
 digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
 moveBackward(500);
 myservo.write(90);
 digitalWrite(LED13, LOW);            //Sets The  LED Duty Ratio OFF
 digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////
