////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Servo Library
#include <Servo.h>
Servo myservo;

//Motor A Pins
const int enableA = 3;  // The On-Board Arduino 
const int pinA1 = 7;   // The On-Board Arduino 
const int pinA2 = 6;  // The On-Board Arduino

//Motor B Pins
const int enableB = 8;    // The On-Board Arduino 
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
//angle defenition
const int defualtAngle = 90;
const int startAngle = 160;
const int finalAngle = 30;
//numeric const
const int T1 = 50;
const int T2 = 250;
const int T3 = 500;
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
   myservo.write(defualtAngle);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){
  //----- ---- --- Main Code --- ---- -----\\
  //Use analogWrite To Run Motor At Adjusted Speed
  analogWrite(enableA, ENASpeed);
  analogWrite(enableB, ENBSpeed);

  carMoveForward();
  delay(T1);
  carAvoidObstacle();
  delay(T1);   
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
void turnRight(int duration) {
 motorABackward();
 motorBForward();
 delay(duration);
}
void turnLeft(int duration) {
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
void carMoveForward(){
int distanceCalculated;
int FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
int FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor
int CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor
int CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor 
distanceCalculated = calculateDistance(); //calculating the Distance 1 time 
delay(T1);

  while( (distanceCalculated > 20 ) && (FR == HIGH) && (FL == HIGH) && (CR == HIGH) && (CL == HIGH) )
  {   
     enableMotors(); //turn ON motor A and B 
     
     moveForward(T2); // car moves forward 
   
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
  bool flag = true;  
  while(flag){      
    flag = searchNewRoute();      
  }   
  myservo.write(defualtAngle); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// - -- ---Car searches New Route to go-------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////////
bool searchNewRoute(){
  bool obstacle = true;  
  
  if(sonicSearchNewRoute()){ // Searching a new route using UltraSonic sensor
    obstacle = false;    
  }
  else{
    irSearchNewRoute();// Searching a new route using IR sensors 
    obstacle = false;   
  }
  myservo.write(defualtAngle);
  
  return obstacle;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// - -- ---Avoiding obstacle usin UltraSonic sensor ------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////
bool sonicSearchNewRoute(){
  bool newroute = false;  
  int newAngle = startAngle;
  int currentAngle = newAngle;
  myservo.write(newAngle); 
  int newDistance =calculateDistance();  
  
  //while the distance < 20 and the servo's angle > finalAngle :  ===>>> 
  // -> the newAngle value would be decremented and servo would be adjusted to that angle,
  //on a new servo's angle, UltraSonic would measure a new distance   
  while((newDistance < 20) && (newAngle > finalAngle)){
    newAngle = newAngle - 40;
    currentAngle = newAngle;
    myservo.write(newAngle);
    newDistance = calculateDistance();    
  }
  if((newDistance > 20) && (newAngle >= finalAngle)){
    newroute = true;
    if(currentAngle > defualtAngle){
      moveBackwardLeft();      
    }
    else{
      moveBackwardRight();      
    }    
  }
  return newroute;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// - -- ---Avoiding obstacle usin IR sensors ------------------------------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void irSearchNewRoute(){  
  int FR = digitalRead(FRONT_RIGHT);// read FRONT_LEFT sensor
  int FL = digitalRead(FRONT_LEFT);// read FRONT_RIGHT sensor
  int CR = digitalRead(CENTER_RIGHT);// read CENTER_RIGHT sensor
  int CL = digitalRead(CENTER_LEFT);// read CENTER_LEFT sensor 
 
 if((FR == LOW || CR == LOW) && (FL == HIGH && CL == HIGH)){
  moveBackwardRight();
 }
 else if((FL == LOW || CL == LOW) && (FR == HIGH && CR == HIGH)){
  moveBackwardLeft();
 }
 else{
  goReverse();
  moveBackwardRight();
  turnRight();  
  moveBackwardRight();
  turnRight();  
  moveBackwardRight();
 }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// - -- --- car moves backward and turn Left/turn Right functions :1----------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward and turn Left -------------------------
void moveBackwardLeft(){
  digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
  turnLeft(T3);
  moveBackward(T2);    
  digitalWrite(LED13, LOW);            //Sets The  LED Duty Ratio OFF      
}
//- -- --- car moves backward and turn Right ----------------------
void moveBackwardRight(){
  digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
  turnRight(T3);
  moveBackward(T2);          
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// - -- --- car moves backward and turn Left/Right functions :2----------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward and turn Left ---------------------------------------------------------------
void goReverseLeft(){
  digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
  turnLeft(30);
  moveBackward(T2);
  myservo.write(defualtAngle);
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward and turn Right -----------------------
void goReverseRight(){
  digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
  turnRight(30);
  moveBackward(T2);
  myservo.write(defualtAngle);
  digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//- -- --- car moves backward ----------------------------------------------------------------------------
void goReverse(){
 digitalWrite(LED13, HIGH);            //Sets The  LED Duty Ratio ON
 digitalWrite(LED1, HIGH);            //Sets The  LED Duty Ratio ON
 moveBackward(T3);
 myservo.write(defualtAngle);
 digitalWrite(LED13, LOW);            //Sets The  LED Duty Ratio OFF
 digitalWrite(LED1, LOW);            //Sets The  LED Duty Ratio OFF
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////
