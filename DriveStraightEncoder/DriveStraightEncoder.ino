/**
   @file: DriveStraightEncoder.ino

   @description
   This program illustrates approximate straight-line driving for a specified distance.
   It adjusts the left and right wheel speeds using the relative encoder counts.
   The motors decelerate when the car reaches the desired distance, so the car
   overshoots the distance slightly. To prevent overshooting, you'll need to insert code
   that decelerates the car before it reaches the desired distance.

   @date
   10/30/2019

*/
#include <TimerOne.h>
#include <Servo.h> // use the servo library for the micro-servo

/******************** motor pins ********************/
// right-side (motor A side of L298N controller)
const int pwmRight = 5;     // ENA - Enable and PWM
const int rightReverse = 22; // IN1 - Reverse Drive
const int rightForward = 23; // IN2 - Forward Drive

// left-side (motor B side of L298N controller)
const int leftForward = 24;  // IN3 - Forward Drive
const int leftReverse = 25;  // IN4 - Reverse Drive
const int pwmLeft = 6;      // ENB - Enable and PWM
/****************************************************/

/************** motor/wheel variables ***************/
const float driveDistanceIN = 12 * 10; // drive distance in inches
const float driveDistanceMM = driveDistanceIN * 25.4; // drive distance in millimeters
const int initialSpeed = 100; // ranges from 0 to 255
const int maxSpeed = 255;
const int speedOffset = 4; // speed offset for driving straight
const int wheelDiam = 66.8; // wheel diameter in millimeters [2.63 in]
const int wheelDiamIN = 2.63;
const float wheelCircum = PI * wheelDiam; // wheel circumference in millimeters
const float wheelCircumIN = PI * wheelDiamIN;
/****************************************************/

/********** micro-servo pin and variables ***********/
const int microServoPin = 9;  // pin for micro-servo
Servo myservo;  // create servo object to control the micro-servo
int posCenter = 65; // micro-servo angles in degrees
const int TRIG_PIN = 26; // sensor trigger pin
const int ECHO_PIN = 27; // sensor echo pin
float minDistance = 15; //cm
float centerDistance = 0;
/****************************************************/

/******* interrupt pins and encoder variables *******/
const int leftWheelPin = 2;  // left wheel interrupt pin --- interrupt 0
const int rightWheelPin = 3;  // right wheel interrupt pin --- interrupt 1
volatile unsigned long leftCounter = 0; // left wheel encoder counter (must use a volatile variable)
volatile unsigned long rightCounter = 0; // right wheel encoder counter (must use a volatile variable)
float counter = 0;
volatile unsigned long leftPrevTime, rightPrevTime; // time variables for debouncing purposes
long debouncingTime = 10; // speed sensor signal debouncing time in milliseconds
float encoderSlots = 20.0;  // number of slots on wheel encoder disc
unsigned long timerDuration = 1000000;
float rotationTime = timerDuration / 1000000.0; // rotation time in seconds
unsigned long timer;
/****************************************************/

const int Light = 12;
float leftRPM;
float rightRPM;
int state;

void setup() {

  Serial.begin(115200); // set the baud rate

  // setup the micro-servo and orient the ultrasonic sensor in center position
  myservo.attach(microServoPin);
  myservo.write(posCenter);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // set the h-bridge motor driver pins
  pinMode(pwmLeft, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(pwmRight, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Light, OUTPUT);

  // setup interrupts
  Timer1.initialize(timerDuration); // set a timer that establishes when to compute the wheel rpms
  attachInterrupt(digitalPinToInterrupt (leftWheelPin), leftISR, FALLING);  // increase left wheel counter when pin goes Low
  attachInterrupt(digitalPinToInterrupt (rightWheelPin), rightISR, FALLING);  // increase right wheel counter when pin goes Low
  Timer1.attachInterrupt( ISR_timerone ); // enable the timer

  state = 0;
  timer = millis();
}

void loop() {
  driveStraight(driveDistanceMM, initialSpeed);
}


void driveStraight(float dist, int initialSpeedLocal) {
  // parameters: dist in millimeters
  //             speed range: 0 to 255

  centerDistance = getDistance(); 
  if (centerDistance <= minDistance) { // make sure the car doesn't bump into an obstacle
    allStop();
    flash();
  }
  
  unsigned long leftCounterCurrent;
  unsigned long rightCounterCurrent;

  // set initial motor speed
  int leftSpeed = initialSpeedLocal;
  int rightSpeed = initialSpeedLocal;

  // Used to determine which way to turn to adjust
  unsigned long leftCounterDifference;
  unsigned long rightCounterDifference;

  // Reset encoder counts
  leftCounter = 0;
  rightCounter = 0;

  // remember previous encoder counts
  unsigned long leftCounterPrev = leftCounter;
  unsigned long rightCounterPrev = rightCounter;

  // determine the target number of encoder counts
  float numOfRev = dist / wheelCircum; //
  unsigned long targetCount = numOfRev * encoderSlots;

  Serial.print("Driving for ");
  Serial.print(dist / 10);
  Serial.print(" cm [");
  Serial.print(driveDistanceIN);
  Serial.print(" in] (");
  Serial.print(targetCount);
  Serial.println(" encoder counts)");


  // drive until one of the encoders reaches target count
  while ( (leftCounter < targetCount) && (rightCounter < targetCount) ) {

    if(millis()-timer>1000){
      speedFlash(leftRPM, rightRPM);
    }
    Serial.println("HERE");
    wallCheck();
    // save current number of encoder counts
    leftCounterCurrent = leftCounter;
    rightCounterCurrent = rightCounter;

    // number of pulses counted since last time
    leftCounterDifference = leftCounterCurrent - leftCounterPrev;
    rightCounterDifference = rightCounterCurrent - rightCounterPrev;

    // Store current encoder count for next time
    leftCounterPrev = leftCounterCurrent;
    rightCounterPrev = rightCounterCurrent;

    // if left has faster value, slow it down and speed up right
    if ( leftCounterDifference > rightCounterDifference ) {
      leftSpeed -= speedOffset;
      rightSpeed += speedOffset;
    }

    // if right has faster value, slow it down and speed up left
    if ( leftCounterDifference < rightCounterDifference ) {
      leftSpeed += speedOffset;
      rightSpeed -= speedOffset;
    }

    // constrain speed to lie between -maxSpeed and maxSpeed
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

    driveWheelsIndependently(leftSpeed, rightSpeed);

    // brief pause to let motors respond
    delay(20);
  }

  // stop when done
  allStop();
}


void driveWheelsIndependently(int leftSpeedLocal, int rightSpeedLocal) {

  // left motor direction
  if ( leftSpeedLocal < 0 ) {
    digitalWrite(leftReverse, HIGH);
    digitalWrite(leftForward, LOW);
  } else {
    digitalWrite(leftReverse, LOW);
    digitalWrite(leftForward, HIGH);
  }

  // right motor direction
  if ( rightSpeedLocal < 0 ) {
    digitalWrite(rightReverse, HIGH);
    digitalWrite(rightForward, LOW);
  } else {
    digitalWrite(rightReverse, LOW);
    digitalWrite(rightForward, HIGH);
  }

  // set wheel speeds independently
  analogWrite(pwmLeft, abs(leftSpeedLocal));
  analogWrite(pwmRight, abs(rightSpeedLocal));
}

void allStop() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, 0);
  analogWrite(pwmRight, 0);
}

/********** interrupt service routines **************/
void leftISR() {

  if ((long)(millis() - leftPrevTime) >= debouncingTime) {
    leftCounter++;
    leftPrevTime = millis();
  }

}

void rightISR() {

  if ((long)(millis() - rightPrevTime) >= debouncingTime) {
    rightCounter++;
    rightPrevTime = millis();
  }

}

void ISR_timerone() {
  Timer1.detachInterrupt();  // stop timer to perform computations and print

  leftRPM = ((leftCounter / encoderSlots) / rotationTime) * 60.00; // left motor rpm computation
  rightRPM = ((rightCounter / encoderSlots) / rotationTime) * 60.00; // right motor rpm computation

  Serial.print("Left Wheel Speed: ");
  Serial.print(leftRPM);
  Serial.print(" RPM - ");

  Serial.print("Right Wheel Speed: ");
  Serial.print(rightRPM);
  Serial.println(" RPM");

  leftCounter = 0;  //  reset counter
  rightCounter = 0;  //  reset counter
  
  Timer1.attachInterrupt( ISR_timerone );  // enable timer
}
/****************************************************/
float getDistance() {
  long timeOfTravel;
  float distance;

  // set the TRIG_PIN low before you send the 10 microsecond pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // set the TRIG_PIN high for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // read the ECHO_PIN and return the time of travel in microseconds
  timeOfTravel = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance in centimeters
  distance = 0.0343 * timeOfTravel  / 2.0;

  // calculate the distance in inches
  //  distance = 0.0133 * timeOfTravel  / 2.0;

  return distance;
}

void flash(){
  for (int i=0; i<5; i++){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100); 
  }
}

void speedFlash(float left, float right){
  float avg = (left+right)/2; //RPM
  Serial.println(avg);
  int velocity = avg*wheelCircumIN/60;
  Serial.println(velocity);
  for (int i=0; i<velocity; i++){
    digitalWrite(Light, HIGH);   // turn the LED on (HIGH is the voltage level)
    wallCheck();
    delay(100);                       // wait for a second
    wallCheck();
    digitalWrite(Light, LOW);    // turn the LED off by making the voltage LOW
    wallCheck();
    delay(100);
    wallCheck();
  }
  timer = millis();
}

void wallCheck(){
  centerDistance = getDistance(); 
  if (centerDistance <= minDistance) { // make sure the car doesn't bump into an obstacle
    allStop();
    digitalWrite(Light, LOW);
    flash();
  } 
}
