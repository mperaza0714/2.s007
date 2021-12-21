/**
 * @file: ObstacleAvoidance.ino
 * 
 * @description
 * This program illustrates a simple obstacle avoidance strategy.
 * 
 */
 
#include <Servo.h> // use the servo library for the micro-servo

// motor pins
// right-side (motor A side of L298N controller)
const int pwmRight = 5;     // ENA - Enable and PWM
const int rightReverse = 22; // IN1 - Reverse Drive
const int rightForward = 23; // IN2 - Forward Drive

// use these pins if your car initially drives in reverse 
// also swap pins 24 and 25 as given below
//const int rightForward = 22; // IN1 - Forward Drive
//const int rightReverse = 23; // IN2 - Reverse Drive


// left-side (motor B side of L298N controller)
const int leftForward = 24;  // IN3 - Forward Drive
const int leftReverse = 25;  // IN4 - Reverse Drive

// use these pins if your car initially drives in reverse
// also swap pins 22 and 23 as given above
//const int leftReverse = 24;  // IN3 - Reverse Drive
//const int leftForward = 25;  // IN4 - Forward Drive

const int pwmLeft = 6;      // ENB - Enable and PWM

// ultrasonic sensor pins
const int TRIG_PIN = 26; // sensor trigger pin
const int ECHO_PIN = 27; // sensor echo pin

// micro-servo pin
const int SERVO_PIN = 9;  // pin for micro-servo

/****************** other variables/constants *******************/
const int minSpeed = 0; // minimum motor speed
const int maxSpeed = 255; // maximum motor speed
const int normalSpeed = 150; // normal driving speed
const int turnSpeed = 150; // turning speed
const unsigned long turnTime = 950; // turning time in milliseconds
const unsigned long brakeTime = 125; // braking time in milliseconds
const unsigned long reverseTime = 500; // reverse driving time in milliseconds
bool drivingForward = false; // driving forward boolean

Servo myservo;  // create servo object to control the micro-servo
int posLeft = 180, posCenter = 65, posRight = 0; // micro-servo angles in degrees

float leftDistance = 0, rightDistance = 0, centerDistance = 0; // sensor distances
float minDistance = 10; // minimum distance from obstacle in centimeters
//float minDistance = 8; // minimum distance from obstacle in inches

int state;
#define IDLE 0
#define PUSH 1
unsigned long timer;
float rate = .018;

void setup() {
  Serial.begin(9600); // set the serial port baud rate (for debugging, if necessary)

  // setup the micro-servo and orient the sensor in the center
  myservo.attach(SERVO_PIN);
  myservo.write(posCenter);

  // set the ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // set motor driver pins
  pinMode(pwmLeft, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(pwmRight, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
  state = 0;
  pinMode(LED_BUILTIN, OUTPUT);

  allStop();
}

void loop() {

  centerDistance = getDistance(); 
  Serial.println(",");
  Serial.println(centerDistance);
  paths(centerDistance);

}

void paths(float distance){
  // .018 in/millis
  switch(state){
    case IDLE:
      if (distance <= minDistance) { // make sure the car doesn't bump into an obstacle
        allStop();
        flash();
        state = 1;
        timer = millis();
      }else {
        allForward(normalSpeed);
        drivingForward = true;
      }
    break;
    case PUSH:
    Serial.println("PUSH");
      allForward(normalSpeed);
      if (millis()-timer > 2500){
        Serial.println("HERE");
        allStop();
        state = 3;
      }
    break;
  }
  
}

void flash(){
  for (int i=0; i<5; i++){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100); 
  }
}


/**************** Driving Functions *****************/
void allStop() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, 0);
  analogWrite(pwmRight, 0);
}

void allForward(int driveSpeed) {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, driveSpeed);
  analogWrite(pwmRight, driveSpeed);
}

void allReverse(int driveSpeed) {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftReverse, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightReverse, HIGH);
  analogWrite(pwmLeft, driveSpeed);
  analogWrite(pwmRight, driveSpeed);
}

void turnLeft(int driveSpeed) {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftReverse, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, driveSpeed);
  analogWrite(pwmRight, driveSpeed);
}

void turnRight(int driveSpeed) {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightReverse, HIGH);
  analogWrite(pwmLeft, driveSpeed);
  analogWrite(pwmRight, driveSpeed);
}

void brake(int brakeSpeed) {
  allReverse(brakeSpeed); // reverse the motors to prevent coasting
  delay(brakeTime);
  allStop();
}
/****************************************************/

/*********** distance calculation function **********/
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
/****************************************************/
