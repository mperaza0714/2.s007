const int pwmRight = 5;     // ENA - Enable and PWM
const int rightReverse = 22; // IN1 - Reverse Drive
const int rightForward = 23; // IN2 - Forward Drive

const int pwmLeft = 6;
const int leftForward = 24;  // IN3 - Forward Drive
const int leftReverse = 25;  // IN4 - Reverse Drive

const int minSpeed = 0; // minimum motor speed
const int maxSpeed = 255; // maximum motor speed
const int normalSpeed = 150; // normal driving speed
const int turnSpeed = 150; // turning speed
const unsigned long turnTime = 950; // turning time in milliseconds
const unsigned long brakeTime = 125; // braking time in milliseconds
const unsigned long reverseTime = 500; // reverse driving time in milliseconds
bool drivingForward = false; // driving forward boolean

const int LLS = A1; //Left Most Sensor
const int LS = A2; // Left Senros
const int CS = A3; //Center Line Sensor
const int RS = A4; //Right Line Sensor
const int RRS = A5; //Right Most Line Sensor
int state;
int spd=200;


void setup() {
  Serial.begin(9600);
  
  pinMode(pwmLeft, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(pwmRight, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);

  pinMode(LLS, INPUT);
  pinMode(LS, INPUT);
  pinMode(CS, INPUT);
  pinMode(RS, INPUT);
  pinMode(RRS, INPUT);

  state = 0;

}

void loop() {

  int leftMostColor = analogRead(LLS);
  int leftColor = analogRead(LS);
  int rightColor = analogRead(RS);
  int centerColor = analogRead(CS);
  int rightMostColor = analogRead(RRS);
  leftColor = map(leftColor, 950, 986, 0, 100);
  leftMostColor = map(leftMostColor, 760, 982, 0, 100);
  centerColor = map(centerColor, 860, 982, 0, 100);
  rightColor = map(rightColor,800, 984, 0, 100);
  rightMostColor = map(rightMostColor,840, 982, 0, 100);
  lineSense(leftColor, leftMostColor, rightColor, rightMostColor, centerColor);

}

void lineSense(int LC, int LMC, int RC, int RMC, int CC){
  switch(state){
    case 0:
      allForward(spd);
      if (RC<50){
        state = 1;
      }
      if (LC<50){
        state = 2;
      }
    break;
    case 1: //rightsensor
      turnLeft(spd);
      if (CC <50){
        state = 0;
      }
      if (RMC<50){
        state = 3;
      }
    break;
    case 2: //left sensor
      turnRight(spd);
      if (CC <50){
        state = 0;
      }
      if (LMC<50){
        state = 4;
      }
    break;
    case 3: //right most sensor
      turnSharpLeft(spd);
      if (RC<50){
        state = 1;
      }
      if (CC<50){
        state = 0;
      }
    break;
    case 4: //left most sensor
      turnSharpRight(spd);
      if (LC<50){
        state = 2;
      }
      if (CC<50){
        state = 0;
      }
    break;
  }
}


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
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, driveSpeed);
  analogWrite(pwmRight, 0);
}

void turnRight(int driveSpeed) {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, 0);
  analogWrite(pwmRight, driveSpeed);
}

void turnSharpLeft(int driveSpeed) {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftReverse, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, driveSpeed);
  analogWrite(pwmRight, driveSpeed);
}

void turnSharpRight(int driveSpeed) {
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
