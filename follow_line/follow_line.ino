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
int lastseen;


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
  int spd = 150;

  while (centerColor>50) {
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
    
    allForward(spd);
    
    if ((centerColor<50) && (rightColor<50) && (leftColor<50)) {
      turnRight(spd);
      delay(300);
      centerColor = analogRead(CS);
      if (centerColor>50) {
        return;
      }
      turnLeft(spd);
      delay(600);
      centerColor = analogRead(CS);
      if (centerColor>50) {
        return;
      }
      allStop();
      return;
    }else if ((centerColor>50) && (rightColor>50) && (leftColor>50) && (rightMostColor>50) && (leftMostColor>50)) {
      Serial.println("WHITE");
      if (lastseen == 0){
        Serial.println("left");
        turnLeft(spd);
      }else if(lastseen == 1){
        Serial.println("right");
        turnRight(spd);
      }else if(lastseen ==2){
        Serial.println("left");
        turnLeft(spd);
      }else if(lastseen ==3){
        Serial.println("right");
        turnRight(spd);
      }else{
        continue;
      }
    }else if (rightColor<50) {
      Serial.println("LEFT");
      lastseen = 0;
      turnLeft(spd);
    }else if (leftColor<50) {
      Serial.println("RIGHT");
      lastseen = 1;
      turnRight(spd);
    }else if (rightMostColor<50) {
      Serial.println("LEFT");
      lastseen = 2;
      turnLeft(spd);
    }else if (leftMostColor<50) {
      Serial.println("RIGHT");
      lastseen = 3;
      turnRight(spd);
    }
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
  analogWrite(pwmRight, driveSpeed-20);
}

void turnRight(int driveSpeed) {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightReverse, LOW);
  analogWrite(pwmLeft, driveSpeed-20);
  analogWrite(pwmRight, driveSpeed);
}

void brake(int brakeSpeed) {
  allReverse(brakeSpeed); // reverse the motors to prevent coasting
  delay(brakeTime);
  allStop();
}
