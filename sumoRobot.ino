// MOTORS
#include <AFMotor.h>
AF_DCMotor leftMotor(4, MOTOR12_64KHZ);
AF_DCMotor rightMotor(3, MOTOR12_64KHZ);

// ULTRASONIC SENSOR
const int triggerPin = A1;

// IR SENSOR
const int distancePin = A0;

// IR SENSORS
const int frontLeftIRPin = A3; // 0 = Blanco, 1 Negro
const int frontRightIRPin = A2;
const int backLeftIRPin = A5;
const int backRightIRPin = A4;

int frontLeftIRValue = 1;
int frontRightIRValue = 1;
int backLeftIRValue = 1;
int backRightIRValue = 1;

// CONSTANTS
const int maxMotorsSpeed = 255;

const int borderValue = 0; // 0 el borde es blanco
const bool moveMotors = true;
float distances[10];

bool isTheBorderAtRight = 0;

int timesRight = 0;
int timesLeft = 0;

// SETUP
void setup() {
  Serial.begin(9600); Serial.println("============= SETUP =============");

  pinMode(triggerPin, OUTPUT);

  pinMode(frontLeftIRValue, INPUT);
  pinMode(frontRightIRValue, INPUT);
  pinMode(backLeftIRValue, INPUT);
  pinMode(backRightIRValue, INPUT);

  for (int i = 0; i < 10; i++) {
    float distance = 4800 / ( analogRead(distancePin) - 20 );

    if (distance > 80) {
      distance = 85;
    }

    else if (distance < 10) {
      distance = 9;
    };

    distances[i] = distance;
  }

  Serial.println("============= FINISH SETUP =============");

  delay(5000);
}

// LOOP
void loop() {
  float distance = getDistance(); // Serial.println(distance);

  // activateTrigger();

  frontLeftIRValue = digitalRead(frontLeftIRPin);
  frontRightIRValue = digitalRead(frontRightIRPin);
  backLeftIRValue = digitalRead(backLeftIRPin);
  backRightIRValue = digitalRead(backRightIRPin);

  if ( frontLeftIRValue == borderValue || frontRightIRValue == borderValue || backLeftIRValue == borderValue || backRightIRValue == borderValue ) {
    defense();
  } else {
    timesRight = 0;
    timesLeft = 0;

    attack(distance);
  }

  delay(10);
}

// HELPERS
void attack(float distance) {
  if ( distance > 80 || distance < 0 ) {
    if ( !moveMotors ) { return; }

    if ( isTheBorderAtRight ) {
      leftMotor.run(BACKWARD);
      rightMotor.run(FORWARD);

      leftMotor.setSpeed(maxMotorsSpeed * 0.8);
      rightMotor.setSpeed(maxMotorsSpeed * 0.8);
    } else {
      leftMotor.run(FORWARD);
      rightMotor.run(BACKWARD);

      leftMotor.setSpeed(maxMotorsSpeed * 0.8);
      rightMotor.setSpeed(maxMotorsSpeed * 0.8);
    }

  } else {
    Serial.println("Move front attack");

    if ( !moveMotors ) { return; }
  
    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);
  }
}

void defense() {
  if ( frontLeftIRValue == borderValue && frontRightIRValue == borderValue ) {
    // Serial.println("Move back");
    if ( !moveMotors ) { return; }

    leftMotor.run(BACKWARD);
    rightMotor.run(BACKWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = false;

    timesRight = 0;
    timesLeft = 0;
  
  } else if ( frontLeftIRValue == borderValue && backLeftIRValue == borderValue ) {
    // Serial.println("Move right");
    if ( !moveMotors ) { return; }

    leftMotor.run(FORWARD);
    rightMotor.run(BACKWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = false;

    timesRight = 0;
    timesLeft = 0;

  } else if ( frontRightIRValue == borderValue && backRightIRValue == borderValue ) {
    // Serial.println("Move left");
    if ( !moveMotors ) { return; }

    leftMotor.run(BACKWARD);
    rightMotor.run(FORWARD);

    rightMotor.setSpeed(maxMotorsSpeed);
    leftMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = true;

    timesRight = 0;
    timesLeft = 0;

  } else if ( backLeftIRValue == borderValue && backRightIRValue == borderValue ) {
    // Serial.println("Move front");
    if ( !moveMotors ) { return; }

    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = false;

    timesRight = 0;
    timesLeft = 0;

  } else if ( frontLeftIRValue == borderValue ) {
    // Serial.println("Move FrontLeft");
    if ( !moveMotors ) { return; }

    leftMotor.run(FORWARD);
    rightMotor.run(BACKWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = false;

    timesLeft += 1;
    timesRight = 0;

  } else if ( frontRightIRValue == borderValue ) {
    // Serial.println("Move BackLeft");
    if ( !moveMotors ) { return; }
  
    leftMotor.run(BACKWARD);
    rightMotor.run(FORWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = true;

    timesRight += 1;
    timesLeft = 0;

  } else if ( backLeftIRValue == borderValue) {
    // Serial.println("Move FrontRight");
    if ( !moveMotors ) { return; }

    leftMotor.run(FORWARD);
    rightMotor.run(BACKWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = false;

    timesRight = 0;
    timesLeft = 0;

  } else if ( backRightIRValue == borderValue) {
    // Serial.println("Move BackRight");
    if ( !moveMotors ) { return; }

    leftMotor.run(BACKWARD);
    rightMotor.run(FORWARD);

    leftMotor.setSpeed(maxMotorsSpeed);
    rightMotor.setSpeed(maxMotorsSpeed);

    isTheBorderAtRight = true;

    timesRight = 0;
    timesLeft = 0;
  }

  if ( timesLeft >= 3 || timesRight >= 3 ) {
    delay(500);
  }

}

float getDistance() {
  float distance = 4800 / ( analogRead(distancePin) - 20 );

  if (distance > 80 || distance < 0) {
    distance = 90;
  }

  else if (distance < 10) {
    distance = 8;
  };

  for (int i = 1; i < 11; i++) {
    distances[i - 1] = distances[i];
  }

  distances[9] = distance;

  float sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += distances[i];
  }

  float avg = sum / 10;

  return avg;
}

float activateTrigger() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
}
