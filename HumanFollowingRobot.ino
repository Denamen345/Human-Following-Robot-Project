#include <Servo.h>
#include <AFMotor.h>

// Define sensor pins
#define LEFT_SENSOR A0      // Left IR sensor
#define ECHO_PIN A1         // Ultrasonic sensor echo pin
#define TRIG_PIN A2         // Ultrasonic sensor trigger pin
#define RIGHT_SENSOR A3     // Right IR sensor

// Initialize motor objects with descriptive names
AF_DCMotor LeftFrontMotor(1, MOTOR12_1KHZ);
AF_DCMotor LeftBackMotor(2, MOTOR12_1KHZ);
AF_DCMotor RightFrontMotor(3, MOTOR34_1KHZ);
AF_DCMotor RightBackMotor(4, MOTOR34_1KHZ);

// Initialize the servo motor
Servo ultrasonicServo;

// Variables
int servoPosition = 0;  // Servo motor position
long ultrasonicTime;    // Time for ultrasonic pulse

// Function prototypes for organization
unsigned int measureDistance();
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMotors();

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Attach the servo motor
  ultrasonicServo.attach(10);

  // Sweep servo from 90° to 180°, back to 0°, and to 90° for setup
  for (servoPosition = 90; servoPosition <= 180; servoPosition++) {
    ultrasonicServo.write(servoPosition);
    delay(15);
  }
  for (servoPosition = 180; servoPosition >= 0; servoPosition--) {
    ultrasonicServo.write(servoPosition);
    delay(15);
  }
  for (servoPosition = 0; servoPosition <= 90; servoPosition++) {
    ultrasonicServo.write(servoPosition);
    delay(15);
  }

  // Set up sensor pins
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Read distance from the ultrasonic sensor
  unsigned int distance = measureDistance();

  // Read IR sensor values
  int leftSensorValue = digitalRead(LEFT_SENSOR);
  int rightSensorValue = digitalRead(RIGHT_SENSOR);

  // Print sensor values to Serial Monitor for debugging
  Serial.print("Left Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print(" | Right Sensor: ");
  Serial.print(rightSensorValue);
  Serial.print(" | Distance: ");
  Serial.println(distance);

  // Movement logic based on sensor inputs
  if ((leftSensorValue == 1) && (distance >= 10 && distance <= 30) && (rightSensorValue == 1)) {
    moveForward();
  } else if ((leftSensorValue == 1) && (rightSensorValue == 0)) {
    turnLeft();
  } else if ((leftSensorValue == 0) && (rightSensorValue == 1)) {
    turnRight();
  } else if (distance > 5 && distance < 10) {
    stopMotors();
  } else if (distance < 5) {
    moveBackward();
  } else {
    stopMotors();
  }

  delay(50); // Short delay to stabilize readings
}

// Measure distance in cm using the ultrasonic sensor
unsigned int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  ultrasonicTime = pulseIn(ECHO_PIN, HIGH);
  return ultrasonicTime / 29 / 2;  // Convert time to distance in cm
}

// Move the robot forward
void moveForward() {
  LeftFrontMotor.setSpeed(120);
  LeftFrontMotor.run(FORWARD);
  LeftBackMotor.setSpeed(120);
  LeftBackMotor.run(FORWARD);
  RightFrontMotor.setSpeed(120);
  RightFrontMotor.run(FORWARD);
  RightBackMotor.setSpeed(120);
  RightBackMotor.run(FORWARD);
}

// Move the robot backward
void moveBackward() {
  LeftFrontMotor.setSpeed(120);
  LeftFrontMotor.run(BACKWARD);
  LeftBackMotor.setSpeed(120);
  LeftBackMotor.run(BACKWARD);
  RightFrontMotor.setSpeed(120);
  RightFrontMotor.run(BACKWARD);
  RightBackMotor.setSpeed(120);
  RightBackMotor.run(BACKWARD);
}

// Turn the robot right
void turnRight() {
  LeftFrontMotor.setSpeed(200);
  LeftFrontMotor.run(FORWARD);
  LeftBackMotor.setSpeed(200);
  LeftBackMotor.run(FORWARD);
  RightFrontMotor.setSpeed(100);
  RightFrontMotor.run(BACKWARD);
  RightBackMotor.setSpeed(100);
  RightBackMotor.run(BACKWARD);
}

// Turn the robot left
void turnLeft() {
  LeftFrontMotor.setSpeed(100);
  LeftFrontMotor.run(BACKWARD);
  LeftBackMotor.setSpeed(100);
  LeftBackMotor.run(BACKWARD);
  RightFrontMotor.setSpeed(200);
  RightFrontMotor.run(FORWARD);
  RightBackMotor.setSpeed(200);
  RightBackMotor.run(FORWARD);
}

// Stop all motors
void stopMotors() {
  LeftFrontMotor.setSpeed(0);
  LeftFrontMotor.run(RELEASE);
  LeftBackMotor.setSpeed(0);
  LeftBackMotor.run(RELEASE);
  RightFrontMotor.setSpeed(0);
  RightFrontMotor.run(RELEASE);
  RightBackMotor.setSpeed(0);
  RightBackMotor.run(RELEASE);
}