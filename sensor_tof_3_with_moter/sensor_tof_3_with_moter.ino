#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Pin Definitions
#define pwmB 26    // Motor B PWM (Left motor)
#define in1B 25    // Motor B direction 1
#define in2B 27    // Motor B direction 2
#define pwmA 32    // Motor A PWM (Right motor)
#define in1A 33    // Motor A direction 1
#define in2A 15    // Motor A direction 2
#define LED_PIN 2       // Status LED
#define TOF_FRONT_XSHUT 13 // Front VL53L0X XSHUT
#define TOF_LEFT_XSHUT 34  // Left VL53L0X XSHUT
#define TOF_RIGHT_XSHUT 14 // Right VL53L0X XSHUT
#define I2C_SDA 21      // I2C SDA
#define I2C_SCL 22      // I2C SCL

// Constants
#define BASE_SPEED 180  // Speed for forward movement
#define TURN_SPEED 100  // Speed for turns
#define WALL_THRESHOLD 15 // cm, wall detection threshold
#define CELL_SIZE 18    // cm, distance per move
#define MAX_MOVES 10    // Stop after 10 moves for testing

// Sensors
Adafruit_VL53L0X tof_front = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_left = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_right = Adafruit_VL53L0X();
bool sensorsInitialized = false;

// Robot state
int moveCount = 0;
int direction = 0; // 0: North, 1: East, 2: South, 3: West

// Function prototypes
void initSensors();
void readSensors(int &front, int &left, int &right);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void moveForward();
void turnLeft();
void turnRight();
void turnAround();

void setup() {
  Serial.begin(115200);
  delay(1000); // Stabilize Serial
  Serial.flush();
  Serial.println("Setup started");
  Wire.begin(I2C_SDA, I2C_SCL);

  // Motor setup
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(pwmB, OUTPUT);
  analogWriteFrequency(pwmA, 5000); // Set PWM frequency
  analogWriteFrequency(pwmB, 5000);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);

  // LED setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Sensor initialization
  initSensors();

  // Initial motor test
  Serial.println("Running initial motor test...");
  setMotorSpeeds(BASE_SPEED, BASE_SPEED); // Move forward
  delay(2000); // 2 seconds
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED); // Turn left
  delay(1000); // 1 second
  setMotorSpeeds(0, 0); // Stop
  Serial.println("Motor test completed. Starting navigation...");
}

void loop() {
  if (moveCount >= MAX_MOVES) {
    setMotorSpeeds(0, 0);
    digitalWrite(LED_PIN, HIGH); // Indicate test complete
    Serial.println("Max moves reached. Stopped.");
    while (true) {} // Stop
  }

  int front, left, right;
  readSensors(front, left, right);

  // Navigation logic
  if (front >= WALL_THRESHOLD) { // No wall ahead
    moveForward();
  } else if (left >= WALL_THRESHOLD) { // Open left
    turnLeft();
    moveForward();
  } else if (right >= WALL_THRESHOLD) { // Open right
    turnRight();
    moveForward();
  } else { // Blocked, turn around
    turnAround();
  }

  moveCount++;
}

void initSensors() {
  pinMode(TOF_FRONT_XSHUT, OUTPUT);
  pinMode(TOF_LEFT_XSHUT, OUTPUT);
  pinMode(TOF_RIGHT_XSHUT, OUTPUT);
  digitalWrite(TOF_FRONT_XSHUT, LOW);
  digitalWrite(TOF_LEFT_XSHUT, LOW);
  digitalWrite(TOF_RIGHT_XSHUT, LOW);
  delay(10);

  int retries = 3;
  digitalWrite(TOF_FRONT_XSHUT, HIGH);
  delay(10);
  while (!tof_front.begin(0x30) && retries--) {
    Serial.println("Retrying Front VL53L0X...");
    delay(100);
  }
  if (retries < 0) {
    Serial.println("Front VL53L0X failed! Using simulated data.");
    sensorsInitialized = false;
    return;
  }

  retries = 3;
  digitalWrite(TOF_LEFT_XSHUT, HIGH);
  delay(10);
  while (!tof_left.begin(0x31) && retries--) {
    Serial.println("Retrying Left VL53L0X...");
    delay(100);
  }
  if (retries < 0) {
    Serial.println("Left VL53L0X failed! Using simulated data.");
    sensorsInitialized = false;
    return;
  }

  retries = 3;
  digitalWrite(TOF_RIGHT_XSHUT, HIGH);
  delay(10);
  while (!tof_right.begin(0x32) && retries--) {
    Serial.println("Retrying Right VL53L0X...");
    delay(100);
  }
  if (retries < 0) {
    Serial.println("Right VL53L0X failed! Using simulated data.");
    sensorsInitialized = false;
    return;
  }

  sensorsInitialized = true;
  Serial.println("All ToF sensors initialized successfully.");
}

void readSensors(int &front, int &left, int &right) {
  if (!sensorsInitialized) {
    Serial.println("Simulating sensor data");
    front = 200; // No walls
    left = 200;
    right = 200;
    return;
  }

  VL53L0X_RangingMeasurementData_t measureFront;
  VL53L0X_RangingMeasurementData_t measureLeft;
  VL53L0X_RangingMeasurementData_t measureRight;

  tof_front.rangingTest(&measureFront, false);
  tof_left.rangingTest(&measureLeft, false);
  tof_right.rangingTest(&measureRight, false);

  front = (measureFront.RangeStatus != 4) ? measureFront.RangeMilliMeter / 10 : 200;
  left = (measureLeft.RangeStatus != 4) ? measureLeft.RangeMilliMeter / 10 : 200;
  right = (measureRight.RangeStatus != 4) ? measureRight.RangeMilliMeter / 10 : 200;

  front = constrain(front, 0, 200);
  left = constrain(left, 0, 200);
  right = constrain(right, 0, 200);

  static int printCounter = 0;
  if (printCounter++ % 10 == 0) {
    Serial.printf("Sensors: F=%d, L=%d, R=%d\n", front, left, right);
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  int leftPWM = constrain(abs(leftSpeed), 0, 255);
  int rightPWM = constrain(abs(rightSpeed), 0, 255);
  if (leftSpeed > 0) {
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
    analogWrite(pwmB, leftPWM);
  } else if (leftSpeed < 0) {
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, HIGH);
    analogWrite(pwmB, leftPWM);
  } else {
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, LOW);
    analogWrite(pwmB, 0);
  }
  if (rightSpeed > 0) {
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
    analogWrite(pwmA, rightPWM);
  } else if (rightSpeed < 0) {
    digitalWrite(in1A, LOW);
    digitalWrite(in2A, HIGH);
    analogWrite(pwmA, rightPWM);
  } else {
    digitalWrite(in1A, LOW);
    digitalWrite(in2A, LOW);
    analogWrite(pwmA, 0);
  }
  static int printCounter = 0;
  if (printCounter++ % 10 == 0) {
    Serial.printf("Motors: Left=%d (PWM=%d), Right=%d (PWM=%d)\n", leftSpeed, leftPWM, rightSpeed, rightPWM);
  }
}

void moveForward() {
  Serial.println("Moving forward");
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  delay(2000); // Move for ~18 cm (adjust based on testing)
  setMotorSpeeds(0, 0);
}

void turnLeft() {
  Serial.println("Turning left");
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(1000); // ~90 degrees (adjust based on testing)
  setMotorSpeeds(0, 0);
  direction = (direction + 3) % 4; // Update direction
}

void turnRight() {
  Serial.println("Turning right");
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(1000); // ~90 degrees
  setMotorSpeeds(0, 0);
  direction = (direction + 1) % 4;
}

void turnAround() {
  Serial.println("Turning around");
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(2000); // ~180 degrees
  setMotorSpeeds(0, 0);
  direction = (direction + 2) % 4;
}