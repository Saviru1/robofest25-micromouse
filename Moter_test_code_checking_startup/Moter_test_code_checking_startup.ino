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

// Encoder Pins (Add your encoder pins here)
#define LEFT_ENCODER_A 18
#define LEFT_ENCODER_B 17
#define RIGHT_ENCODER_A 19
#define RIGHT_ENCODER_B 16

// Constants
#define BASE_SPEED 180  // Speed for forward movement
#define TURN_SPEED 100  // Speed for turns
#define WALL_THRESHOLD 15 // cm, wall detection threshold
#define CELL_SIZE 18    // cm, distance per move
#define MAX_MOVES 10    // Stop after 10 moves for testing

// Encoder Constants (Adjust based on your robot)
#define COUNTS_PER_CM 20    // Encoder counts per cm (needs calibration)
#define COUNTS_PER_90_DEGREE 150  // Encoder counts for 90° turn (needs calibration)
#define COUNTS_PER_CELL (CELL_SIZE * COUNTS_PER_CM)

// Sensors
Adafruit_VL53L0X tof_front = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_left = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_right = Adafruit_VL53L0X();
bool sensorsInitialized = false;

// Encoder Variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long leftTargetCount = 0;
long rightTargetCount = 0;
bool moving = false;
bool turning = false;

// Robot state
int moveCount = 0;
int direction = 0; // 0: North, 1: East, 2: South, 3: West

// Function prototypes
void initSensors();
void initEncoders();
void readSensors(int &front, int &left, int &right);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void moveForwardEncoder();
void turnLeftEncoder();
void turnRightEncoder();
void turnAroundEncoder();
void updateEncoders();
void leftEncoderISR();
void rightEncoderISR();
bool checkEncoderTarget();

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
  analogWriteFrequency(pwmA, 5000);
  analogWriteFrequency(pwmB, 5000);
  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);

  // LED setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Encoder setup
  initEncoders();

  // Sensor initialization
  initSensors();

  // Initial motor test with encoders
  Serial.println("Running initial motor test with encoders...");
  moveForwardEncoder();
  delay(1000);
  turnLeftEncoder();
  delay(1000);
  setMotorSpeeds(0, 0);
  Serial.println("Motor test completed. Starting navigation...");
}

void loop() {
  if (moveCount >= MAX_MOVES) {
    setMotorSpeeds(0, 0);
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Max moves reached. Stopped.");
    while (true) {}
  }

  // Only process next move if not currently moving or turning
  if (!moving && !turning) {
    int front, left, right;
    readSensors(front, left, right);

    // Navigation logic
    if (front >= WALL_THRESHOLD) {
      moveForwardEncoder();
    } else if (left >= WALL_THRESHOLD) {
      turnLeftEncoder();
    } else if (right >= WALL_THRESHOLD) {
      turnRightEncoder();
    } else {
      turnAroundEncoder();
    }
    
    moveCount++;
  }

  // Check if we've reached our encoder target
  if ((moving || turning) && checkEncoderTarget()) {
    setMotorSpeeds(0, 0);
    moving = false;
    turning = false;
    Serial.println("Movement completed");
  }

  delay(10); // Small delay to prevent overwhelming the processor
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

void initEncoders() {
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
  
  Serial.println("Encoders initialized");
}

void leftEncoderISR() {
  int a = digitalRead(LEFT_ENCODER_A);
  int b = digitalRead(LEFT_ENCODER_B);
  if (a == b) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  int a = digitalRead(RIGHT_ENCODER_A);
  int b = digitalRead(RIGHT_ENCODER_B);
  if (a == b) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void readSensors(int &front, int &left, int &right) {
  if (!sensorsInitialized) {
    Serial.println("Simulating sensor data");
    front = 200;
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

void moveForwardEncoder() {
  Serial.println("Moving forward with encoders");
  leftTargetCount = leftEncoderCount + COUNTS_PER_CELL;
  rightTargetCount = rightEncoderCount + COUNTS_PER_CELL;
  moving = true;
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
}

void turnLeftEncoder() {
  Serial.println("Turning left with encoders");
  leftTargetCount = leftEncoderCount - COUNTS_PER_90_DEGREE;
  rightTargetCount = rightEncoderCount + COUNTS_PER_90_DEGREE;
  turning = true;
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  direction = (direction + 3) % 4;
}

void turnRightEncoder() {
  Serial.println("Turning right with encoders");
  leftTargetCount = leftEncoderCount + COUNTS_PER_90_DEGREE;
  rightTargetCount = rightEncoderCount - COUNTS_PER_90_DEGREE;
  turning = true;
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  direction = (direction + 1) % 4;
}

void turnAroundEncoder() {
  Serial.println("Turning around with encoders");
  leftTargetCount = leftEncoderCount + COUNTS_PER_90_DEGREE * 2;
  rightTargetCount = rightEncoderCount - COUNTS_PER_90_DEGREE * 2;
  turning = true;
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  direction = (direction + 2) % 4;
}

bool checkEncoderTarget() {
  bool leftReached = (moving && leftEncoderCount >= leftTargetCount) || 
                     (turning && ((leftTargetCount > leftEncoderCount) ? 
                      (leftEncoderCount >= leftTargetCount) : (leftEncoderCount <= leftTargetCount)));
  
  bool rightReached = (moving && rightEncoderCount >= rightTargetCount) || 
                      (turning && ((rightTargetCount > rightEncoderCount) ? 
                       (rightEncoderCount >= rightTargetCount) : (rightEncoderCount <= rightTargetCount)));
  
  if (leftReached && rightReached) {
    Serial.printf("Encoders reached: Left=%ld/%ld, Right=%ld/%ld\n", 
                  leftEncoderCount, leftTargetCount, rightEncoderCount, rightTargetCount);
    return true;
  }
  
  // Print encoder status occasionally
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.printf("Encoders: Left=%ld/%ld, Right=%ld/%ld\n", 
                  leftEncoderCount, leftTargetCount, rightEncoderCount, rightTargetCount);
    lastPrint = millis();
  }
  
  return false;
}