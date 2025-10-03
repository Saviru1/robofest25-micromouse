/* seria monitor eke PID values enter krnn*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_tockn.h>

// ESP32 Pin Definitions
#define MOTOR_L_PWM 26    // Left motor PWM
#define MOTOR_L_DIR 25    // Left motor direction
#define MOTOR_R_PWM 32    // Right motor PWM
#define MOTOR_R_DIR 33    // Right motor direction
#define ENCODER_L 18      // Left encoder signal
#define ENCODER_R 19      // Right encoder signal
#define LED_PIN 2         // Status LED
#define TOF_FRONT_XSHUT 13 // Front VL53L0X XSHUT
#define TOF_LEFT_XSHUT 12  // Left VL53L0X XSHUT
#define TOF_RIGHT_XSHUT 14 // Right VL53L0X XSHUT
#define I2C_SDA 21        // I2C SDA
#define I2C_SCL 22        // I2C SCL

// PID constants (defaults, adjustable via Serial)
float KP_WALL = 0.15;
float KI_WALL = 0.01;
float KD_WALL = 0.05;
float KP_HEADING = 1.0;
float KI_HEADING = 0.05;
float KD_HEADING = 0.2;
#define BASE_SPEED 120  // PWM 0-255, exploration speed
#define CELL_SIZE 18    // cm
#define WALL_THRESHOLD 15 // cm

// Sensors
Adafruit_VL53L0X tof_front = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_left = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_right = Adafruit_VL53L0X();
MPU6050 mpu(Wire);

// Robot state
float currentYaw = 0.0;
float targetYaw = 0.0;
volatile long encoderLCount = 0, encoderRCount = 0;
float prevErrorWall = 0, integralWall = 0;
float prevErrorHeading = 0, integralHeading = 0;

// Function prototypes
void initSensors();
void readSensors(int &front, int &left, int &right);
float getYaw();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
float pidWall(int left, int right);
float pidHeading();
void blinkLED();
void moveStraight();

// Interrupts
void IRAM_ATTR encoderLISR() { encoderLCount++; }
void IRAM_ATTR encoderRISR() { encoderRCount++; }

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Motor setup
  ledcSetup(0, 5000, 8); ledcAttachPin(MOTOR_L_PWM, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(MOTOR_R_PWM, 1);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);

  // Encoder setup
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), encoderLISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), encoderRISR, RISING);

  // LED setup
  pinMode(LED_PIN, OUTPUT);

  // Sensor initialization
  initSensors();

  // PID tuning prompt
  Serial.println("Enter PID values (KP_WALL,KI_WALL,KD_WALL,KP_HEADING,KI_HEADING,KD_HEADING):");
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      sscanf(input.c_str(), "%f,%f,%f,%f,%f,%f", &KP_WALL, &KI_WALL, &KD_WALL, &KP_HEADING, &KI_HEADING, &KD_HEADING);
      Serial.printf("PID updated: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    KP_WALL, KI_WALL, KD_WALL, KP_HEADING, KI_HEADING, KD_HEADING);
      break;
    }
  }
  if (millis() - startTime >= 10000) {
    Serial.println("No input; using default PID values.");
  }

  delay(1000); // Stabilize sensors
}

void loop() {
  // Check for new PID values
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    sscanf(input.c_str(), "%f,%f,%f,%f,%f,%f", &KP_WALL, &KI_WALL, &KD_WALL, &KP_HEADING, &KI_HEADING, &KD_HEADING);
    Serial.printf("PID updated: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                  KP_WALL, KI_WALL, KD_WALL, KP_HEADING, KI_HEADING, KD_HEADING);
    integralWall = 0; // Reset integrals
    integralHeading = 0;
  }

  moveStraight(); // Move straight with PID control
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
    while (true) blinkLED();
  }

  retries = 3;
  digitalWrite(TOF_LEFT_XSHUT, HIGH);
  delay(10);
  while (!tof_left.begin(0x31) && retries--) {
    Serial.println("Retrying Left VL53L0X...");
    delay(100);
  }
  if (retries < 0) {
    while (true) blinkLED();
  }

  retries = 3;
  digitalWrite(TOF_RIGHT_XSHUT, HIGH);
  delay(10);
  while (!tof_right.begin(0x32) && retries--) {
    Serial.println("Retrying Right VL53L0X...");
    delay(100);
  }
  if (retries < 0) {
    while (true) blinkLED();
  }

  mpu.begin();
  mpu.calcGyroOffsets(true);
}

void readSensors(int &front, int &left, int &right) {
  front = tof_front.readRangeSingleMillimeters() / 10;
  left = tof_left.readRangeSingleMillimeters() / 10;
  right = tof_right.readRangeSingleMillimeters() / 10;
  if (tof_front.rangingTest()) front = constrain(front, 0, 200);
  else front = 200;
  if (tof_left.rangingTest()) left = constrain(left, 0, 200);
  else left = 200;
  if (tof_right.rangingTest()) right = constrain(right, 0, 200);
  else right = 200;
  Serial.printf("Sensors: F=%d, L=%d, R=%d\n", front, left, right);
}

float getYaw() {
  mpu.update();
  float yaw = mpu.getAngleZ();
  Serial.printf("Yaw: %.2f, Target: %.2f\n", yaw, targetYaw);
  return yaw;
}

void moveStraight() {
  targetYaw = getYaw(); // Set initial heading
  long targetEnc = encoderLCount + (CELL_SIZE * 20 / (PI * 4.0)); // Move 18cm
  while (encoderLCount < targetEnc) {
    int front, left, right;
    readSensors(front, left, right);
    float wallCorrection = (left < WALL_THRESHOLD && right < WALL_THRESHOLD) ? pidWall(left, right) : 0;
    float headingCorrection = pidHeading();
    int leftSpeed = constrain(BASE_SPEED + wallCorrection + headingCorrection, 0, 255);
    int rightSpeed = constrain(BASE_SPEED - wallCorrection - headingCorrection, 0, 255);
    setMotorSpeeds(leftSpeed, rightSpeed);
    Serial.printf("WallCorr=%.2f, HeadCorr=%.2f, LSpeed=%d, RSpeed=%d\n",
                  wallCorrection, headingCorrection, leftSpeed, rightSpeed);
  }
  setMotorSpeeds(0, 0);
  delay(500); // Pause before next move
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_L_DIR, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_R_DIR, rightSpeed >= 0 ? HIGH : LOW);
  ledcWrite(0, abs(leftSpeed));
  ledcWrite(1, abs(rightSpeed));
}

float pidWall(int left, int right) {
  float error = left - right;
  integralWall += error;
  float derivative = error - prevErrorWall;
  prevErrorWall = error;
  float output = KP_WALL * error + KI_WALL * integralWall + KD_WALL * derivative;
  Serial.printf("Wall PID: Error=%.2f, Integral=%.2f, Derivative=%.2f, Output=%.2f\n",
                error, integralWall, derivative, output);
  return output;
}

float pidHeading() {
  float error = targetYaw - getYaw();
  if (error > 180.0) error -= 360.0;
  else if (error < -180.0) error += 360.0;
  integralHeading += error;
  float derivative = error - prevErrorHeading;
  prevErrorHeading = error;
  float output = KP_HEADING * error + KI_HEADING * integralHeading + KD_HEADING * derivative;
  Serial.printf("Heading PID: Error=%.2f, Integral=%.2f, Derivative=%.2f, Output=%.2f\n",
                error, integralHeading, derivative, output);
  return output;
}

void blinkLED() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(500);
}