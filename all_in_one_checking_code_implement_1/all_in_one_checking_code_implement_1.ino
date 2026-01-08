#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_tockn.h>
#include <SimpleKalmanFilter.h>
#include <QueueArray.h>

// Pin Definitions - CORRECTED
#define pwmB 26    // Motor B PWM (Left motor)
#define in1B 25   // Motor B direction 1
#define in2B 27   // Motor B direction 2
#define pwmA 32    // Motor A PWM (Right motor)
#define in1A 33    // Motor A direction 1
#define in2A 15    // Motor A direction 2

// Encoder Pins - CORRECTED ASSIGNMENTS
#define LEFT_ENCODER_A 16   // Left motor encoder A (Motor B)
#define LEFT_ENCODER_B 17   // Left motor encoder B (Motor B)
#define RIGHT_ENCODER_A 19  // Right motor encoder A (Motor A)
#define RIGHT_ENCODER_B 18  // Right motor encoder B (Motor A)

#define LED_PIN 2       // Status LED
#define TOF_FRONT_XSHUT 13 // SENSOR1: Front VL53L0X XSHUT
#define TOF_LEFT_XSHUT 34  // SENSOR2: Left VL53L0X XSHUT
#define TOF_RIGHT_XSHUT 14 // SENSOR3: Right VL53L0X XSHUT
#define I2C_SDA 21      // I2C SDA
#define I2C_SCL 22      // I2C SCL
#define TCRT_PIN 5      // Left TCRT5000 IR sensor
#define TCRT_PIN2 35    // Right TCRT5000 IR sensor

// Maze parameters
#define MAZE_SIZE 16
#define GOAL_X 7
#define GOAL_Y 7
#define CELL_SIZE 18    // cm
#define WALL_THRESHOLD 15 // cm

// Wall bits
#define WALL_NORTH 0x01
#define WALL_EAST 0x02
#define WALL_SOUTH 0x04
#define WALL_WEST 0x08

// Directions
#define DIR_NORTH 0
#define DIR_EAST 1
#define DIR_SOUTH 2
#define DIR_WEST 3

// PID constants (hardcoded for tuning)
float KP_WALL = 0.15;    // Wall-following proportional gain
float KI_WALL = 0.01;    // Wall-following integral gain
float KD_WALL = 0.05;    // Wall-following derivative gain
float KP_HEADING = 1.0;  // Heading proportional gain
float KI_HEADING = 0.05; // Heading integral gain
float KD_HEADING = 0.2;  // Heading derivative gain
#define BASE_SPEED 180   // Base speed for exploration
#define FAST_SPEED 255   // Max speed for optimized run
#define TURN_SPEED 100   // Speed for turns

// Sensors
Adafruit_VL53L0X tof_front = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_left = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_right = Adafruit_VL53L0X();
MPU6050 mpu(Wire);
bool sensorsInitialized = false;

// Kalman filters
SimpleKalmanFilter kalmanX(0.01, 0.01, 0.001);
SimpleKalmanFilter kalmanY(0.01, 0.01, 0.001);
SimpleKalmanFilter kalmanYaw(0.01, 0.01, 0.001);

// Maze data
uint8_t maze[MAZE_SIZE][MAZE_SIZE] = {0};
int distance[MAZE_SIZE][MAZE_SIZE];
bool visited[MAZE_SIZE][MAZE_SIZE] = {false};
QueueArray<int> queueX(100), queueY(100); // For flood-fill
int path[256]; // Store path for optimized run
int pathLength = 0;

// Robot state
int posX = 0, posY = 0;
int direction = DIR_NORTH;
float currentYaw = 0.0;
float targetYaw = 0.0;

// Encoder variables - CORRECTED
volatile long encoderCountA = 0; // Right motor (Motor A)
volatile long encoderCountB = 0; // Left motor (Motor B)
volatile int lastEncodedA = 0;
volatile int lastEncodedB = 0;
long prevEncoderA = 0, prevEncoderB = 0;

// PID variables
float prevErrorWall = 0, integralWall = 0;
float prevErrorHeading = 0, integralHeading = 0;

// Function prototypes
void initSensors();
void readSensors(int &front, int &left, int &right);
float getYaw();
void correctYaw();
void updatePosition();
void floodFill();
int getNextMove();
void moveForward();
void moveForwardSimple();
void moveForwardFast();
void turnLeft();
void turnRight();
void turnAround();
void runOptimizedPath();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
float pidWall(int left, int right);
float pidHeading();
void blinkLED();
void testSensors();
void testEncoders(); // NEW: Encoder test function
void testMotorsWithEncoders(); // NEW: Motor test with encoder feedback

// CORRECTED Interrupt handlers for encoders
void IRAM_ATTR updateEncoderA() {
  // RIGHT encoder (Motor A)
  int MSB = digitalRead(RIGHT_ENCODER_A);
  int LSB = digitalRead(RIGHT_ENCODER_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedA << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountA++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountA--;

  lastEncodedA = encoded;
}

void IRAM_ATTR updateEncoderB() {
  // LEFT encoder (Motor B)
  int MSB = digitalRead(LEFT_ENCODER_A);
  int LSB = digitalRead(LEFT_ENCODER_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedB << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountB++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountB--;

  lastEncodedB = encoded;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Stabilize Serial
  Serial.flush();
  Serial.println("=== MicroMouse Robot Setup ===");
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

  // CORRECTED Encoder setup
  Serial.println("Initializing encoders...");
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);  // Motor A Right
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);  // Motor A Right  
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);   // Motor B Left
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);   // Motor B Left
  
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), updateEncoderB, CHANGE);

  // LED setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Test encoders first
  testEncoders();

  // Sensor initialization
  initSensors();

  // Initialize distance array
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      distance[x][y] = abs(x - GOAL_X) + abs(y - GOAL_Y);
    }
  }
  visited[0][0] = true;

  Serial.println("Using default PID values: KP_WALL=0.15, KI_WALL=0.01, KD_WALL=0.05, KP_HEADING=1.0, KI_HEADING=0.05, KD_HEADING=0.2");
  Serial.println("Available commands: test, encoders, motortest, move, movesimple, flood, run");
  delay(1000); // Stabilize sensors
}

void loop() {
  static bool initialTestDone = false;
  if (!initialTestDone) {
    Serial.println("Running initial motor test with encoder feedback...");
    testMotorsWithEncoders();
    initialTestDone = true;
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.println("Command: " + input);
    
    if (input == "test") {
      testSensors();
    } else if (input == "encoders") {
      testEncoders();
    } else if (input == "move") {
      moveForward();
    } else if (input == "movesimple") {
      moveForwardSimple();
    } else if (input == "flood") {
      floodFill();
      Serial.println("Flood-fill completed.");
    } else if (input == "run") {
      runOptimizedPath();
    } else if (input == "motortest") {
      testMotorsWithEncoders();
    } else if (input == "blink") {
      blinkLED();
    }
    return; // Exit loop to process command
  }

  // Main exploration logic
  int front, left, right;
  readSensors(front, left, right);

  // Update maze walls based on ToF sensors
  if (front < WALL_THRESHOLD) maze[posX][posY] |= (1 << direction);
  if (left < WALL_THRESHOLD) maze[posX][posY] |= (1 << ((direction + 3) % 4));
  if (right < WALL_THRESHOLD) maze[posX][posY] |= (1 << ((direction + 1) % 4));

  // Flood fill algorithm
  floodFill();

  // Determine next move
  int nextDir = getNextMove();

  // Execute turn
  int turn = (nextDir - direction + 4) % 4;
  if (turn == 1) {
    Serial.println("Turning right");
    turnRight();
  } else if (turn == 3) {
    Serial.println("Turning left");
    turnLeft();
  } else if (turn == 2) {
    Serial.println("Turning around");
    turnAround();
  }

  // Move forward
  Serial.println("Moving forward");
  moveForward();

  // Update position
  switch (direction) {
    case DIR_NORTH: posY++; break;
    case DIR_EAST: posX++; break;
    case DIR_SOUTH: posY--; break;
    case DIR_WEST: posX--; break;
  }
  visited[posX][posY] = true;

  Serial.printf("Position: (%d, %d), Direction: %d\n", posX, posY, direction);

  // Goal check
  if (posX == GOAL_X && posY == GOAL_Y) {
    digitalWrite(LED_PIN, HIGH); // Indicate goal reached
    Serial.println("*** GOAL REACHED! *** Running optimized path...");
    runOptimizedPath();
  }
}

void initSensors() {
  Serial.println("Initializing sensors...");
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

  Serial.println("Initializing MPU6050...");
  mpu.begin();
  delay(100); // Allow MPU6050 to stabilize
  mpu.calcGyroOffsets(true);
  Serial.println("MPU6050 initialized and offsets calculated");
  sensorsInitialized = true;
  Serial.println("All sensors initialized successfully.");
}

void readSensors(int &front, int &left, int &right) {
  if (!sensorsInitialized) {
    // Simulate sensor data for testing
    front = 200; // Simulate no walls
    left = 8;    // Simulate left wall
    right = 10;  // Simulate right wall
  } else {
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
  }
  
  static int printCounter = 0;
  if (printCounter++ % 10 == 0) {
    Serial.printf("Sensors: F=%dcm, L=%dcm, R=%dcm\n", front, left, right);
  }
}

float getYaw() {
  if (!sensorsInitialized) {
    // Simulate yaw for testing
    return currentYaw;
  }
  mpu.update();
  float yaw = mpu.getAngleZ();
  
  static int printCounter = 0;
  if (printCounter++ % 20 == 0) {
    Serial.printf("Yaw: %.2f°, Target: %.2f°\n", yaw, targetYaw);
  }
  return yaw;
}

void correctYaw() {
  int left, right, front;
  readSensors(front, left, right);
  if (left < WALL_THRESHOLD && right < WALL_THRESHOLD) {
    float error = left - right;
    if (abs(error) < 2.0) { // Robot is centered
      currentYaw = targetYaw; // Reset yaw
    }
  }
}

void updatePosition() {
  long deltaA = encoderCountA - prevEncoderA;
  long deltaB = encoderCountB - prevEncoderB;
  prevEncoderA = encoderCountA;
  prevEncoderB = encoderCountB;

  // Calculate distance traveled (assuming 4.3cm wheel diameter, 20 pulses/revolution)
  float dist = (deltaA + deltaB) / 2.0 * (PI * 4.3 / 20.0); 
  // Calculate yaw change (assuming 10cm track width)
  float dYaw = (deltaA - deltaB) * (PI * 4.3 / 40.0);

  currentYaw = kalmanYaw.updateEstimate(currentYaw + dYaw);
  
  // Update position estimates
  posX = kalmanX.updateEstimate(posX + dist * cos(currentYaw * PI / 180));
  posY = kalmanY.updateEstimate(posY + dist * sin(currentYaw * PI / 180));

  static int printCounter = 0;
  if (printCounter++ % 20 == 0) {
    Serial.printf("Position: X=%.1f, Y=%.1f, Yaw=%.1f°\n", posX, posY, currentYaw);
  }
}

void floodFill() {
  // Reset queues
  while (!queueX.isEmpty()) queueX.pop();
  while (!queueY.isEmpty()) queueY.pop();
  
  // Start flood fill from current position
  queueX.push(posX); 
  queueY.push(posY);
  
  while (!queueX.isEmpty()) {
    int x = queueX.pop();
    int y = queueY.pop();
    
    if (visited[x][y]) {
      int minDist = 255;
      
      // Check all possible directions
      if (y < MAZE_SIZE - 1 && !(maze[x][y] & WALL_NORTH)) 
        minDist = min(minDist, distance[x][y + 1]);
      if (x < MAZE_SIZE - 1 && !(maze[x][y] & WALL_EAST)) 
        minDist = min(minDist, distance[x + 1][y]);
      if (y > 0 && !(maze[x][y] & WALL_SOUTH)) 
        minDist = min(minDist, distance[x][y - 1]);
      if (x > 0 && !(maze[x][y] & WALL_WEST)) 
        minDist = min(minDist, distance[x - 1][y]);
        
      if (distance[x][y] != minDist + 1) {
        distance[x][y] = minDist + 1;
        // Push neighbors for processing
        if (y < MAZE_SIZE - 1) { queueX.push(x); queueY.push(y + 1); }
        if (x < MAZE_SIZE - 1) { queueX.push(x + 1); queueY.push(y); }
        if (y > 0) { queueX.push(x); queueY.push(y - 1); }
        if (x > 0) { queueX.push(x - 1); queueY.push(y); }
      }
    }
  }
}

int getNextMove() {
  int minDist = distance[posX][posY];
  int nextDir = direction;

  // Check forward
  int newX = posX, newY = posY;
  switch (direction) {
    case DIR_NORTH: newY++; break;
    case DIR_EAST: newX++; break;
    case DIR_SOUTH: newY--; break;
    case DIR_WEST: newX--; break;
  }
  if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE && 
      !(maze[posX][posY] & (1 << direction)) && distance[newX][newY] < minDist) {
    minDist = distance[newX][newY];
    nextDir = direction;
  }

  // Check left
  int leftDir = (direction + 3) % 4;
  newX = posX; newY = posY;
  switch (leftDir) {
    case DIR_NORTH: newY++; break;
    case DIR_EAST: newX++; break;
    case DIR_SOUTH: newY--; break;
    case DIR_WEST: newX--; break;
  }
  if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE && 
      !(maze[posX][posY] & (1 << leftDir)) && distance[newX][newY] < minDist) {
    minDist = distance[newX][newY];
    nextDir = leftDir;
  }

  // Check right
  int rightDir = (direction + 1) % 4;
  newX = posX; newY = posY;
  switch (rightDir) {
    case DIR_NORTH: newY++; break;
    case DIR_EAST: newX++; break;
    case DIR_SOUTH: newY--; break;
    case DIR_WEST: newX--; break;
  }
  if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE && 
      !(maze[posX][posY] & (1 << rightDir)) && distance[newX][newY] < minDist) {
    minDist = distance[newX][newY];
    nextDir = rightDir;
  }

  // Check behind (only if no other options)
  int backDir = (direction + 2) % 4;
  newX = posX; newY = posY;
  switch (backDir) {
    case DIR_NORTH: newY++; break;
    case DIR_EAST: newX++; break;
    case DIR_SOUTH: newY--; break;
    case DIR_WEST: newX--; break;
  }
  if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE && 
      !(maze[posX][posY] & (1 << backDir)) && distance[newX][newY] < minDist) {
    nextDir = backDir;
  }

  path[pathLength++] = nextDir; // Store move for optimized run
  return nextDir;
}

void moveForward() {
  Serial.println("Starting forward movement with PID control");
  float startYaw = getYaw();
  targetYaw = startYaw;
  
  // Calculate target encoder count for one cell (18cm)
  long targetEnc = encoderCountB + (CELL_SIZE * 20 / (PI * 4.3)); 
  
  Serial.printf("Target encoder count: %ld (current: %ld)\n", targetEnc, encoderCountB);
  
  int printCounter = 0;
  while (encoderCountB < targetEnc) {
    int left, right, front;
    readSensors(front, left, right);
    
    float wallCorrection = pidWall(left, right);
    float headingCorrection = pidHeading();
    
    int leftSpeed = constrain(BASE_SPEED + wallCorrection + headingCorrection, -255, 255);
    int rightSpeed = constrain(BASE_SPEED - wallCorrection - headingCorrection, -255, 255);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    if (printCounter++ % 10 == 0) {
      Serial.printf("Moving: L=%d, R=%d, EncB=%ld/%ld\n", 
                    leftSpeed, rightSpeed, encoderCountB, targetEnc);
    }
    
    updatePosition();
    delay(10);
  }
  
  setMotorSpeeds(0, 0);
  Serial.println("Forward movement completed");
}

void moveForwardSimple() {
  Serial.println("Moving forward (simple mode)");
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  delay(2000); // Move for 2 seconds
  setMotorSpeeds(0, 0);
  Serial.println("Simple move completed");
}

void moveForwardFast() {
  Serial.println("Starting fast forward movement");
  float startYaw = getYaw();
  targetYaw = startYaw;
  
  long targetEnc = encoderCountB + (CELL_SIZE * 20 / (PI * 4.3));
  
  int printCounter = 0;
  while (encoderCountB < targetEnc) {
    int left, right, front;
    readSensors(front, left, right);
    
    float wallCorrection = pidWall(left, right);
    float headingCorrection = pidHeading();
    
    int leftSpeed = constrain(FAST_SPEED + wallCorrection + headingCorrection, -255, 255);
    int rightSpeed = constrain(FAST_SPEED - wallCorrection - headingCorrection, -255, 255);
    
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    if (printCounter++ % 10 == 0) {
      Serial.printf("FastMove: L=%d, R=%d, EncB=%ld/%ld\n", 
                    leftSpeed, rightSpeed, encoderCountB, targetEnc);
    }
    
    updatePosition();
    delay(10);
  }
  
  setMotorSpeeds(0, 0);
  Serial.println("Fast forward movement completed");
}

void turnLeft() {
  Serial.println("Turning left 90 degrees");
  targetYaw -= 90.0;
  if (targetYaw < -180.0) targetYaw += 360.0;
  
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  
  while (abs(getYaw() - targetYaw) > 3.0) {
    Serial.printf("Turning: CurrentYaw=%.1f, TargetYaw=%.1f\n", getYaw(), targetYaw);
    delay(50);
  }
  
  setMotorSpeeds(0, 0);
  direction = (direction + 3) % 4;
  Serial.println("Left turn completed");
}

void turnRight() {
  Serial.println("Turning right 90 degrees");
  targetYaw += 90.0;
  if (targetYaw > 180.0) targetYaw -= 360.0;
  
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  
  while (abs(getYaw() - targetYaw) > 3.0) {
    Serial.printf("Turning: CurrentYaw=%.1f, TargetYaw=%.1f\n", getYaw(), targetYaw);
    delay(50);
  }
  
  setMotorSpeeds(0, 0);
  direction = (direction + 1) % 4;
  Serial.println("Right turn completed");
}

void turnAround() {
  Serial.println("Turning around 180 degrees");
  targetYaw += 180.0;
  if (targetYaw > 180.0) targetYaw -= 360.0;
  
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  
  while (abs(getYaw() - targetYaw) > 5.0) {
    Serial.printf("Turning: CurrentYaw=%.1f, TargetYaw=%.1f\n", getYaw(), targetYaw);
    delay(50);
  }
  
  setMotorSpeeds(0, 0);
  direction = (direction + 2) % 4;
  Serial.println("Turn around completed");
}

void runOptimizedPath() {
  Serial.println("=== RUNNING OPTIMIZED PATH ===");
  // Reset to start position
  posX = 0; posY = 0; 
  direction = DIR_NORTH;
  currentYaw = 0.0; targetYaw = 0.0;
  encoderCountA = 0; encoderCountB = 0;
  prevEncoderA = 0; prevEncoderB = 0;
  
  for (int i = 0; i < pathLength; i++) {
    int nextDir = path[i];
    int turn = (nextDir - direction + 4) % 4;
    
    Serial.printf("Step %d/%d: CurrentDir=%d, NextDir=%d, Turn=%d\n", 
                  i+1, pathLength, direction, nextDir, turn);
    
    if (turn == 1) turnRight();
    else if (turn == 3) turnLeft();
    else if (turn == 2) turnAround();
    
    moveForwardFast();
    
    // Update position
    switch (direction) {
      case DIR_NORTH: posY++; break;
      case DIR_EAST: posX++; break;
      case DIR_SOUTH: posY--; break;
      case DIR_WEST: posX--; break;
    }
    
    Serial.printf("New position: (%d, %d)\n", posX, posY);
  }
  
  digitalWrite(LED_PIN, HIGH);
  Serial.println("*** OPTIMIZED PATH COMPLETED ***");
  
  // Blink LED to indicate completion
  while (true) {
    blinkLED();
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor (Motor B)
  int leftPWM = constrain(abs(leftSpeed), 0, 255);
  if (leftSpeed > 0) {
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, HIGH);
  } else {
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, LOW);
  }
  analogWrite(pwmB, leftPWM);

  // Right motor (Motor A)
  int rightPWM = constrain(abs(rightSpeed), 0, 255);
  if (rightSpeed > 0) {
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(in1A, LOW);
    digitalWrite(in2A, HIGH);
  } else {
    digitalWrite(in1A, LOW);
    digitalWrite(in2A, LOW);
  }
  analogWrite(pwmA, rightPWM);

  static int printCounter = 0;
  if (printCounter++ % 15 == 0) {
    Serial.printf("Motors: Left=%d (PWM=%d), Right=%d (PWM=%d)\n", 
                  leftSpeed, leftPWM, rightSpeed, rightPWM);
  }
}

float pidWall(int left, int right) {
  float error = left - right;
  integralWall += error;
  integralWall = constrain(integralWall, -100, 100);
  float derivative = error - prevErrorWall;
  prevErrorWall = error;
  
  float output = KP_WALL * error + KI_WALL * integralWall + KD_WALL * derivative;
  
  static int printCounter = 0;
  if (printCounter++ % 20 == 0) {
    Serial.printf("Wall PID: Error=%.1f, Output=%.1f\n", error, output);
  }
  
  return output;
}

float pidHeading() {
  float error = targetYaw - getYaw();
  // Normalize error to -180 to 180 range
  if (error > 180.0) error -= 360.0;
  else if (error < -180.0) error += 360.0;
  
  integralHeading += error;
  integralHeading = constrain(integralHeading, -100, 100);
  float derivative = error - prevErrorHeading;
  prevErrorHeading = error;
  
  float output = KP_HEADING * error + KI_HEADING * integralHeading + KD_HEADING * derivative;
  
  static int printCounter = 0;
  if (printCounter++ % 20 == 0) {
    Serial.printf("Heading PID: Error=%.1f, Output=%.1f\n", error, output);
  }
  
  return output;
}

// NEW: Encoder test function
void testEncoders() {
  Serial.println("=== ENCODER TEST ===");
  Serial.println("Manually rotate each wheel to verify encoder counts.");
  Serial.println("Positive counts should increase when moving forward.");
  Serial.println("Send any character to stop.");
  
  long lastCountA = encoderCountA;
  long lastCountB = encoderCountB;
  unsigned long lastPrint = millis();
  
  while (!Serial.available()) {
    if (millis() - lastPrint > 300) {
      long deltaA = encoderCountA - lastCountA;
      long deltaB = encoderCountB - lastCountB;
      lastCountA = encoderCountA;
      lastCountB = encoderCountB;
      
      Serial.printf("EncA (Right): %6ld (Δ=%3ld) | EncB (Left): %6ld (Δ=%3ld)\n", 
                    encoderCountA, deltaA, encoderCountB, deltaB);
      lastPrint = millis();
    }
    delay(10);
  }
  
  if (Serial.available()) {
    Serial.read(); // Clear the serial buffer
  }
  Serial.println("Encoder test stopped.");
}

// NEW: Motor test with encoder feedback
void testMotorsWithEncoders() {
  Serial.println("=== MOTOR TEST WITH ENCODER FEEDBACK ===");
  
  // Test forward
  Serial.println("1. Testing FORWARD for 2 seconds...");
  long startA = encoderCountA;
  long startB = encoderCountB;
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  delay(2000);
  setMotorSpeeds(0, 0);
  Serial.printf("   Encoder counts - Right: %ld, Left: %ld\n", 
                encoderCountA - startA, encoderCountB - startB);
  
  delay(1000);
  
  // Test left turn
  Serial.println("2. Testing LEFT TURN for 1 second...");
  startA = encoderCountA;
  startB = encoderCountB;
  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(1000);
  setMotorSpeeds(0, 0);
  Serial.printf("   Encoder counts - Right: %ld, Left: %ld\n", 
                encoderCountA - startA, encoderCountB - startB);
  
  delay(1000);
  
  // Test right turn
  Serial.println("3. Testing RIGHT TURN for 1 second...");
  startA = encoderCountA;
  startB = encoderCountB;
  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(1000);
  setMotorSpeeds(0, 0);
  Serial.printf("   Encoder counts - Right: %ld, Left: %ld\n", 
                encoderCountA - startA, encoderCountB - startB);
  
  Serial.println("Motor test completed.");
}

void testSensors() {
  Serial.println("=== SENSOR TEST ===");
  Serial.println("Testing all sensors. Send any character to stop.");
  
  while (!Serial.available()) {
    int front, left, right;
    readSensors(front, left, right);
    float yaw = getYaw();
    
    Serial.printf("ToF - Front: %3dcm, Left: %3dcm, Right: %3dcm | ", 
                  front, left, right);
    Serial.printf("Yaw: %6.1f° | ", yaw);
    Serial.printf("Encoders - Right: %6ld, Left: %6ld\n", 
                  encoderCountA, encoderCountB);
    
    delay(500);
  }
  
  if (Serial.available()) {
    Serial.read(); // Clear the serial buffer
  }
  Serial.println("Sensor test stopped.");
}

void blinkLED() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(500);
}