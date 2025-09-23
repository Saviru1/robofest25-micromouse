#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>

// ======================= Motor pins (L9110/L298 two-pin-per-motor style) =======================
#define AIN1 26
#define AIN2 25
#define BIN1 33
#define BIN2 32

// ======================= IR sensors =======================
#define IR_LEFT 4    // INPUT-ONLY is OK; beware ESP32 strapping on GPIO4 if pulled low at boot
#define IR_RIGHT 34  // input-only pin; OK for digital/analog input

// ======================= TOF VL53L0X XSHUT pins (one XSHUT per sensor) =======================
// IMPORTANT: Avoid conflicts with motor pins. Reassigned RIGHT_DIAG from 33 to 23.
#define TOF_FRONT_XSHUT      13
#define TOF_LEFT_XSHUT       12
#define TOF_RIGHT_XSHUT      14
#define TOF_LEFT_DIAG_XSHUT  27
#define TOF_RIGHT_DIAG_XSHUT 23  // CHANGED from 33 -> 23

// I2C is SDA=21, SCL=22 by default on ESP32-DevKitC
// If your board differs, uncomment and set explicitly:
// #define SDA_PIN 21
// #define SCL_PIN 22

// ======================= Light and misc =======================
#define LIGHT_PIN 2

// ======================= Maze parameters =======================
#define MAZE_SIZE 16
#define CELL_SIZE_CM 16.0f

// ======================= PID parameters =======================
#define KP 1.0f
#define KI 0.01f
#define KD 0.1f

// Motion tuning
#define BASE_SPEED 150          // PWM 0..255
#define TURN_SPEED 120          // PWM limit for turning
#define SPEED_CM_PER_SEC 10.0f  // Rough estimate at BASE_SPEED (calibrate on your robot)

// Front wall stop threshold (cm)
#define FRONT_WALL_STOP_CM 10

// ======================= Sensors / devices =======================
Adafruit_MPU6050 mpu;

// 5x ToF sensors
VL53L0X tofFront;
VL53L0X tofLeft;
VL53L0X tofRight;
VL53L0X tofLeftDiag;
VL53L0X tofRightDiag;

// Unique I2C addresses we’ll assign during init (7-bit)
#define ADDR_FRONT     0x30
#define ADDR_LEFT      0x31
#define ADDR_RIGHT     0x32
#define ADDR_LEFT_DIAG 0x33
#define ADDR_RIGHT_DIAG 0x34

// ======================= Flood fill data =======================
int flood[MAZE_SIZE][MAZE_SIZE];
int visited[MAZE_SIZE][MAZE_SIZE];
int walls[MAZE_SIZE][MAZE_SIZE][4]; // 0:N, 1:E, 2:S, 3:W
int currentX = 0, currentY = 0;
int currentDir = 0; // 0:N, 1:E, 2:S, 3:W

// ======================= PID / heading =======================
float pidError = 0, pidLastError = 0, pidIntegral = 0;
float targetAngleDeg = 0.0f;     // degrees
float gyroBias_dps = 0.0f;       // degrees per second bias
float currentAngleDeg = 0.0f;    // integrated yaw in degrees
unsigned long lastGyroTime = 0;  // micros

// ======================= Sensor filtering =======================
#define FILTER_SAMPLES 5
int frontFilter[FILTER_SAMPLES], leftFilter[FILTER_SAMPLES], rightFilter[FILTER_SAMPLES];
int leftDiagFilter[FILTER_SAMPLES], rightDiagFilter[FILTER_SAMPLES];
int filterIndex = 0;

// ======================= Helpers =======================
static inline float wrapAngleDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void driveMotor(int pinA, int pinB, int speed) {
  // Speed in [-255, 255]
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(pinA, speed);
    analogWrite(pinB, 0);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, -speed);
  }
}

void setWheelSpeeds(int left, int right) {
  driveMotor(AIN1, AIN2, left);
  driveMotor(BIN1, BIN2, right);
}

void stopMotors() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

// ======================= Setup and initialization =======================
void initializePins() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  stopMotors();
  digitalWrite(LIGHT_PIN, LOW);
}

bool initOneTOF(VL53L0X &sensor, int xshutPin, uint8_t newAddr) {
  // Bring this sensor out of reset
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, HIGH);
  delay(10);

  if (!sensor.init()) {
    Serial.println("Failed to detect a VL53L0X sensor during init().");
    return false;
  }
  sensor.setTimeout(500);
  sensor.setAddress(newAddr);
  sensor.startContinuous();
  return true;
}

bool initializeAllTOF() {
  // Hold all in shutdown
  int xshuts[5] = {
    TOF_FRONT_XSHUT, TOF_LEFT_XSHUT, TOF_RIGHT_XSHUT, TOF_LEFT_DIAG_XSHUT, TOF_RIGHT_DIAG_XSHUT
  };
  for (int i = 0; i < 5; i++) {
    pinMode(xshuts[i], OUTPUT);
    digitalWrite(xshuts[i], LOW);
  }
  delay(20);

  // Bring up one by one, init, assign new address, start continuous
  if (!initOneTOF(tofFront,     TOF_FRONT_XSHUT,     ADDR_FRONT)) return false;
  if (!initOneTOF(tofLeft,      TOF_LEFT_XSHUT,      ADDR_LEFT)) return false;
  if (!initOneTOF(tofRight,     TOF_RIGHT_XSHUT,     ADDR_RIGHT)) return false;
  if (!initOneTOF(tofLeftDiag,  TOF_LEFT_DIAG_XSHUT, ADDR_LEFT_DIAG)) return false;
  if (!initOneTOF(tofRightDiag, TOF_RIGHT_DIAG_XSHUT,ADDR_RIGHT_DIAG)) return false;

  // Init filters
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    frontFilter[i] = leftFilter[i] = rightFilter[i] = 100;
    leftDiagFilter[i] = rightDiagFilter[i] = 100;
  }
  filterIndex = 0;
  return true;
}

void initializeGyro() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate bias (deg/s)
  Serial.println("Calibrating Gyro...");
  float sum_dps = 0.0f;
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float dps = g.gyro.z * 180.0f / PI; // rad/s -> deg/s
    sum_dps += dps;
    delay(5);
  }
  gyroBias_dps = sum_dps / 200.0f;
  currentAngleDeg = 0.0f;
  lastGyroTime = micros(); // IMPORTANT: initialize to avoid huge first dt
  Serial.print("Gyro Bias (deg/s): "); Serial.println(gyroBias_dps, 6);
}

void initializeFloodFill() {
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      flood[x][y] = abs(x - MAZE_SIZE/2) + abs(y - MAZE_SIZE/2);
      visited[x][y] = 0;
      for (int d = 0; d < 4; d++) {
        walls[x][y][d] = 0; // 0 unknown, 1 no wall, 2 wall
      }
    }
  }
  visited[currentX][currentY] = 1;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  initializePins();

  // If your board does not default to SDA=21, SCL=22, uncomment:
  // Wire.begin(SDA_PIN, SCL_PIN);
  Wire.begin();

  if (!initializeAllTOF()) {
    Serial.println("TOF init failed. Check wiring and XSHUT pins.");
    while (1) delay(10);
  }

  initializeGyro();
  initializeFloodFill();

  digitalWrite(LIGHT_PIN, HIGH);
  delay(500);

  Serial.println("Micromouse Initialized");
}

// ======================= TOF reading with median filter =======================
int medianOfArray(int arr[], int n) {
  int tmp[FILTER_SAMPLES];
  for (int i = 0; i < n; i++) tmp[i] = arr[i];
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (tmp[i] > tmp[j]) {
        int t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t;
      }
    }
  }
  return tmp[n/2];
}

int readFilteredDistance(VL53L0X &sensor, int filterArray[]) {
  int mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    // Return a large distance on timeout
    mm = 2000;
  }
  int cm = mm / 10; // mm -> cm
  filterArray[filterIndex] = cm;
  return medianOfArray(filterArray, FILTER_SAMPLES);
}

void readAllSensors(int &front, int &left, int &right, int &leftDiag, int &rightDiag) {
  front    = readFilteredDistance(tofFront,     frontFilter);
  left     = readFilteredDistance(tofLeft,      leftFilter);
  right    = readFilteredDistance(tofRight,     rightFilter);
  leftDiag = readFilteredDistance(tofLeftDiag,  leftDiagFilter);
  rightDiag= readFilteredDistance(tofRightDiag, rightDiagFilter);

  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;
}

// ======================= Wall mapping =======================
void updateWallsWithSensors(int front, int left, int right, int leftDiag, int rightDiag) {
  bool wallFront = front < 12; // cm thresholds (tune!)
  bool wallLeft  = (left < 15)  || (leftDiag < 10);
  bool wallRight = (right < 15) || (rightDiag < 10);

  // Mark current cell’s walls
  walls[currentX][currentY][currentDir] = wallFront ? 2 : 1;
  walls[currentX][currentY][(currentDir + 3) % 4] = wallLeft ? 2 : 1;
  walls[currentX][currentY][(currentDir + 1) % 4] = wallRight ? 2 : 1;

  // Optionally also mark the neighbor cell’s opposite wall when known (not strictly required)
}

// ======================= Gyro-driven PID straight =======================
void updateGyroAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float dps = g.gyro.z * 180.0f / PI - gyroBias_dps; // deg/s
  unsigned long now = micros();
  float dt = (now - lastGyroTime) / 1000000.0f;
  if (dt < 0.0001f || dt > 0.1f) {
    // skip unrealistic dt (e.g., first sample or large pause)
    dt = 0.0f;
  } else {
    currentAngleDeg += dps * dt;
    currentAngleDeg = wrapAngleDeg(currentAngleDeg);
  }
  lastGyroTime = now;
}

void pidControlStraight() {
  updateGyroAngle();

  pidError = wrapAngleDeg(targetAngleDeg - currentAngleDeg);
  pidIntegral += pidError * 0.01f; // rough timestep; you can use dt if you cache it
  pidIntegral = constrain(pidIntegral, -100.0f, 100.0f);
  float pidDerivative = (pidError - pidLastError) / 0.01f;

  float correction = KP * pidError + KI * pidIntegral + KD * pidDerivative;
  pidLastError = pidError;

  int leftSpeed = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setWheelSpeeds(leftSpeed, rightSpeed);
}

void moveForwardPID(int cells) {
  targetAngleDeg = currentAngleDeg; // maintain current heading
  pidIntegral = 0;
  pidLastError = 0;

  float targetDistCm = cells * CELL_SIZE_CM;
  float traveledCm = 0.0f;
  unsigned long startMs = millis();

  while (traveledCm < targetDistCm) {
    pidControlStraight();

    // simple open-loop distance estimate (calibrate SPEED_CM_PER_SEC)
    float elapsed = (millis() - startMs) / 1000.0f;
    traveledCm = elapsed * SPEED_CM_PER_SEC;

    // Safety: stop if wall detected too close
    int f, l, r, ld, rd;
    readAllSensors(f, l, r, ld, rd);
    updateWallsWithSensors(f, l, r, ld, rd);
    if (f <= FRONT_WALL_STOP_CM) {
      Serial.println("Front wall detected during forward move; stopping early.");
      break;
    }

    delay(10);
  }
  stopMotors();
}

// ======================= Turning =======================
void turnToAngle(float absoluteTargetDeg) {
  absoluteTargetDeg = wrapAngleDeg(absoluteTargetDeg);
  const float tol = 2.0f; // degrees
  unsigned long lastUpdate = millis();

  while (true) {
    updateGyroAngle();
    float err = wrapAngleDeg(absoluteTargetDeg - currentAngleDeg);
    if (fabs(err) <= tol) break;

    // Proportional turn
    float turn = constrain(err * 2.0f, -TURN_SPEED, TURN_SPEED); // tune gain
    int left = (int)turn;
    int right = -(int)turn;
    setWheelSpeeds(left, right);

    // Basic timeout protection
    if (millis() - lastUpdate > 2000) {
      lastUpdate = millis();
    }

    delay(10);
  }
  stopMotors();
}

// ======================= Flood fill =======================
int getAccessibleMinNeighbor(int x, int y) {
  int minVal = 100000;
  const int dirs[4][2] = {{0,1},{1,0},{0,-1},{-1,0}}; // N,E,S,W
  for (int d = 0; d < 4; d++) {
    int nx = x + dirs[d][0];
    int ny = y + dirs[d][1];
    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
      // No wall means walls[x][y][d] != 2 and also neighbor’s opposite not 2
      if (walls[x][y][d] != 2 /* && walls[nx][ny][(d+2)%4] != 2 */) {
        minVal = min(minVal, flood[nx][ny]);
      }
    }
  }
  return minVal;
}

void updateFloodFill() {
  bool updated = true;
  while (updated) {
    updated = false;
    for (int x = 0; x < MAZE_SIZE; x++) {
      for (int y = 0; y < MAZE_SIZE; y++) {
        int mn = getAccessibleMinNeighbor(x, y);
        if (mn + 1 < flood[x][y]) {
          flood[x][y] = mn + 1;
          updated = true;
        }
      }
    }
  }
}

// ======================= Navigation =======================
bool hasWall(int fromDir, int toDir) {
  int relDir = (toDir - fromDir + 4) % 4;
  switch (relDir) {
    case 0: return walls[currentX][currentY][fromDir] == 2;           // front
    case 1: return walls[currentX][currentY][(fromDir+1)%4] == 2;     // right
    case 3: return walls[currentX][currentY][(fromDir+3)%4] == 2;     // left
    case 2: return true; // avoid back steps unless stuck
  }
  return true;
}

void getNextCell(int &x, int &y, int dir) {
  switch (dir) {
    case 0: y++; break;
    case 1: x++; break;
    case 2: y--; break;
    case 3: x--; break;
  }
}

int chooseNextDirection() {
  int bestDir = -1;
  int minFlood = 100000;

  for (int dir = 0; dir < 4; dir++) {
    if (!hasWall(currentDir, dir)) {
      int nx = currentX, ny = currentY;
      getNextCell(nx, ny, dir);
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        if (flood[nx][ny] < minFlood) {
          minFlood = flood[nx][ny];
          bestDir = dir;
        }
      }
    }
  }

  // If blocked, allow backtracking (ignore hasWall but still choose min flood)
  if (bestDir == -1) {
    for (int dir = 0; dir < 4; dir++) {
      int nx = currentX, ny = currentY;
      getNextCell(nx, ny, dir);
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        if (flood[nx][ny] < minFlood) {
          minFlood = flood[nx][ny];
          bestDir = dir;
        }
      }
    }
  }

  return bestDir;
}

void updatePosition(int newDir) {
  switch (newDir) {
    case 0: currentY++; break;
    case 1: currentX++; break;
    case 2: currentY--; break;
    case 3: currentX--; break;
  }
  currentDir = newDir;
  visited[currentX][currentY] = 1;
}

void moveToDirectionOptimized(int targetDir) {
  int relDir = (targetDir - currentDir + 4) % 4;
  switch (relDir) {
    case 0:
      moveForwardPID(1);
      break;
    case 1:
      turnToAngle(wrapAngleDeg(currentAngleDeg + 90.0f));
      moveForwardPID(1);
      break;
    case 3:
      turnToAngle(wrapAngleDeg(currentAngleDeg - 90.0f));
      moveForwardPID(1);
      break;
    case 2:
      turnToAngle(wrapAngleDeg(currentAngleDeg + 180.0f));
      moveForwardPID(1);
      break;
  }
  updatePosition(targetDir);
}

// ======================= Main loop =======================
void loop() {
  int f, l, r, ld, rd;
  readAllSensors(f, l, r, ld, rd);
  updateWallsWithSensors(f, l, r, ld, rd);

  int nextDir = chooseNextDirection();
  if (nextDir != -1) {
    moveToDirectionOptimized(nextDir);
    updateFloodFill();

    if (currentX == MAZE_SIZE/2 && currentY == MAZE_SIZE/2) {
      Serial.println("Reached center!");
      // TODO: return-to-start or speed-run logic
      delay(500);
    }
  }

  delay(30);
}