# 🤖 Micromouse - SLIIT ROBOFEST 2025

> **Autonomous maze-solving robot for SLIIT ROBOFEST 2025 (University Category)**

A fully autonomous micromouse robot capable of exploring, mapping, and solving a 16×16 maze using optimal pathfinding algorithms.  This project combines embedded systems programming, sensor fusion, and advanced navigation algorithms to create a competition-ready solution.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Software Architecture](#software-architecture)
- [Installation & Setup](#installation--setup)
- [Code Structure](#code-structure)
- [Algorithms](#algorithms)
- [Configuration & Tuning](#configuration--tuning)
- [Testing](#testing)
- [Competition Rules](#competition-rules)
- [Contributors](#contributors)
- [License](#license)

---

## 🎯 Overview

This micromouse robot is designed to compete in the **SLIIT ROBOFEST 2025 University Category**. The robot autonomously navigates through a 16×16 cell maze (each cell 16cm × 16cm), discovers walls, maps the environment, and calculates the shortest path to the center using the flood-fill algorithm.  

### Project Goals
- ✅ Autonomous maze exploration and mapping
- ✅ Real-time wall detection using multiple ToF sensors
- ✅ Gyroscope-based heading stabilization
- ✅ PID-controlled straight-line motion
- ✅ Optimal pathfinding with flood-fill algorithm
- ✅ Competition-ready performance

---

## ✨ Features

### Hardware Features
- **5x VL53L0X Time-of-Flight (ToF) sensors** for precise wall detection
  - Front sensor
  - Left and right sensors
  - Left and right diagonal sensors
- **MPU6050 IMU** for gyroscope-based heading control
- **Dual IR sensors** for supplementary detection
- **ESP32 microcontroller** for processing and control
- **L9110/L298N motor driver** for dual DC motor control

### Software Features
- **Flood Fill Algorithm** for optimal pathfinding
- **PID Control** for smooth, straight-line motion
- **Gyroscope Integration** for precise angle tracking and turning
- **Median Filtering** for robust sensor readings
- **Dynamic Wall Mapping** with real-time updates
- **Continuous ToF Sensing** for low-latency measurements

---

## 🔧 Hardware Components

| Component | Model/Type | Quantity | Purpose |
|-----------|-----------|----------|---------|
| Microcontroller | ESP32-DevKitC | 1 | Main processing unit |
| IMU | MPU6050 | 1 | Gyroscope & accelerometer |
| ToF Sensors | VL53L0X | 5 | Wall distance measurement |
| IR Sensors | Analog IR | 2 | Supplementary detection |
| Motor Driver | L9110/L298N | 1 | Motor control |
| DC Motors | N20/similar | 2 | Locomotion |
| Power Supply | LiPo Battery | 1 | Power source |

### Pin Configuration (ESP32)

#### Motor Pins
- `AIN1`: GPIO 26
- `AIN2`: GPIO 25
- `BIN1`: GPIO 33
- `BIN2`: GPIO 32

#### Sensor Pins
- **IR Sensors:**
  - `IR_LEFT`: GPIO 4
  - `IR_RIGHT`: GPIO 34

- **ToF XSHUT Pins:**
  - `TOF_FRONT_XSHUT`: GPIO 13
  - `TOF_LEFT_XSHUT`: GPIO 12
  - `TOF_RIGHT_XSHUT`: GPIO 14
  - `TOF_LEFT_DIAG_XSHUT`: GPIO 27
  - `TOF_RIGHT_DIAG_XSHUT`: GPIO 23

- **I2C (ToF & MPU6050):**
  - `SDA`: GPIO 21
  - `SCL`: GPIO 22

#### Other
- `LIGHT_PIN`: GPIO 2 (Status LED)

---

## 🏗️ Software Architecture

### Main Components

1. **Sensor Management**
   - Multi-sensor ToF initialization with unique I2C addresses
   - Median filtering for noise reduction
   - Continuous measurement mode for low latency

2. **Navigation System**
   - Flood-fill algorithm for pathfinding
   - Wall detection and mapping
   - Position tracking in 16×16 grid

3. **Motion Control**
   - Gyroscope-based PID control for straight movement
   - Precise angle-based turning
   - Dynamic speed adjustment

4. **State Machine**
   - Exploration phase
   - Return-to-start phase
   - Speed-run phase (future enhancement)

---

## 📦 Installation & Setup

### Prerequisites
- **Arduino IDE** (v1.8.19 or later) or **PlatformIO**
- **ESP32 Board Support** installed
- **Required Libraries:**
  - `Adafruit_MPU6050`
  - `Adafruit_Sensor`
  - `VL53L0X` (by Pololu)
  - `Wire` (I2C communication)

### Installation Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/Saviru1/robofest25-micromouse.git
   cd robofest25-micromouse
   ```

2. **Install required libraries**
   - In Arduino IDE:   `Sketch → Include Library → Manage Libraries`
   - Search and install:  
     - "Adafruit MPU6050"
     - "Adafruit Unified Sensor"
     - "VL53L0X" by Pololu

3. **Open the main code**
   - Open `micromouse_code/micromouse.ino`

4. **Configure board settings**
   - Board: "ESP32 Dev Module"
   - Upload Speed: 115200
   - Flash Frequency: 80MHz
   - Partition Scheme: "Default"

5. **Upload to ESP32**
   - Connect ESP32 via USB
   - Select correct COM port
   - Click "Upload"

---

## 📁 Code Structure

```
robofest25-micromouse/
├── Document/
│   └── ROBOFEST 2025 University Category Technical Specifications.pdf
├── micromouse_code/
│   └── micromouse.ino              # Main implementation
├── ir_sensor_checking_code/
│   └── ir_sensor_checking_code.ino # IR sensor testing
├── nithu code2                      # Alternative implementation (testing)
├── nithus code                      # Alternative implementation (testing)
├── README.md
└── LICENSE
```

### Main Code Components (`micromouse.ino`)

- **Sensor Initialization** (lines 118-171): ToF and IMU setup
- **Gyroscope Calibration** (lines 173-196): Bias calculation
- **Flood Fill Algorithm** (lines 378-409): Pathfinding logic
- **PID Control** (lines 301-349): Straight-line motion control
- **Navigation Logic** (lines 412-497): Direction selection and movement
- **Main Loop** (lines 500-518): Sensor reading and decision-making

---

## 🧠 Algorithms

### 1. Flood Fill Algorithm

The robot uses the **flood-fill algorithm** to find the shortest path:

- **Initialization**:  Each cell is assigned a distance value from the center
- **Wall Discovery**: As the robot explores, it updates wall information
- **Path Selection**: Always moves toward the cell with the lowest flood value
- **Dynamic Updating**: Recalculates paths when new walls are discovered

```cpp
// Simplified flood fill logic
1. Start at (0,0), goal at center (8,8)
2. Read sensors, detect walls
3. Update wall map
4. Recalculate flood values
5. Move to neighbor with lowest value
6. Repeat until goal reached
```

### 2. PID Control

**Proportional-Integral-Derivative** control for straight-line motion:

```cpp
correction = Kp * error + Ki * integral + Kd * derivative
leftSpeed = BASE_SPEED + correction
rightSpeed = BASE_SPEED - correction
```

**Default Parameters:**
- `KP = 1.0`
- `KI = 0.01`
- `KD = 0.1`

### 3. Gyroscope Integration

Continuous angle tracking using MPU6050:

```cpp
currentAngle += (gyroRate - bias) * deltaTime
```

This ensures precise turning and heading maintenance.

---

## ⚙️ Configuration & Tuning

### Critical Parameters (in `micromouse.ino`)

```cpp
// Maze configuration
#define MAZE_SIZE 16
#define CELL_SIZE_CM 16.0f

// Motion parameters
#define BASE_SPEED 150          // PWM (0-255)
#define TURN_SPEED 120          // PWM for turning
#define SPEED_CM_PER_SEC 10.0f  // Calibrate this!  

// Wall detection thresholds (cm)
#define FRONT_WALL_STOP_CM 10
// In updateWallsWithSensors():
bool wallFront = front < 12;
bool wallLeft  = (left < 15) || (leftDiag < 10);
bool wallRight = (right < 15) || (rightDiag < 10);

// PID constants
#define KP 1.0f
#define KI 0.01f
#define KD 0.1f
```

### Tuning Guide

1. **Speed Calibration**
   - Measure actual distance traveled in 1 second at `BASE_SPEED`
   - Update `SPEED_CM_PER_SEC` accordingly

2. **Wall Detection Thresholds**
   - Place robot at known distances from walls
   - Read sensor values via Serial Monitor
   - Adjust thresholds in `updateWallsWithSensors()`

3. **PID Tuning**
   - Start with `KP` only (set `KI` and `KD` to 0)
   - Increase until robot oscillates slightly
   - Add `KD` to reduce oscillation
   - Add `KI` if steady-state error persists

4. **Gyroscope Calibration**
   - Robot must be stationary during startup
   - Bias is calculated automatically over 200 samples
   - Ensure stable surface during calibration

---

## 🧪 Testing

### 1. IR Sensor Test

Use `ir_sensor_checking_code/ir_sensor_checking_code.ino` to verify IR sensors:

```cpp
// Adjust threshold based on Serial Monitor readings
int threshold = 1000;
```

### 2. ToF Sensor Test

Add debug code to `loop()`:

```cpp
Serial.print("Front:   "); Serial.print(f);
Serial.print(" | Left: "); Serial.print(l);
Serial.print(" | Right: "); Serial.println(r);
```

### 3. Motor Test

Verify motor directions: 
- Forward: Both motors should spin forward
- Turn left: Left motor backward, right motor forward
- Turn right: Left motor forward, right motor backward

### 4. Gyroscope Test

Monitor angle during manual turns:

```cpp
Serial.print("Angle: "); Serial.println(currentAngleDeg);
```

Verify that 90° physical turn ≈ 90° reading.

---

## 🏆 Competition Rules

Based on **SLIIT ROBOFEST 2025 University Category**: 

- **Maze Specifications:**
  - 16×16 grid
  - Each cell:  16cm × 16cm
  - Wall height: ~5cm
  - Start:  Corner (0,0)
  - Goal: Center (4 cells)

- **Robot Constraints:**
  - Maximum size: Must fit in one cell
  - Autonomous operation (no remote control)
  - Must not damage maze walls

- **Scoring:**
  - Time to reach center
  - Shortest path optimization
  - Return-to-start run (optional)

📄 **Full specifications**:  See `Document/ROBOFEST 2025 University Category Technical Specifications.pdf`

---

## 👥 Contributors


**Copyright Holder:** FERDINANDO K. P. S. S. M. D.S.S. 

### Contributions

- **[@Saviru1](https://github.com/Saviru1)**
- **[@Nithurjithan-Nithur8](https://github.com/Nithurjithan-Nithur8)**
- **[@Kehara-Kehara2002](https://github.com/Kehara-Kehara2002)**
---

## 📄 License

This project is licensed under the **MIT License**. 

```
Copyright (c) 2025 FERDINANDO K.P.S.S.M.D.S.S.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.  
```

See [LICENSE](./LICENSE) for full text.  

---

## 🔗 Resources

- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [VL53L0X Datasheet](https://www.pololu.com/product/2490)
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [Flood Fill Algorithm Explanation](https://en.wikipedia.org/wiki/Flood_fill)

---

## 🚀 Future Enhancements

- [ ] Implement speed-run mode after initial exploration
- [ ] Add return-to-start functionality
- [ ] Optimize turn speeds for faster times
- [ ] Add LCD display for status information
- [ ] Implement encoder-based distance measurement
- [ ] Add obstacle avoidance for competition scenarios

---

## 📞 Contact

For questions or collaboration:  
- **GitHub**: [@Saviru1](https://github.com/Saviru1)
- **Repository**: [robofest25-micromouse](https://github.com/Saviru1/robofest25-micromouse)

---

<div align="center">

**Built for SLIIT ROBOFEST 2025** 🏆

[![License:  MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Contributors](https://img.shields.io/badge/Contributors-3-brightgreen. svg)](https://github.com/Saviru1/robofest25-micromouse/graphs/contributors)

</div>
