#include <Wire.h>

const int I2C_SDA = 21;
const int I2C_SCL = 22;
const byte MPU_ADDR = 0x68;  // Default I2C address for MPU6050

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Wake up MPU6050 (it's in sleep mode by default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  delay(100);
  
  // Check WHO_AM_I
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);

  if (Wire.available()) {
    byte whoami = Wire.read();
    Serial.print("WHO_AM_I = 0x");
    Serial.println(whoami, HEX);
    if (whoami == 0x68) {
      Serial.println("MPU6050 detected successfully!");
    } else {
      Serial.println("Unexpected WHO_AM_I response!");
    }
  }
}

void loop() {
  int16_t gx, gy, gz;

  // Request gyroscope data (6 bytes)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // Starting register for gyro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  gx = (Wire.read() << 8) | Wire.read(); // GYRO_XOUT
  gy = (Wire.read() << 8) | Wire.read(); // GYRO_YOUT
  gz = (Wire.read() << 8) | Wire.read(); // GYRO_ZOUT

  // Print raw values
  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" | Gyro Y: "); Serial.print(gy);
  Serial.print(" | Gyro Z: "); Serial.println(gz);

  delay(500);
}
