

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  Serial.println("ESP32 + VL53L0X Test");

  // Initialize I2C
  Wire.begin(21, 22); // SDA=21, SCL=22

  if (!lox.begin()) {
    Serial.println("Failed to find VL53L0X sensor! Check wiring.");
    while (1);
  }
  Serial.println("VL53L0X detected!");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) { // 4 = out of range
    Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }

  delay(200);
}
