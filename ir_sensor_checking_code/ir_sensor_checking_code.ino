const int irSensorPin =34; // Example analog pin on ESP32 for IR sensor input
int sensorValue = 0;        // Variable to store sensor reading
int threshold = 1000;       // Threshold value to detect object (adjust as needed)

void setup() {
  Serial.begin(115200);
  pinMode(irSensorPin, INPUT);
}

void loop() {
  // Read the analog value from the IR sensor
  sensorValue = analogRead(irSensorPin);

  // Print the sensor value for debugging
  Serial.print("IR sensor analog value: ");
  Serial.println(sensorValue);

  // Detect presence if sensorValue crosses threshold
  if (sensorValue > threshold) {
    Serial.println("Object detected!");
  } else {
    Serial.println("No object detected.");
  }

  delay(200); // Small delay for stability
}
