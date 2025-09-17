#define LED_PIN 2   // D2 = GPIO2 on ESP32 Dev Module

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // LED ON
  delay(500);
  digitalWrite(LED_PIN, LOW);   // LED OFF
  delay(500);
}
