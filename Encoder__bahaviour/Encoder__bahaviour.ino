#define pwmB 23
#define in1B 26
#define in2B 27

// Encoder pins
#define ENCODER_A 18   // C1
#define ENCODER_B 19   // C2

volatile int encoderCount = 0;
int lastEncoded = 0;
int MotorSpeedB = 255;

void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(ENCODER_A); // MSB = most significant bit
  int LSB = digitalRead(ENCODER_B); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB;   // Convert to 2-bit value
  int sum = (lastEncoded << 2) | encoded; 

  // Check encoder state changes (Gray code)
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;

  lastEncoded = encoded;
}

void setup() {
  Serial.begin(115200);

  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);

  analogWrite(pwmB, 0); // initialize PWM
}

void loop() {
  // Run motor forward
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  analogWrite(pwmB, MotorSpeedB);

  // Print encoder count every 500 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);
    lastPrint = millis();
  }
}
