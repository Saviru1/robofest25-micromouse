// ---------------- MOTOR B ----------------
#define pwmB 26
#define in1B 25
#define in2B 27

// ---------------- MOTOR A ----------------
#define pwmA 32
#define in1A 33
#define in2A 15

// ---------------- ENCODERS ----------------
// Motor A encoder
#define ENCODER_A1 18   // C1 of motor A
#define ENCODER_A2 34   // C2 of motor A

// Motor B encoder
#define ENCODER_B1 19   // C1 of motor B
#define ENCODER_B2 35   // C2 of motor B

// ---------------- VARIABLES ----------------
volatile int encoderCountA = 0;
volatile int encoderCountB = 0;

int lastEncodedA = 0;
int lastEncodedB = 0;

int MotorSpeedA = 200;  // Adjust as needed
int MotorSpeedB = 200;  // Adjust as needed

// ---------------- INTERRUPT HANDLERS ----------------
void IRAM_ATTR updateEncoderA() {
  int MSB = digitalRead(ENCODER_A1);
  int LSB = digitalRead(ENCODER_A2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedA << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountA++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountA--;

  lastEncodedA = encoded;
}

void IRAM_ATTR updateEncoderB() {
  int MSB = digitalRead(ENCODER_B1);
  int LSB = digitalRead(ENCODER_B2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedB << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountB++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountB--;

  lastEncodedB = encoded;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  // Motor A pins
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  // Motor B pins
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Encoder A pins
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);

  // Encoder B pins
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B2), updateEncoderB, CHANGE);

  analogWrite(pwmA, 0);
  analogWrite(pwmB, 0);
}

// ---------------- LOOP ----------------
void loop() {
  // Run Motor A forward
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  analogWrite(pwmA, MotorSpeedA);

  // Run Motor B forward
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  analogWrite(pwmB, MotorSpeedB);

  // Print encoder counts every 500 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Motor A Encoder: ");
    Serial.print(encoderCountA);
    Serial.print(" | Motor B Encoder: ");
    Serial.println(encoderCountB);
    lastPrint = millis();
  }
}
 