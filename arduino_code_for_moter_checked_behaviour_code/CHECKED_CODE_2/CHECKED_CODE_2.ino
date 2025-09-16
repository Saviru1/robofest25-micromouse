#define pwmB 23
#define in1B 26
#define in2B 27

int MotorSpeedB = 255;

void setup() {
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  analogWrite(pwmB, 0); // initialize PWM
}

void loop() {
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);

  analogWrite(pwmB, MotorSpeedB); // PWM to motor
  delay(3000);

  analogWrite(pwmB, 0);           // stop motor
  delay(1000);
}