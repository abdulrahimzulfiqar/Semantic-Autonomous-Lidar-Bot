// --- HARDWARE DIAGNOSTIC SCRIPT ---
// This disables ALL closed-loop PID and allows you to test raw motor power and
// encoder counting. Use the Arduino IDE Serial Monitor (115200 baud) to send
// commands. Send "L150" to push 150 power to Left motor. Send "L0" to stop.
// Send "R150" to push 150 power to Right motor. Send "R0" to stop.
// Send "L-150" to drive left backwards.

#define L_ENCODER_A 2
#define L_ENCODER_B 4
#define R_ENCODER_A 3
#define R_ENCODER_B 5

#define L_IN1 7
#define L_IN2 8
#define L_PWM 6
#define R_IN3 12
#define R_IN4 13
#define R_PWM 11

volatile long left_ticks = 0;
volatile long right_ticks = 0;

void doEncoderLeft() {
  if (digitalRead(L_ENCODER_A) == digitalRead(L_ENCODER_B)) {
    left_ticks--;
  } else {
    left_ticks++;
  }
}

void doEncoderRight() {
  if (digitalRead(R_ENCODER_A) == digitalRead(R_ENCODER_B)) {
    right_ticks++;
  } else {
    right_ticks--;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(L_ENCODER_A, INPUT_PULLUP);
  pinMode(L_ENCODER_B, INPUT_PULLUP);
  pinMode(R_ENCODER_A, INPUT_PULLUP);
  pinMode(R_ENCODER_B, INPUT_PULLUP);

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), doEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), doEncoderRight, RISING);

  Serial.println("===========================================");
  Serial.println("Hardware Diagnostics Ready.");
  Serial.println("Send 'L150' for Left Forward, 'L-150' for Left Backward.");
  Serial.println("Send 'L0' to Stop.");
  Serial.println("Send 'R150' for Right Forward, 'R-150' for Right Backward.");
  Serial.println("Send 'R0' to Stop.");
  Serial.println("===========================================");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0)
      return;

    char motor = cmd.charAt(0);
    int pwm = cmd.substring(1).toInt();

    if (motor == 'L' || motor == 'l') {
      setMotorL(pwm);
      Serial.print("Left Motor given Output: ");
      Serial.println(pwm);
    } else if (motor == 'R' || motor == 'r') {
      setMotorR(pwm);
      Serial.print("Right Motor given Output: ");
      Serial.println(pwm);
    } else if (motor == 'S' || motor == 's') {
      setMotorL(0);
      setMotorR(0);
      Serial.println("EMERGENCY STOP!");
    }
  }

  // Print telemetry every second
  static unsigned long last_print = 0;
  if (millis() - last_print > 1000) {
    last_print = millis();
    Serial.print("L_Ticks: ");
    Serial.print(left_ticks);
    Serial.print("\t|\tR_Ticks: ");
    Serial.println(right_ticks);
  }
}

void setMotorL(int pwm) {
  // Invert PWM: Positive command must produce negative actual PWM to drive
  // physically forward
  int out = constrain(-pwm, -255, 255);
  if (out > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, out);
  } else if (out < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_PWM, -out);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, 0);
  }
}

void setMotorR(int pwm) {
  // Invert PWM: Positive command must produce negative actual PWM to drive
  // physically forward
  int out = constrain(-pwm, -255, 255);
  if (out > 0) {
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
    analogWrite(R_PWM, out);
  } else if (out < 0) {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
    analogWrite(R_PWM, -out);
  } else {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, LOW);
    analogWrite(R_PWM, 0);
  }
}
