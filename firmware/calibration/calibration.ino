/*
 * ROBOT ENCODER CALIBRATOR
 * Upload this to count ticks for calibration.
 */

// PINS (Matched to your wiring)
#define LEFT_ENC_A 2
#define RIGHT_ENC_A 3

volatile long left_count = 0;
volatile long right_count = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftPulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightPulse, CHANGE);

  Serial.println("--- ENCODER CALIBRATOR ---");
  Serial.println("Spin the wheels manually!");
}

void loop() {
  static long last_print = 0;

  if (millis() - last_print > 200) {
    Serial.print("Left: ");
    Serial.print(left_count);
    Serial.print("   |   Right: ");
    Serial.println(right_count);
    last_print = millis();
  }
}

void leftPulse() {
  left_count++; // Just counting pulses, direction doesn't matter for this test
}

void rightPulse() { right_count++; }
