/*
 * ROS 2 Arduino Bridge for LidarBot
 * Hardware: Arduino Uno, L298N Driver, MPU6050, 2x DC Motors with Encoders
 * Communication: Serial (USB)
 * Control: Closed-Loop PID Velocity Control
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// --- PIN DEFINITIONS (Based on Gemini.md) ---
// Right Motor
#define R_ENCODER_A 3 // Yellow
#define R_ENCODER_B 5 // Green
#define R_PWM 11      // L298N ENB
#define R_IN3 12      // IN3
#define R_IN4 13      // IN4

// Left Motor
#define L_ENCODER_A 2 // Yellow
#define L_ENCODER_B 4 // Green
#define L_PWM 6       // L298N ENA
#define L_IN1 7       // IN1
#define L_IN2 8       // IN2

// --- CONSTANTS ---
const long BAUD_RATE = 115200;
const int COMMAND_TIMEOUT_MS = 1000;
const int PID_INTERVAL_MS = 20; // 50Hz PID loop

// --- PID GAINS (Tune these// -- PID Variables --
float Kp = 0.5; // Lower P to prevent overreaction
float Ki = 2.0; // Higher I to reach target
float Kd = 0.0;
float Kff = 0.2; // Feed-forward gain (approx 255 PWM / 1300 ticks/sec)

// --- STATE ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

long prev_left_ticks = 0;
long prev_right_ticks = 0;

// Target velocities (ticks per second)
float target_left_speed = 0.0;
float target_right_speed = 0.0;

// Actual velocities (ticks per second)
float actual_left_speed = 0.0;
float actual_right_speed = 0.0;

// PID Variables
float left_error_integral = 0;
float right_error_integral = 0;
float left_prev_error = 0;
float right_prev_error = 0;

// Current PWM outputs
int left_pwm_out = 0;
int right_pwm_out = 0;

unsigned long last_command_time = 0;
unsigned long last_pid_time = 0;
unsigned long last_telemetry_time = 0;

Adafruit_MPU6050 mpu;

// --- INTERRUPT HANDLERS ---
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
  Serial.begin(BAUD_RATE);
  delay(100);

  // Motor Pins
  pinMode(L_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  stopMotors();

  // Encoder Pins
  pinMode(L_ENCODER_A, INPUT_PULLUP);
  pinMode(L_ENCODER_B, INPUT_PULLUP);
  pinMode(R_ENCODER_A, INPUT_PULLUP);
  pinMode(R_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), doEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), doEncoderRight, CHANGE);

  // MPU6050
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void loop() {
  unsigned long current_time = millis();

  // 1. Read Commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseCommand(input);
    last_command_time = current_time;
  }

  // 2. Safety Timeout
  if (current_time - last_command_time > COMMAND_TIMEOUT_MS) {
    target_left_speed = 0;
    target_right_speed = 0;
  }

  // 3. PID Loop (50 Hz)
  if (current_time - last_pid_time >= PID_INTERVAL_MS) {
    calculatePID(current_time - last_pid_time);
    last_pid_time = current_time;
  }

  // 4. Send Telemetry to Python Bridge
  if (current_time - last_telemetry_time >= 50) {
    sendTelemetry();
    last_telemetry_time = current_time;
  }
}

void parseCommand(String input) {
  // Expected: "M,target_left_ticks_per_sec,target_right_ticks_per_sec"
  if (input.charAt(0) == 'M') {
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > 0) {
      target_left_speed =
          input.substring(firstComma + 1, secondComma).toFloat();
      target_right_speed = input.substring(secondComma + 1).toFloat();
    }
  }
}

void calculatePID(unsigned long dt_ms) {
  float dt_sec = dt_ms / 1000.0;

  // Atomically grab ticks
  noInterrupts();
  long current_left_ticks = left_ticks;
  long current_right_ticks = right_ticks;
  interrupts();

  // Calculate actual speeds (ticks / second)
  actual_left_speed = (current_left_ticks - prev_left_ticks) / dt_sec;
  actual_right_speed = (current_right_ticks - prev_right_ticks) / dt_sec;

  prev_left_ticks = current_left_ticks;
  prev_right_ticks = current_right_ticks;

  // -- LEFT MOTOR PID --
  float left_error = target_left_speed - actual_left_speed;
  left_error_integral += left_error * dt_sec;
  // Anti-windup
  left_error_integral = constrain(left_error_integral, -255.0 / Ki, 255.0 / Ki);
  float left_derivative = (left_error - left_prev_error) / dt_sec;
  float left_ff = target_left_speed * Kff;

  float left_out = left_ff + (Kp * left_error) + (Ki * left_error_integral) +
                   (Kd * left_derivative);

  // -- RIGHT MOTOR PID --
  float right_error = target_right_speed - actual_right_speed;
  right_error_integral += right_error * dt_sec;
  // Anti-windup
  right_error_integral =
      constrain(right_error_integral, -255.0 / Ki, 255.0 / Ki);
  float right_derivative = (right_error - right_prev_error) / dt_sec;
  float right_ff = target_right_speed * Kff;

  float right_out = right_ff + (Kp * right_error) +
                    (Ki * right_error_integral) + (Kd * right_derivative);

  // If target is perfectly 0, reset integral and force stop to prevent jitter
  if (target_left_speed == 0) {
    left_error_integral = 0;
    left_out = 0;
  }
  if (target_right_speed == 0) {
    right_error_integral = 0;
    right_out = 0;
  }

  // Set PWM directly (positional form PID)
  left_pwm_out = left_out;
  right_pwm_out = right_out;

  left_pwm_out = constrain(left_pwm_out, -255, 255);
  right_pwm_out = constrain(right_pwm_out, -255, 255);

  setMotorSpeeds(left_pwm_out, right_pwm_out);

  left_prev_error = left_error;
  right_prev_error = right_error;
}

void setMotorSpeeds(int left_in, int right_in) {
  // Hardware isolated test confirms: Positive PWM physically drives both motors
  // backward. We need positive input from ROS (forward) to produce negative
  // PWM.
  int left = -left_in;
  int right = -right_in;

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, left);
  } else if (left < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_PWM, -left);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_PWM, 0);
  }

  if (right > 0) {
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
    analogWrite(R_PWM, right);
  } else if (right < 0) {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
    analogWrite(R_PWM, -right);
  } else {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, LOW);
    analogWrite(R_PWM, 0);
  }
}

void stopMotors() {
  target_left_speed = 0;
  target_right_speed = 0;
  setMotorSpeeds(0, 0);
}

void sendTelemetry() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Send raw ticks so Python bridge can still calculate full odometry
  Serial.print("E,");
  Serial.print(left_ticks);
  Serial.print(",");
  Serial.print(right_ticks);
  Serial.print(",");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.print(",");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.println(g.gyro.z);
}
