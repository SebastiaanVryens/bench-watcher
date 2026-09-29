#include <ESP32Servo.h>

// =====================================================================
// Pins
// =====================================================================
const int SERVO_PIN = 5;
const int SERVO2_PIN = 15; // mirrors SERVO_PIN, opposite direction

// =====================================================================
// Servo
// =====================================================================
const int SERVO_CENTER_DEG = 90;

// Normal sweep
const int SERVO_SWEEP_MIN_DEG = -10;
const int SERVO_SWEEP_MAX_DEG = 90;
const int SERVO_SWEEP_UP_STEP_DEG = 3;
const int SERVO_SWEEP_DOWN_STEP_DEG = 4;
const unsigned long SERVO_STEP_INTERVAL_MS = 7;

// Fast alarm shake
const int SERVO_ALARM_MIN_DEG = -10;
const int SERVO_ALARM_MAX_DEG = 90;
const int SERVO_ALARM_STEP_DEG = 4;
const unsigned long SERVO_ALARM_STEP_INTERVAL_MS = 3;

// Alarm timing
const unsigned long ALARM_PERIOD_MS = 10000;  // alarm starts every 10 s
const unsigned long ALARM_DURATION_MS = 5000; // how long each alarm lasts

// =====================================================================
// Globals
// =====================================================================
Servo servo1;
Servo servo2;

enum ServoState { SWEEPING, ALARM };
ServoState state = SWEEPING;
unsigned long stateChangeTime = 0;

bool servoSweepingUp = true;
int servoPosDegrees = SERVO_SWEEP_MIN_DEG;
unsigned long lastServoStep = 0;

// =====================================================================
// Helpers
// =====================================================================
void writeMirroredServos(int angle) {
  angle = constrain(angle, 0, 180);
  servo1.write(angle);
  servo2.write(2 * SERVO_CENTER_DEG - angle);
}

// Shared bounce logic: move one step toward the current end, reverse at limits
void stepBounce(int minDeg, int maxDeg, int upStep, int downStep, unsigned long intervalMs) {
  unsigned long now = millis();
  if (now - lastServoStep < intervalMs) {
    return;
  }
  lastServoStep = now;

  if (servoSweepingUp) {
    servoPosDegrees = min(servoPosDegrees + upStep, maxDeg);
    if (servoPosDegrees >= maxDeg) {
      servoSweepingUp = false;
    }
  } else {
    servoPosDegrees = max(servoPosDegrees - downStep, minDeg);
    if (servoPosDegrees <= minDeg) {
      servoSweepingUp = true;
    }
  }

  writeMirroredServos(servoPosDegrees);
}

void stepServoSweep() {
  stepBounce(SERVO_SWEEP_MIN_DEG, SERVO_SWEEP_MAX_DEG,
             SERVO_SWEEP_UP_STEP_DEG, SERVO_SWEEP_DOWN_STEP_DEG,
             SERVO_STEP_INTERVAL_MS);
}

void stepServoAlarm() {
  stepBounce(SERVO_ALARM_MIN_DEG, SERVO_ALARM_MAX_DEG,
             SERVO_ALARM_STEP_DEG, SERVO_ALARM_STEP_DEG,
             SERVO_ALARM_STEP_INTERVAL_MS);
}

// =====================================================================
// Setup / Loop
// =====================================================================
void setup() {
  servo1.attach(SERVO_PIN);
  servo2.attach(SERVO2_PIN);
  writeMirroredServos(servoPosDegrees);
  stateChangeTime = millis();
}

void loop() {
  unsigned long now = millis();

  switch (state) {
    case SWEEPING:
      stepServoSweep();
      if (now - stateChangeTime >= ALARM_PERIOD_MS - ALARM_DURATION_MS) {
        state = ALARM;
        stateChangeTime = now;
      }
      break;

    case ALARM:
      stepServoAlarm();
      if (now - stateChangeTime >= ALARM_DURATION_MS) {
        state = SWEEPING;
        stateChangeTime = now;
      }
      break;
  }
}