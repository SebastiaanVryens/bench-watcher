#include <Stepper.h>
#include <ESP32Servo.h>

// =====================================================================
// Pins
// =====================================================================
const int TRIGGER_PIN = 2;
const int ECHO_PIN = 4;
const int SERVO_PIN = 5;
const int SERVO2_PIN = 15; // mirrors SERVO_PIN, moves in the opposite direction

// Stepper driver inputs, in physical wiring order (IN1, IN2, IN3, IN4)
const int MOTOR_PIN_IN1 = 26;
const int MOTOR_PIN_IN2 = 27;
const int MOTOR_PIN_IN3 = 14;
const int MOTOR_PIN_IN4 = 25;

// Second stepper (opposite direction)
const int MOTOR2_PIN_IN1 = 23;
const int MOTOR2_PIN_IN2 = 22;
const int MOTOR2_PIN_IN3 = 21;
const int MOTOR2_PIN_IN4 = 19;

// =====================================================================
// HC-SR04 Ultrasonic Sensor
// =====================================================================
const unsigned long SAMPLE_MS = 50;
const unsigned long TRIGGER_SETTLE_US = 2;
const unsigned long TRIGGER_PULSE_US = 10;
const unsigned long ECHO_TIMEOUT_US = 30000;
const float SPEED_OF_SOUND_CM_PER_US = 0.0343;

const float DETECT_DISTANCE_CM = 200; //aka bench distance

// Serial Plotter display range (debug purposes)
const int PLOT_MIN_CM = 0;
const int PLOT_MAX_CM = 450;

// =====================================================================
// 28BYJ-48 Stepper
// =====================================================================
const int STEPS_PER_REV = 2048; // full-step resolution for 28BYJ-48 with Stepper.h
const int SPEED_RPM = 15; // is around the max
const int STEPS = 50; //around the max with current wheels

// Forward/backward motion cycle
const unsigned long FORWARD_DURATION_MS = 500000; //5 min
const unsigned long BACKWARD_DURATION_MS = 500000;
const unsigned long DIRECTION_BREAK_MS = 3000;

// =====================================================================
// Servo
// =====================================================================
const int SERVO_CENTER_DEG = 90;

const int SERVO_SWEEP_MIN_DEG = -10;
const int SERVO_SWEEP_MAX_DEG = 90;
const int SERVO_SWEEP_UP_STEP_DEG = 5;
const int SERVO_SWEEP_DOWN_STEP_DEG = 4;
const unsigned long SERVO_STEP_INTERVAL_MS = 7;

// Fast "alarm" shake
const int SERVO_ALARM_MIN_DEG = -10;
const int SERVO_ALARM_MAX_DEG = 90;
const unsigned long SERVO_ALARM_INTERVAL_MS = 250;
const int SERVO_ALARM_STEP_DEG = 6;
const unsigned long SERVO_ALARM_STEP_INTERVAL_MS = 3;

// =====================================================================
// State machine timing
// =====================================================================
const unsigned long CONFIRM_DURATION_MS = 1000;
const unsigned long STOP_DURATION_MS = 5000;
const unsigned long COOLDOWN_DURATION_MS = 5000;

// =====================================================================
// Globals
// =====================================================================
unsigned long lastReading = 0;

Stepper motor(STEPS_PER_REV, MOTOR_PIN_IN1, MOTOR_PIN_IN3, MOTOR_PIN_IN2, MOTOR_PIN_IN4);
Stepper motor2(STEPS_PER_REV, MOTOR2_PIN_IN1, MOTOR2_PIN_IN3, MOTOR2_PIN_IN2, MOTOR2_PIN_IN4);

Servo servo1;
Servo servo2; // mirrors servo1, opposite direction
bool servoSweepingUp = true;
int servoPosDegrees = SERVO_SWEEP_MIN_DEG;
unsigned long lastServoStep = 0;

bool servoAlarmToggle = false;
bool servoAlarmInitialized = false;
unsigned long lastServoAlarmStep = 0;

// RUNNING    -> rotating normally, servo sweeping
// CONFIRMING -> distance just dropped below threshold; motor + servo KEEP
//               moving normally while we wait CONFIRM_DURATION_MS to see if
//               it's a real detection
// STOPPED    -> confirmed detection; motor paused, servo shakes fast, for
//               STOP_DURATION_MS
// COOLDOWN   -> resumed rotating, but ignoring new detections for
//               COOLDOWN_DURATION_MS so it doesn't immediately re-trigger
enum MotorState { RUNNING, CONFIRMING, STOPPED, COOLDOWN };
MotorState state = RUNNING;
unsigned long stateChangeTime = 0;

enum MotionPhase { FORWARD, BREAK_AFTER_FORWARD, BACKWARD, BREAK_AFTER_BACKWARD };
MotionPhase motionPhase = FORWARD;
unsigned long motionPhaseChangeTime = 0;

void setup() {
  Serial.begin(115200);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  motor.setSpeed(SPEED_RPM);
  motor2.setSpeed(SPEED_RPM);

  servo1.attach(SERVO_PIN);
  servo2.attach(SERVO2_PIN);

  motionPhaseChangeTime = millis();
}

float readDistanceCm() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(TRIGGER_SETTLE_US);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(TRIGGER_PULSE_US);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return -1;
  }
  return duration * SPEED_OF_SOUND_CM_PER_US / 2.0;
}

void driveMotors() {
  unsigned long now = millis();

  switch (motionPhase) {
    case FORWARD:
      motor.step(STEPS);
      motor2.step(-STEPS);
      if (now - motionPhaseChangeTime >= FORWARD_DURATION_MS) {
        motionPhase = BREAK_AFTER_FORWARD;
        motionPhaseChangeTime = now;
      }
      break;

    case BREAK_AFTER_FORWARD:
      if (now - motionPhaseChangeTime >= DIRECTION_BREAK_MS) {
        motionPhase = BACKWARD;
        motionPhaseChangeTime = now;
      }
      break;

    case BACKWARD:
      motor.step(-STEPS);
      motor2.step(STEPS);
      if (now - motionPhaseChangeTime >= BACKWARD_DURATION_MS) {
        motionPhase = BREAK_AFTER_BACKWARD;
        motionPhaseChangeTime = now;
      }
      break;

    case BREAK_AFTER_BACKWARD:
      if (now - motionPhaseChangeTime >= DIRECTION_BREAK_MS) {
        motionPhase = FORWARD;
        motionPhaseChangeTime = now;
      }
      break;
  }
}

void writeMirroredServos(int angle) {
  angle = constrain(angle, 0, 180);
  servo1.write(angle);
  servo2.write(2 * SERVO_CENTER_DEG - angle);
}

void stepServoSweep() {
  if (millis() - lastServoStep < SERVO_STEP_INTERVAL_MS) {
    return;
  }
  lastServoStep = millis();

  if (servoSweepingUp) {
    if (servoPosDegrees <= SERVO_SWEEP_MAX_DEG) {
      servoPosDegrees += SERVO_SWEEP_UP_STEP_DEG;
      int writeAngle = constrain(servoPosDegrees, SERVO_SWEEP_MIN_DEG, SERVO_SWEEP_MAX_DEG);
      writeMirroredServos(writeAngle);
      Serial.println(servoPosDegrees);
    } else {
      servoSweepingUp = false;
      servoPosDegrees = SERVO_SWEEP_MAX_DEG;
    }
  } else {
    if (servoPosDegrees >= SERVO_SWEEP_MIN_DEG) {
      servoPosDegrees -= SERVO_SWEEP_DOWN_STEP_DEG;
      int writeAngle = constrain(servoPosDegrees, SERVO_SWEEP_MIN_DEG, SERVO_SWEEP_MAX_DEG);
      writeMirroredServos(writeAngle);
      Serial.println(servoPosDegrees);
    } else {
      servoSweepingUp = true;
      servoPosDegrees = SERVO_SWEEP_MIN_DEG;
    }
  }
}

void stepServoAlarm() {
  if (!servoAlarmInitialized) {
    servoSweepingUp = servoSweepingUp;
    servoAlarmInitialized = true;
    lastServoAlarmStep = millis();
    return;
  }

  if (millis() - lastServoAlarmStep < SERVO_ALARM_STEP_INTERVAL_MS) {
    return;
  }
  lastServoAlarmStep = millis();

  if (servoSweepingUp) {
    servoPosDegrees += SERVO_ALARM_STEP_DEG;
    if (servoPosDegrees >= SERVO_ALARM_MAX_DEG) {
      servoPosDegrees = SERVO_ALARM_MAX_DEG;
      servoSweepingUp = false;
    }
  } else {
    servoPosDegrees -= SERVO_ALARM_STEP_DEG;
    if (servoPosDegrees <= SERVO_ALARM_MIN_DEG) {
      servoPosDegrees = SERVO_ALARM_MIN_DEG;
      servoSweepingUp = true;
    }
  }

  writeMirroredServos(servoPosDegrees);
}

void loop() {
  // --- Sensor sampling (non-blocking, every SAMPLE_MS) ---
  if (millis() - lastReading >= SAMPLE_MS) {
    lastReading = millis();

    float distance = readDistanceCm();
    bool belowThreshold = (distance > 0 && distance < DETECT_DISTANCE_CM);

    Serial.print("Zero:");
    Serial.print(PLOT_MIN_CM);
    Serial.print("\tMax:");
    Serial.print(PLOT_MAX_CM);
    Serial.print("\tDistance:");
    Serial.println(distance);

    if (state == RUNNING && belowThreshold) {
      state = CONFIRMING;
      stateChangeTime = millis();
    } else if (state == CONFIRMING && !belowThreshold) {
      state = RUNNING;
    }
  }

  // --- State machine ---
  switch (state) {
    case RUNNING:
      driveMotors();
      stepServoSweep();
      break;

    case CONFIRMING:
      driveMotors();
      stepServoSweep();
      if (millis() - stateChangeTime >= CONFIRM_DURATION_MS) {
        state = STOPPED;
        stateChangeTime = millis();
        servoAlarmInitialized = false;
      }
      break;

    case STOPPED:
      stepServoAlarm();
      if (millis() - stateChangeTime >= STOP_DURATION_MS) {
        state = COOLDOWN;
        stateChangeTime = millis();
      }
      break;

    case COOLDOWN:
      driveMotors();
      stepServoSweep();
      if (millis() - stateChangeTime >= COOLDOWN_DURATION_MS) {
        state = RUNNING;
      }
      break;
  }
}
