#include <Stepper.h>
#include <ESP32Servo.h>
#include <GardenSpine.h>
#include <math.h>

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
const float MIN_DETECT_DISTANCE = 150; //TODO: implement when someone stands in front to joke, that it doesn't do anything. Or something else.

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
const unsigned long FORWARD_DURATION_MS = 200000; //2 min
const unsigned long BACKWARD_DURATION_MS = 200000;
const unsigned long DIRECTION_BREAK_MS = 3000;

const float ROBOT_SPEED_CM_PER_S = 3.0f;

const float TRACK_LENGTH_CM = (FORWARD_DURATION_MS / 1000.0f) * ROBOT_SPEED_CM_PER_S;

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
const unsigned long STOP_DURATION_MS = 5000;
const unsigned long COOLDOWN_DURATION_MS = 5000;

// =====================================================================
// Live segmentation (per-person / per-object detection via the sweep)
// =====================================================================
const float MIN_SEGMENT_WIDTH_CM = 15.0f;

const float CONFIRM_TRIGGER_WIDTH_CM = 25.0f;

const unsigned long MAX_SEGMENT_GAP_MS = 400;

const unsigned long MAX_SEGMENT_DURATION_MS = 90000;

// =====================================================================
// Bench occupancy tracking
// =====================================================================
const int MAX_TRACKED_OCCUPANTS = 10;

const float POSITION_MATCH_TOLERANCE_CM = 20.0f;

const unsigned long DEPARTURE_CONFIRM_MS = 5000;

const unsigned long REPORT_INTERVAL_MS = 30000;

enum ObjectSize { SIZE_SMALL, SIZE_MEDIUM, SIZE_LARGE };
const float SMALL_OBJECT_MAX_WIDTH_CM = 25.0f;   // e.g. a bag or a narrow object
const float MEDIUM_OBJECT_MAX_WIDTH_CM = 65.0f;  // e.g. one seated person

ObjectSize classifyWidth(float widthCm) {
  if (widthCm <= SMALL_OBJECT_MAX_WIDTH_CM) return SIZE_SMALL;
  if (widthCm <= MEDIUM_OBJECT_MAX_WIDTH_CM) return SIZE_MEDIUM;
  return SIZE_LARGE;
}

const char* objectSizeLabel(ObjectSize size) {
  switch (size) {
    case SIZE_SMALL: return "small";
    case SIZE_MEDIUM: return "medium";
    default: return "large";
  }
}

const float PERSON_MIN_WIDTH_CM = 40.0f;
const float PERSON_TYPICAL_WIDTH_CM = 50.0f; // nominal shoulder-width-ish seated footprint
const int MAX_ESTIMATED_PEOPLE_PER_CLUSTER = 6; // sanity cap against runaway/noisy widths

int estimatePersonCount(float widthCm) {
  if (widthCm < PERSON_MIN_WIDTH_CM) {
    return 0;
  }
  int count = (int)round(widthCm / PERSON_TYPICAL_WIDTH_CM);
  if (count < 1) count = 1;
  if (count > MAX_ESTIMATED_PEOPLE_PER_CLUSTER) count = MAX_ESTIMATED_PEOPLE_PER_CLUSTER;
  return count;
}

enum PresenceType { TYPE_OBJECT, TYPE_PERSON };

PresenceType classifyPresenceType(int personCount) {
  return (personCount > 0) ? TYPE_PERSON : TYPE_OBJECT;
}

const char* presenceTypeLabel(PresenceType type) {
  return (type == TYPE_PERSON) ? "person" : "object";
}

struct BenchOccupant {
  bool active;
  float centerPositionCm;
  float widthCm;
  ObjectSize size;
  PresenceType presenceType;
  int personCount;              // 0 for objects; 1, 2, 3... for (usually single) person segments
  unsigned long firstSeenMs;
  unsigned long lastSeenMs;
  unsigned long clearSinceMs; 
};

BenchOccupant occupants[MAX_TRACKED_OCCUPANTS];

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

// Running estimate of the robot's position along the track (cm).
float robotPositionCm = 0.0f;
unsigned long lastPositionUpdateMs = 0;

// --- Live segmentation state ---
bool segmentActive = false;
bool segmentTriggered = false;
int segmentOccupantIdx = -1;
float segmentStartPositionCm = 0.0f;
float segmentEndPositionCm = 0.0f;
unsigned long segmentStartMs = 0;
unsigned long segmentLastBelowMs = 0;

bool pendingNewOccupantAlarm = false;

GardenSpine spine;
unsigned long lastReportMs = 0;

// RUNNING  -> rotating normally, servo sweeping, sweep continuously
//             segmented into individual detections in the background
// STOPPED  -> a brand-new occupant was just registered; motor paused,
//             servo shakes fast, for STOP_DURATION_MS
// COOLDOWN -> resumed rotating, but ignoring new-occupant alarms for
//             COOLDOWN_DURATION_MS so it doesn't immediately re-trigger
enum MotorState { RUNNING, STOPPED, COOLDOWN };
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

  spine.begin();
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

// ---------------------------------------------------------------------
// Robot position tracking
// ---------------------------------------------------------------------
void updateRobotPosition() {
  unsigned long now = millis();
  if (lastPositionUpdateMs == 0) {
    lastPositionUpdateMs = now;
    return;
  }

  float elapsedS = (now - lastPositionUpdateMs) / 1000.0f;
  lastPositionUpdateMs = now;

  if (motionPhase == FORWARD) {
    robotPositionCm += ROBOT_SPEED_CM_PER_S * elapsedS;
  } else if (motionPhase == BACKWARD) {
    robotPositionCm -= ROBOT_SPEED_CM_PER_S * elapsedS;
  }

  robotPositionCm = constrain(robotPositionCm, 0.0f, TRACK_LENGTH_CM);
}

// ---------------------------------------------------------------------
// Bench occupancy tracking
// ---------------------------------------------------------------------
int findOccupantNear(float positionCm) {
  for (int i = 0; i < MAX_TRACKED_OCCUPANTS; i++) {
    if (!occupants[i].active) continue;
    float halfSpan = occupants[i].widthCm / 2.0f + POSITION_MATCH_TOLERANCE_CM;
    if (fabs(occupants[i].centerPositionCm - positionCm) <= halfSpan) {
      return i;
    }
  }
  return -1;
}

int findFreeOccupantSlot() {
  for (int i = 0; i < MAX_TRACKED_OCCUPANTS; i++) {
    if (!occupants[i].active) {
      return i;
    }
  }
  return -1;
}

void logPersonCountChange(int idx, int previousPersonCount, unsigned long now) {
  if (occupants[idx].personCount < previousPersonCount) {
    int leftCount = previousPersonCount - occupants[idx].personCount;
    unsigned long clusterSatMs = now - occupants[idx].firstSeenMs;
    Serial.print(leftCount);
    Serial.print(" of ");
    Serial.print(previousPersonCount);
    Serial.print(" left the segment at ");
    Serial.print(occupants[idx].centerPositionCm);
    Serial.print(" cm, present for an estimated ");
    Serial.print(clusterSatMs / 1000);
    Serial.println(" s so far");
  } else if (occupants[idx].personCount > previousPersonCount) {
    Serial.print(occupants[idx].personCount - previousPersonCount);
    Serial.print(" more joined the segment at ");
    Serial.print(occupants[idx].centerPositionCm);
    Serial.println(" cm");
  }
}

int registerDetection(float centerPositionCm, float widthCm, bool &outWasNew) {
  unsigned long now = millis();
  int idx = findOccupantNear(centerPositionCm);
  outWasNew = (idx == -1);

  if (idx == -1) {
    idx = findFreeOccupantSlot();
    if (idx == -1) {
      Serial.println("Occupant table full, ignoring new detection.");
      outWasNew = false;
      return -1;
    }
    occupants[idx].active = true;
    occupants[idx].centerPositionCm = centerPositionCm;
    occupants[idx].widthCm = widthCm;
    occupants[idx].size = classifyWidth(widthCm);
    occupants[idx].personCount = estimatePersonCount(widthCm);
    occupants[idx].presenceType = classifyPresenceType(occupants[idx].personCount);
    occupants[idx].firstSeenMs = now;
    Serial.print("New occupant tracked at ");
    Serial.print(centerPositionCm);
    Serial.print(" cm, width ");
    Serial.print(widthCm);
    Serial.print(" cm (");
    Serial.print(objectSizeLabel(occupants[idx].size));
    Serial.print(", ");
    Serial.print(presenceTypeLabel(occupants[idx].presenceType));
    if (occupants[idx].personCount > 1) {
      Serial.print(" x");
      Serial.print(occupants[idx].personCount);
    }
    Serial.println(")");
  } else {
    int previousPersonCount = occupants[idx].personCount;

    occupants[idx].centerPositionCm = (occupants[idx].centerPositionCm + centerPositionCm) / 2.0f;
    occupants[idx].widthCm = (occupants[idx].widthCm + widthCm) / 2.0f;
    occupants[idx].size = classifyWidth(occupants[idx].widthCm);
    occupants[idx].personCount = estimatePersonCount(occupants[idx].widthCm);
    occupants[idx].presenceType = classifyPresenceType(occupants[idx].personCount);

    logPersonCountChange(idx, previousPersonCount, now);
  }

  occupants[idx].lastSeenMs = now;
  occupants[idx].clearSinceMs = 0; // seen again, so this spot isn't "clearing"
  return idx;
}

void updateOccupantLiveMeasurement(int idx, float centerPositionCm, float widthCm) {
  int previousPersonCount = occupants[idx].personCount;

  occupants[idx].centerPositionCm = centerPositionCm;
  occupants[idx].widthCm = widthCm;
  occupants[idx].size = classifyWidth(widthCm);
  occupants[idx].personCount = estimatePersonCount(widthCm);
  occupants[idx].presenceType = classifyPresenceType(occupants[idx].personCount);
  occupants[idx].lastSeenMs = millis();
  occupants[idx].clearSinceMs = 0;

  logPersonCountChange(idx, previousPersonCount, occupants[idx].lastSeenMs);
}

void checkForDepartures(float currentPositionCm) {
  unsigned long now = millis();

  for (int i = 0; i < MAX_TRACKED_OCCUPANTS; i++) {
    if (!occupants[i].active) continue;
    float halfSpan = occupants[i].widthCm / 2.0f + POSITION_MATCH_TOLERANCE_CM;
    if (fabs(occupants[i].centerPositionCm - currentPositionCm) > halfSpan) continue;

    if (occupants[i].clearSinceMs == 0) {
      occupants[i].clearSinceMs = now;
    } else if (now - occupants[i].clearSinceMs >= DEPARTURE_CONFIRM_MS) {
      unsigned long satForMs = occupants[i].lastSeenMs - occupants[i].firstSeenMs;
      Serial.print("Occupant at ");
      Serial.print(occupants[i].centerPositionCm);
      Serial.print(" cm (");
      Serial.print(objectSizeLabel(occupants[i].size));
      Serial.print(", ");
      Serial.print(presenceTypeLabel(occupants[i].presenceType));
      if (occupants[i].personCount > 1) {
        Serial.print(" x");
        Serial.print(occupants[i].personCount);
      }
      Serial.print(") left after sitting for ");
      Serial.print(satForMs / 1000);
      Serial.println(" s");
      occupants[i].active = false;
    }
  }
}

void computeOccupancyStats(int &humanCountOut, float &avgHumanSitSecondsOut) {
  unsigned long now = millis();
  int humanCount = 0;
  unsigned long totalSitMsWeighted = 0;

  for (int i = 0; i < MAX_TRACKED_OCCUPANTS; i++) {
    if (!occupants[i].active) continue;
    if (occupants[i].presenceType != TYPE_PERSON) continue;

    int n = occupants[i].personCount;
    unsigned long sitMs = now - occupants[i].firstSeenMs;

    humanCount += n;
    totalSitMsWeighted += sitMs * (unsigned long)n;
  }

  humanCountOut = humanCount;
  avgHumanSitSecondsOut = (humanCount > 0) ? (totalSitMsWeighted / 1000.0f) / humanCount : 0.0f;
}

// Publishes the current human count and average sit time to the server.
void reportOccupancy() {
  if (!spine.connected()) {
    return;
  }

  int humanCount;
  float avgSitSeconds;
  computeOccupancyStats(humanCount, avgSitSeconds);

  spine.publish("bench_occupant_count", humanCount, "people");
  spine.publish("bench_avg_sit_time", avgSitSeconds, "seconds");

  Serial.print("Reported occupancy: ");
  Serial.print(humanCount);
  Serial.print(" people, avg sit time ");
  Serial.print(avgSitSeconds);
  Serial.println(" s");
}

// ---------------------------------------------------------------------
// Live segmentation
// ---------------------------------------------------------------------
void finalizeSegment() {
  float widthCm = fabs(segmentEndPositionCm - segmentStartPositionCm);
  float centerPositionCm = (segmentStartPositionCm + segmentEndPositionCm) / 2.0f;

  if (segmentTriggered && segmentOccupantIdx != -1) {
    updateOccupantLiveMeasurement(segmentOccupantIdx, centerPositionCm, widthCm);
  } else if (widthCm >= MIN_SEGMENT_WIDTH_CM) {
    bool wasNew = false;
    registerDetection(centerPositionCm, widthCm, wasNew);
  }

  segmentActive = false;
  segmentTriggered = false;
  segmentOccupantIdx = -1;
}

void updateSegmentation(bool belowThreshold, float positionCm) {
  unsigned long now = millis();

  if (belowThreshold) {
    if (!segmentActive) {
      segmentActive = true;
      segmentTriggered = false;
      segmentOccupantIdx = -1;
      segmentStartPositionCm = positionCm;
      segmentStartMs = now;
    }
    segmentEndPositionCm = positionCm;
    segmentLastBelowMs = now;

    if (now - segmentStartMs >= MAX_SEGMENT_DURATION_MS) {
      finalizeSegment();
      return; // if still below threshold next sample, a fresh run starts naturally
    }

    float widthSoFarCm = fabs(segmentEndPositionCm - segmentStartPositionCm);

    if (!segmentTriggered && widthSoFarCm >= CONFIRM_TRIGGER_WIDTH_CM) {
      segmentTriggered = true;
      bool wasNew = false;
      segmentOccupantIdx = registerDetection(
        (segmentStartPositionCm + segmentEndPositionCm) / 2.0f,
        widthSoFarCm,
        wasNew
      );
      if (wasNew) {
        pendingNewOccupantAlarm = true;
      }
    } else if (segmentTriggered && segmentOccupantIdx != -1) {
      updateOccupantLiveMeasurement(
        segmentOccupantIdx,
        (segmentStartPositionCm + segmentEndPositionCm) / 2.0f,
        widthSoFarCm
      );
    }
  } else if (segmentActive) {
    if (now - segmentLastBelowMs >= MAX_SEGMENT_GAP_MS) {
      finalizeSegment();
    }
  }
}

// ---------------------------------------------------------------------
// Motion / servo
// ---------------------------------------------------------------------
void driveMotors() {
  updateRobotPosition();

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
  spine.loop();

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

    updateSegmentation(belowThreshold, robotPositionCm);

    if (!belowThreshold) {
      checkForDepartures(robotPositionCm);
    }
  }

  // --- State machine ---
  switch (state) {
    case RUNNING:
      driveMotors();
      stepServoSweep();
      if (pendingNewOccupantAlarm) {
        pendingNewOccupantAlarm = false;
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
        lastPositionUpdateMs = millis();
      }
      break;

    case COOLDOWN:
      driveMotors();
      stepServoSweep();
      pendingNewOccupantAlarm = false;
      if (millis() - stateChangeTime >= COOLDOWN_DURATION_MS) {
        state = RUNNING;
      }
      break;
  }

  // --- Periodic occupancy report to the server ---
  if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = millis();
    reportOccupancy();
  }
}
