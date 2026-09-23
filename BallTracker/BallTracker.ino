#include "ColorSensor.h"
#include "IRRing.h"
#include "RobotCompass.h"
#include "RobotMotors.h"
#include <Arduino.h>
#include <math.h>

// Instantiate the subsystems
IRRing irRing;
RobotCompass compass;
RobotMotors robot;
ColorSensor colorSensor;

float initialHeading = 0.0;
bool isCalibrated = false;

// ============================================================
//  White-line backup state
// ============================================================
const unsigned long COLOR_BACKUP_MS = 500;   // How long the robot drives backward after hitting a white line (ms).
const unsigned long COLOR_COOLDOWN_MS = 600; // Time to wait after a backup before sensors are checked again (ms).
static bool inColorBackup = false;           // State flag: true when the robot is currently in the "backing up" phase.
static unsigned long colorBackupStart = 0;   // Timestamp (millis) of when the current backup started.
static float colorBackupAngle = 0.0f;        // The specific angle the robot is moving to avoid the line.
static unsigned long colorBackupEnd = 0;     // Timestamp (millis) of when the last backup finished; used for cooldown.
static unsigned long colorFirstTrigger = 0;   // Timestamp of initial detection, used for hard safety timeout.

// ============================================================
//  ---- TEST MODE ----
//  Set to 0 for NORMAL mode (ball-tracking + colour avoidance).
//  Set to 1 for ANGLE-ONLY mode (prints ball angle, NO movement).
//  Set to 2 for MOTOR TEST (drives N, E, S, W in sequence, no IR).
//  Set to 3 for CONSTANT EAST DRIVE (drives 90 deg constantly for tuning).
//  Set to 4 for BALL TRACKING working code (Ignores all colour data).
//  Set to 5 for COLOUR SENSOR TEST (avoidance only, prints raw readings).
// ============================================================
const int TEST_MODE = 4;

// ============================================================
//  ---- SPEED & POWER TUNING ----
//  basePursueSpeed: The target PWM power for chasing the ball.
//    - Value Range: 0 to 255.
//    - Current Value: 45 (Safe indoor walk-speed).
//    - Competition: 150-200 (Aggressive play).
//    - Performance Note: At speeds > 150, you may need to reduce
//      kP_Rotation (e.g. 0.4) to maintain smooth movement.
// ============================================================
int basePursueSpeed = 180;           // The target translation power (PWM) for normal ball chasing.
const float kP_Rotation = 0.5f;      // Proportional gain for compass correction. Higher = snappier rotation.
const float HEADING_DEADZONE = 7.0f; // Compass error threshold (degrees) before rotation correction kicks in.
const float ANGLE_SMOOTH_ALPHA = 0.15f; // Alpha for low-pass angle filter. Lower = smoother/slower, Higher = twitchier/faster.
const float MIN_IR_STRENGTH = 15.0f; // Minimum IR signal strength required to consider the ball "visible".
const float MAX_ANGLE_JUMP = 120.0f; // Max degrees the ball angle can move in one frame before being ignored (noise filter).

// ============================================================

// Persistent angle smoother state (unit vector components)
float smoothX = 1.0f; // The X-component (cos) of our smoothed ball direction vector.
float smoothY = 0.0f; // The Y-component (sin) of our smoothed ball direction vector.

void setup() {
  Serial.begin(115200);
  delay(1000);

  irRing.begin();
  robot.begin();

  // Colour sensors — Wire1 (SCL=pin16, SDA=pin17) via TCA9548A @ 0x70
  colorSensor.init();

  if (!compass.begin()) {
    Serial.println("Ooops, no BNO055 detected!");
  } else {
    Serial.println("Compass settling...");
    delay(400);
    initialHeading = compass.getHeading();
    isCalibrated = true;
  }

  Serial.print("Goal Heading Locked at: ");
  Serial.println(initialHeading);

  if (TEST_MODE == 1) {
    Serial.println("=== ANGLE-ONLY MODE: Robot will NOT move. ===");
    Serial.println("Move the ball around and check angles in Serial Monitor.");
  } else if (TEST_MODE == 2) {
    Serial.println("=== MOTOR TEST MODE ===");
    Serial.println("Robot will drive in 4 directions at low speed.");
    Serial.println("Watch which PHYSICAL direction it moves for each label.");
    delay(2000);
  } else if (TEST_MODE == 3) {
    Serial.println("=== CONSTANT EAST DRIVE MODE ===");
    Serial.println("Robot will drive at angle 90 (East) constantly.");
    delay(1000);
  } else if (TEST_MODE == 4) {
    Serial.println("=== BALL-ONLY MODE: No colour sensor. ===");
    Serial.println("Original ball-tracking behaviour, colour sensor ignored.");
  } else if (TEST_MODE == 5) {
    Serial.println("=== COLOUR SENSOR TEST MODE ===");
    Serial.println("Robot brakes until white is detected, then backs away.");
    Serial.println("Raw sensor readings printed every 200 ms.");
  }
}

// ============================================================
// MOTOR TEST MODE (TEST_MODE == 2)
// Drives in each cardinal direction for 2 seconds.
// YOU observe which physical direction the robot moves and report back.
// ============================================================
void motorTestLoop() {
  struct TestStep {
    const char *label;
    float angle;
  };

  // These are the angles we INTEND to drive.
  // Report which PHYSICAL direction the robot actually moves.
  TestStep steps[] = {
      {"NORTH (Forward)", 0},
      {"EAST  (Right)", 90},
      {"SOUTH (Back)", 180},
      {"WEST  (Left)", 270},
  };

  for (int i = 0; i < 4; i++) {
    Serial.print(">>> Driving: ");
    Serial.print(steps[i].label);
    Serial.print(" (angle=");
    Serial.print(steps[i].angle);
    Serial.println(")");

    unsigned long start = millis();
    while (millis() - start < 2000) {
      robot.drive(steps[i].angle, 40, 0); // speed=40, no turning
      delay(15);
    }

    // Stop for 1.5 seconds between each direction
    robot.brake();
    Serial.println("--- STOPPED ---");
    delay(1500);
  }

  Serial.println("=== MOTOR TEST COMPLETE ===");
  Serial.println(
      "Please report which physical direction the robot moved for each label.");
  Serial.println("Example: 'NORTH command -> robot moved WEST'");

  // Halt forever after test
  robot.brake();
  while (true) {
    delay(1000);
  }
}

// ============================================================
// ANGLE-ONLY MODE (TEST_MODE == 1)
// Prints ball angle + MaxIdx but does NOT drive motors.
// ============================================================
void angleOnlyLoop() {
  irRing.update();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    if (irRing.isBallDetected()) {
      Serial.print("Ball:");
      Serial.print(irRing.getBallAngle(), 0);
      Serial.print(" Str:");
      Serial.print(irRing.getBallStrength(), 0);
      Serial.print(" MaxIdx:");
      Serial.println(irRing.getStrongestSensor());
    } else {
      Serial.println("No ball detected");
    }
  }
}

// ============================================================
// NORMAL BALL-TRACKING MODE (TEST_MODE == 0)
// ============================================================
void normalLoop() {
  irRing.update();

  // ---- 1. Compass Heading Hold (always computed, used in all states) ----
  float turnRate = 0;
  if (isCalibrated) {
    float currentHeading = compass.getHeading();
    float error = initialHeading - currentHeading;
    if (error > 180.0f)
      error -= 360.0f;
    if (error < -180.0f)
      error += 360.0f;
    if (fabsf(error) > HEADING_DEADZONE) {
      turnRate = kP_Rotation * error;
      turnRate = constrain(turnRate, -80, 80);
    }
  }

  // ---- 2. White-Line Detection (overrides ball tracking) ----
  if (!inColorBackup) {
    // Only read I2C when not backing up to prevent motor EMI from locking the bus
    colorSensor.updateReadings();

    // Don't re-check sensors during cooldown after a backup
    if (millis() - colorBackupEnd >= COLOR_COOLDOWN_MS) {
      // Check if any sensor is seeing white right now
      float avoidAngle = colorSensor.getAvoidAngle();
      if (avoidAngle >= 0.0f) {
        // Trigger backup
        inColorBackup = true;
        colorBackupStart = millis();
        colorBackupAngle = avoidAngle;
        Serial.print("[WHITE LINE] Backing away at angle: ");
        Serial.println(avoidAngle, 0);
      }
    }
  }

  if (inColorBackup) {
    // Drive away from the white line for COLOR_BACKUP_MS ms
    // Orientation is held by turnRate; only translation direction changes
    if (millis() - colorBackupStart < COLOR_BACKUP_MS) {
      robot.drive(colorBackupAngle, basePursueSpeed, turnRate);
      delay(15);
      return; // skip ball-tracking this iteration
    } else {
      inColorBackup = false; // backup complete — resume normal play
      colorBackupEnd = millis(); // start cooldown
      Serial.println("[WHITE LINE] Backup complete, resuming.");
    }
  }

  // ---- 3. Ball Tracking ----
  float driveAngle = 0;
  float activeSpeed = 0;
  static int detectionStreak = 0;

  if (irRing.isBallDetected()) {
    float rawAngle = irRing.getBallAngle();
    float rawStrength = irRing.getBallStrength();

    if (rawStrength >= MIN_IR_STRENGTH) {
      float currentSmoothAngle = atan2f(smoothY, smoothX) * (180.0f / PI);
      if (currentSmoothAngle < 0)
        currentSmoothAngle += 360.0f;
      float diff = fabsf(rawAngle - currentSmoothAngle);
      if (diff > 180.0f)
        diff = 360.0f - diff;

      if (diff < MAX_ANGLE_JUMP || rawStrength > 150.0f) {
        float rawRad = rawAngle * (PI / 180.0f);
        smoothX = (1.0f - ANGLE_SMOOTH_ALPHA) * smoothX +
                  ANGLE_SMOOTH_ALPHA * cosf(rawRad);
        smoothY = (1.0f - ANGLE_SMOOTH_ALPHA) * smoothY +
                  ANGLE_SMOOTH_ALPHA * sinf(rawRad);
      }
    }

    detectionStreak++;

    float ballAngle = atan2f(smoothY, smoothX) * (180.0f / PI);
    if (ballAngle < 0)
      ballAngle += 360.0f;

    if (detectionStreak >= 2) {
      activeSpeed = basePursueSpeed;
    }

    // Adaptive Orbital Logic
    float angError = ballAngle;
    if (angError > 180.0f)
      angError -= 360.0f;

    float baseOffset = angError * 0.8f;
    baseOffset = constrain(baseOffset, -50.0f, 50.0f);

    float damp = 1.0f - ((rawStrength - 80.0f) / 170.0f);
    damp = constrain(damp, 0.0f, 1.0f);

    float finalOffset = baseOffset * damp;
    driveAngle = ballAngle + finalOffset;

    if (driveAngle < 0)
      driveAngle += 360.0f;
    if (driveAngle >= 360.0f)
      driveAngle -= 360.0f;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 50) {
      lastPrint = millis();
      Serial.print("Ball:");
      Serial.print(ballAngle, 0);
      Serial.print(" Drive:");
      Serial.print(driveAngle, 0);
      Serial.print(" Str:");
      Serial.print(rawStrength, 0);
      Serial.print(" MaxIdx:");
      Serial.print(irRing.getStrongestSensor());
      Serial.print(" Turn:");
      Serial.println(turnRate, 0);
    }

  } else {
    detectionStreak = 0;
    smoothX = smoothX * 0.80f + 1.0f * 0.20f;
    smoothY = smoothY * 0.80f;
    static unsigned long lastMissedPrint = 0;
    if (millis() - lastMissedPrint >= 200) {
      lastMissedPrint = millis();
      Serial.println("Ball -> Stale (Braking)");
    }
  }

  robot.drive(driveAngle, activeSpeed, turnRate);
  delay(15);
}

// ============================================================
// BALL-ONLY MODE (TEST_MODE == 4)
// Identical to the original normalLoop() BEFORE colour sensor
// integration — compass heading hold + ball tracking, no colour.
// ============================================================
void ballOnlyLoop() {
  irRing.update();

  // ---- 1. Compass Heading Hold (always computed, used in all states) ----
  float turnRate = 0;
  if (isCalibrated) {
    float currentHeading = compass.getHeading();
    float error = initialHeading - currentHeading;
    if (error > 180.0f)
      error -= 360.0f;
    if (error < -180.0f)
      error += 360.0f;
    if (fabsf(error) > HEADING_DEADZONE) {
      turnRate = kP_Rotation * error;
      turnRate = constrain(turnRate, -80, 80);
    }
  }

  // ---- 3. Ball Tracking ----
  float driveAngle = 0;
  float activeSpeed = 0;
  static int detectionStreak = 0;

  if (irRing.isBallDetected()) {
    float rawAngle = irRing.getBallAngle();
    float rawStrength = irRing.getBallStrength();

    if (rawStrength >= MIN_IR_STRENGTH) {
      float currentSmoothAngle = atan2f(smoothY, smoothX) * (180.0f / PI);
      if (currentSmoothAngle < 0)
        currentSmoothAngle += 360.0f;
      float diff = fabsf(rawAngle - currentSmoothAngle);
      if (diff > 180.0f)
        diff = 360.0f - diff;

      if (diff < MAX_ANGLE_JUMP || rawStrength > 150.0f) {
        float rawRad = rawAngle * (PI / 180.0f);
        smoothX = (1.0f - ANGLE_SMOOTH_ALPHA) * smoothX +
                  ANGLE_SMOOTH_ALPHA * cosf(rawRad);
        smoothY = (1.0f - ANGLE_SMOOTH_ALPHA) * smoothY +
                  ANGLE_SMOOTH_ALPHA * sinf(rawRad);
      }
    }

    detectionStreak++;

    float ballAngle = atan2f(smoothY, smoothX) * (180.0f / PI);
    if (ballAngle < 0)
      ballAngle += 360.0f;

    if (detectionStreak >= 2) {
      activeSpeed = basePursueSpeed;
    }

    // Adaptive Orbital Logic
    float angError = ballAngle;
    if (angError > 180.0f)
      angError -= 360.0f;

    // Deadzone: if ball is within 15° of dead-ahead, drive straight at it.
    // Without this, tiny sensor noise flips the offset sign every few ms,
    // causing the robot to jitter left/right instead of going straight.
    float baseOffset = 0.0f;
    if (fabsf(angError) > 15.0f) {
      baseOffset = angError * 0.8f;
      baseOffset = constrain(baseOffset, -50.0f, 50.0f);
    }

    float damp = 1.0f - ((rawStrength - 80.0f) / 170.0f);
    damp = constrain(damp, 0.0f, 1.0f);

    float finalOffset = baseOffset * damp;
    driveAngle = ballAngle + finalOffset;

    if (driveAngle < 0)
      driveAngle += 360.0f;
    if (driveAngle >= 360.0f)
      driveAngle -= 360.0f;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 50) {
      lastPrint = millis();
      Serial.print("Ball:");
      Serial.print(ballAngle, 0);
      Serial.print(" Drive:");
      Serial.print(driveAngle, 0);
      Serial.print(" Str:");
      Serial.print(rawStrength, 0);
      Serial.print(" MaxIdx:");
      Serial.print(irRing.getStrongestSensor());
      Serial.print(" Turn:");
      Serial.println(turnRate, 0);
    }

  } else {
    detectionStreak = 0;
    smoothX = smoothX * 0.80f + 1.0f * 0.20f;
    smoothY = smoothY * 0.80f;
    static unsigned long lastMissedPrint = 0;
    if (millis() - lastMissedPrint >= 200) {
      lastMissedPrint = millis();
      Serial.println("Ball -> Stale (Braking)");
    }
  }

  robot.drive(driveAngle, activeSpeed, turnRate);
  delay(15);
}

// ============================================================
// COLOUR SENSOR TEST MODE (TEST_MODE == 5)
// Just colour avoidance — no ball tracking.
// Robot brakes until a white line is detected; then it backs
// away at basePursueSpeed for COLOR_BACKUP_MS ms.
// Raw green readings from all 8 channels are printed every 200 ms
// so you can verify wiring and tune the buffer threshold.
// ============================================================
void colorSensorTestLoop() {
  // ---- NO compass during colour test — isolate the colour behaviour ----
  float turnRate = 0;

  // ---- White-line backup ----
  static bool c5Backup = false;
  static unsigned long c5BackupStart = 0;
  static float c5BackupAngle = 0.0f;
  static unsigned long c5BackupEnd = 0;
  static unsigned long c5FirstTrigger = 0;   // for hard safety timeout

  // Hard safety: if we've been in continuous avoidance for >2 seconds, force stop
  if (c5Backup && (millis() - c5FirstTrigger > 2000)) {
    c5Backup = false;
    c5BackupEnd = millis();
    robot.brake();
    Serial.println("[COLOR TEST] SAFETY TIMEOUT — forced brake after 2 seconds.");
    delay(15);
    return;
  }

  if (!c5Backup) {
    // ---- Read sensors ONLY when not backing up ----
    // Motors draw high current during backup, which can cause EMI on the I2C bus
    // and lock up the Wire library + loop. We don't need color data while backing up!
    colorSensor.updateReadings();

    // Print raw readings periodically for calibration
    static unsigned long lastColorPrint = 0;
    if (millis() - lastColorPrint >= 200) {
      lastColorPrint = millis();
      Serial.print("[COLOR] Raw: ");
      colorSensor.printReadings();
    }

    // Don't re-check sensors during cooldown after a backup
    if (millis() - c5BackupEnd >= COLOR_COOLDOWN_MS) {
      float avoidAngle = colorSensor.getAvoidAngle();
      if (avoidAngle >= 0.0f) {
        c5Backup = true;
        c5BackupStart = millis();
        c5FirstTrigger = millis();
        c5BackupAngle = avoidAngle;

        // Print detailed trigger info
        Serial.print("[COLOR TEST] WHITE on angle ");
        Serial.print(avoidAngle, 0);
        Serial.print("  Readings: ");
        colorSensor.printReadings();
        Serial.print("[COLOR TEST] Baselines: ");
        colorSensor.printGreenValues();
      }
    }
  }

  if (c5Backup) {
    if (millis() - c5BackupStart < COLOR_BACKUP_MS) {
      robot.drive(c5BackupAngle, basePursueSpeed, turnRate);
      delay(15);
      return;
    } else {
      // Backup complete — BRAKE IMMEDIATELY
      c5Backup = false;
      c5BackupEnd = millis();
      robot.brake();
      Serial.print("[COLOR TEST] Backup done — BRAKED. Current readings: ");
      colorSensor.printReadings();
    }
  }

  // No white line detected — hold still
  robot.brake();
  delay(15);
}

void loop() {
  if (TEST_MODE == 3) {
    robot.drive(90, 60, 0); // Constant East drive at speed 60
    delay(15);
  } else if (TEST_MODE == 2) {
    motorTestLoop();
  } else if (TEST_MODE == 1) {
    angleOnlyLoop();
  } else if (TEST_MODE == 4) {
    ballOnlyLoop();
  } else if (TEST_MODE == 5) {
    colorSensorTestLoop();
  } else {
    normalLoop(); // TEST_MODE == 0: ball tracking + colour avoidance
  }
}
