#include <Arduino.h>
#include <stdio.h>
#include "src/IR/IR.h"
#include "src/motor/motor.h"
#include "src/movement/movement.h"

// Bench uploads can start paused; ordinary Arduino uploads remain automatic.
#ifndef START_PAUSED
#define START_PAUSED 0
#endif

IR ir;
Motors motors;
Tracker tracker;
bool running = !START_PAUSED;
bool starting = true;
uint32_t bootAt = 0;
uint32_t lastUpdate = 0;
const char* stopReason = "STOPPED";

void stopTracking(const char* reason) {
  running = false;
  stopReason = reason;
  tracker.reset();
  motors.stop();
}

void scheduleTracking() {
  motors.stop();
  tracker.reset();
  running = true;
  starting = true;
  bootAt = lastUpdate = millis();
  stopReason = "STOPPED";
  Serial.println("Run:STARTING_IN_2_SECONDS. Keep clear; s stops.");
}

void printReading(const BallReading& ball, const DriveCommand& drive) {
  char line[416];
  int motorPwm[4];
  motors.signedPwm(motorPwm);
  const char* state = !running ? stopReason
                     : starting ? "STARTING"
                     : drive.state == DriveState::BriefHold ? "PUSH_HOLD"
                     : !ball.detected ? (ball.strength >= IR::MIN_DUTY ? "NO_DIRECTION" : "NO_BALL")
                     : !drive.ready ? "ACQUIRING"
                     : drive.state == DriveState::Aligning ? "ALIGNING" : "TRACKING";
  int length;
  if (ball.detected) {
    float displayAngle = fmodf(roundf(ball.degrees * 10.0f) / 10.0f, 360.0f);
    length = snprintf(line, sizeof(line),
        "Angle:%.1f Raw:%.1f Zero:%.1f Cal:%s Error:%.1f Signal:%.3f Contrast:%.3f Quality:%.2f Sensor:%d Forward:%.1f Turn:%.1f AppliedF:%.1f AppliedT:%.1f PWM:%d/255 Limit:%d M1:%d M2:%d M3:%d M4:%d State:%s FW:IR11-PURSUIT\n",
        displayAngle, ball.rawDegrees, ir.holderZero(), ir.calibrationState(),
        drive.error, ball.strength, ball.contrast, ball.coherence, ball.sensor,
        drive.forward, drive.turn, motors.appliedForwardPwm(), motors.appliedTurnPwm(),
        motors.maxAppliedPwm(), Motors::MAX_WHEEL_PWM,
        motorPwm[0], motorPwm[1], motorPwm[2], motorPwm[3], state);
  } else {
    length = snprintf(line, sizeof(line),
        "Angle:none Zero:%.1f Cal:%s Signal:%.3f Contrast:%.3f Quality:%.2f Reason:%s Forward:%.1f Turn:%.1f AppliedF:%.1f AppliedT:%.1f PWM:%d/255 Limit:%d M1:%d M2:%d M3:%d M4:%d State:%s FW:IR11-PURSUIT\n",
        ir.holderZero(), ir.calibrationState(), ball.strength, ball.contrast, ball.coherence, ball.reason,
        drive.forward, drive.turn, motors.appliedForwardPwm(), motors.appliedTurnPwm(),
        motors.maxAppliedPwm(), Motors::MAX_WHEEL_PWM,
        motorPwm[0], motorPwm[1], motorPwm[2], motorPwm[3], state);
  }
  // Never let a slow serial reader delay a stop.
  if (Serial && length > 0 && length < static_cast<int>(sizeof(line)) &&
      Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  }
}

bool abortRequested() {
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch != '\r' && ch != '\n' && ch != ' ' && ch != '\t') {
      motors.stop();
      Serial.println("MotorTest:CANCELLED.");
      return true;
    }
  }
  return false;
}

bool runMotorSelfTest() {
  // Raised-wheel hardware verification: M1..M4 individually, then CW spin, then Forward drive.
  Serial.println("MotorTest:STARTING_IN_2_SECONDS. RAISE ALL WHEELS; s aborts.");
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (abortRequested()) return false;
    delay(10);
  }
  const int channelSign[4] = {1, -1, -1, 1}; // Forward polarity per channel in mixer space
  for (int ch = 0; ch < 4; ++ch) {
    Serial.print("MotorTest:CHANNEL_M");
    Serial.println(ch + 1);
    t0 = millis();
    while (millis() - t0 < 450) {
      if (abortRequested()) return false;
      motors.testChannel(ch, 90 * channelSign[ch]);
      delay(20);
    }
    motors.stop();
    delay(250);
  }
  Serial.println("MotorTest:CLOCKWISE_TURN");
  t0 = millis();
  while (millis() - t0 < 500) {
    if (abortRequested()) return false;
    motors.drive(0, 80);
    delay(20);
  }
  motors.stop();
  delay(300);
  Serial.println("MotorTest:STRAIGHT_FORWARD");
  t0 = millis();
  while (millis() - t0 < 500) {
    if (abortRequested()) return false;
    motors.drive(120, 0);
    delay(20);
  }
  motors.stop();
  Serial.println("MotorTest:DONE. Stopped; send r to resume tracking.");
  return true;
}

void setup() {
  motors.begin();
  Serial.begin(115200);
  ir.begin();
  running = !START_PAUSED && ir.readyForPursuit();
  stopReason = "STOPPED";
  starting = true;
  bootAt = lastUpdate = millis();
  tracker.reset();
}

void loop() {
  // Auto-starts after 2s; s stops, r/g runs, z calibrates holder zero, t runs raised-wheel test.
  for (int i = 0; i < 32 && Serial.available(); ++i) {
    char rawCmd = Serial.read();
    char command = (rawCmd >= 'A' && rawCmd <= 'Z') ? static_cast<char>(rawCmd - 'A' + 'a') : rawCmd;
    if (command == 's') {
      stopTracking("STOPPED");
    } else if (command == 'r' || command == 'g') {
      scheduleTracking();
    } else if (command == 'z') {
      bool wasRunning = running;
      stopTracking("CALIBRATING");
      bool saved = ir.calibrateHolder();
      bool stopped = ir.calibrationCancelled();
      while (Serial.available()) {
        int ch = Serial.read();
        if (ch == 's' || ch == 'S') stopped = true;
      }
      if (!stopped && (saved || wasRunning)) {
        scheduleTracking();
      } else {
        stopReason = stopped ? "STOPPED" : "CALIBRATION_FAILED";
      }
      lastUpdate = millis();
      return;
    } else if (command == 't') {
      stopTracking("MOTOR_TEST");
      runMotorSelfTest();
      stopTracking("STOPPED");
      lastUpdate = millis();
      return;
    } else if (command != '\r' && command != '\n' && command != ' ' && command != '\t') {
      // Never kill active autonomous tracking on a stray serial byte.
      Serial.println("INFO: commands are s (stop), r or g (run), z (calibrate holder), t (raised-wheel test).");
    }
  }
  uint32_t now = millis();
  if (starting && now - bootAt >= 2000) starting = false;
  if (now - lastUpdate < 20) return;
  if (now - lastUpdate > 100) {
    // Discard stale commands, then recover from fresh readings without rearming.
    motors.stop();
    tracker.reset();
  }
  lastUpdate = now;

  BallReading ball = ir.read();
  DriveCommand drive = tracker.update(ball, running && !starting, millis());
  if (!drive.ready) motors.stop();
  else if (drive.state == DriveState::BriefHold) {
    if (drive.forward > 0) motors.holdStraight(static_cast<int>(lroundf(drive.forward)));
    else motors.holdTurn();
  }
  else motors.drive(drive.forward, drive.turn);

  // The first readings are printed while stopped, before movement is allowed.
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 100 || tracker.samples() == 1) {
    printReading(ball, drive);
    lastPrint = now;
  }
  static uint32_t lastDiagnostics = 0;
  if ((!running || !ball.detected) && now - lastDiagnostics >= 1000) {
    ir.printDiagnostics();
    lastDiagnostics = now;
  }
}