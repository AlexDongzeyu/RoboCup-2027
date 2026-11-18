#include "motor.h"
#include <Arduino.h>
#include <math.h>

namespace {
// Existing M1/M2/M3/M4 wiring; physical wheel positions still need verification.
const uint8_t inputA[] = {1, 4, 5, 7};
const uint8_t inputB[] = {2, 3, 6, 8};
const int polarity[] = {-1, -1, 1, 1};

float ramp(float current, float target, int upStep) {
  if (current * target < 0) target = 0;
  int limit = fabsf(target) < fabsf(current) ? Motors::RAMP_DOWN_STEP : upStep;
  return current + fminf(limit, fmaxf(-limit, target - current));
}
}

void Motors::begin() {
  // Teensy 4.1 pwm_init() already sets FlexPWM pins 1..8 to 4,482.29 Hz (<=20 kHz MDD3A spec).
  analogWriteResolution(8);
  stop();
}

void Motors::write(int index, int power) {
  // Enforce the cap at the final pin-write boundary as well as in the mixer.
  int signedPower = constrain(power, -MAX_WHEEL_PWM, MAX_WHEEL_PWM) * polarity[index];
  // MDD3A uses PWM/PWM: turn the inactive input off before powering the other.
  if (signedPower >= 0) {
    analogWrite(inputB[index], 0);
    analogWrite(inputA[index], signedPower);
  } else {
    analogWrite(inputA[index], 0);
    analogWrite(inputB[index], -signedPower);
  }
}

void Motors::drive(float targetForward, float targetClockwise) {
  if (!isfinite(targetForward) || !isfinite(targetClockwise) ||
      targetForward < 0 || targetForward > MAX_FORWARD_PWM || fabsf(targetClockwise) > MAX_TURN_PWM) {
    stop();
    Serial.println("ERROR: motor command outside limits; stopped");
    return;
  }
  // Facing in place clears translation immediately so the robot turns on the spot instead of orbiting.
  forward = targetForward == 0 && targetClockwise != 0 ? 0
      : ramp(forward, targetForward, RAMP_UP_STEP);
  // Flip turn through 0 immediately on sign change; seed in-place turns at breakaway torque.
  if (targetClockwise == 0 || turn * targetClockwise < 0) {
    turn = 0;
  } else if (forward == 0 && turn == 0 && fabsf(targetClockwise) >= MIN_TURN_START_PWM) {
    turn = copysignf(static_cast<float>(MIN_TURN_START_PWM), targetClockwise);
  } else {
    turn = ramp(turn, targetClockwise, TURN_RAMP_UP_STEP);
  }
  if (forward > 0) {
    // Allow responsive steering while keeping |turn| <= 0.80 * forward at all speeds so all 4
    // wheels maintain forward sign continuously without a delayed mid-sprint reversal cycle.
    float limit = fminf(forward * 0.80f, fmaxf(28.0f, forward * DRIVE_TURN_RATIO));
    turn = fminf(limit, fmaxf(-limit, turn));
  }
  apply();
}

void Motors::holdStraight(int maxForward) {
  if (maxForward < 0 || maxForward > MAX_FORWARD_PWM) {
    stop();
    Serial.println("ERROR: hold command outside limits; stopped");
    return;
  }
  forward = ramp(forward, static_cast<float>(maxForward), RAMP_UP_STEP);
  turn = 0;
  apply();
}

void Motors::holdTurn() {
  forward = 0;
  turn = fminf(static_cast<float>(MAX_TURN_PWM), fmaxf(-static_cast<float>(MAX_TURN_PWM), turn));
  apply();
}

void Motors::testChannel(int index, int power) {
  if (index < 0 || index >= 4) {
    stop();
    return;
  }
  forward = 0;
  turn = 0;
  for (int i = 0; i < 4; ++i) {
    applied[i] = (i == index) ? constrain(power, -MAX_WHEEL_PWM, MAX_WHEEL_PWM) : 0;
    write(i, applied[i]);
  }
}

void Motors::apply() {
  float target[] = {forward + turn, -forward + turn,
                    -forward + turn, forward + turn};
  float peak = MAX_WHEEL_PWM;
  for (float value : target) peak = fmaxf(peak, fabsf(value));
  int next[4];
  bool reversing = false;
  for (int i = 0; i < 4; ++i) {
    next[i] = lroundf(target[i] * MAX_WHEEL_PWM / peak);
    if (next[i] * applied[i] < 0) reversing = true;
  }
  // Keep the neutral reversal cycle coordinated instead of driving only two wheels.
  for (int i = 0; i < 4; ++i) {
    applied[i] = reversing ? 0 : next[i];
    write(i, applied[i]);
  }
}

int Motors::maxAppliedPwm() const {
  int peak = 0;
  for (int power : applied) {
    if (abs(power) > peak) peak = abs(power);
  }
  return peak;
}

float Motors::appliedForwardPwm() const {
  return (applied[0] - applied[1] - applied[2] + applied[3]) * 0.25f;
}

float Motors::appliedTurnPwm() const {
  return (applied[0] + applied[1] + applied[2] + applied[3]) * 0.25f;
}

void Motors::signedPwm(int (&commands)[4]) const {
  for (int i = 0; i < 4; ++i) commands[i] = applied[i] * polarity[i];
}

void Motors::stop() {
  forward = 0;
  turn = 0;
  for (int i = 0; i < 4; ++i) {
    applied[i] = 0;
    write(i, 0);
  }
}
