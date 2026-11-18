#pragma once

#include <Arduino.h>
#include "../IR/IR.h"
#include "../motor/motor.h"
#include <math.h>
#include <stdint.h>

enum class DriveState : uint8_t { Acquiring, Aligning, Chasing, BriefHold };

struct DriveCommand {
  float error = 0;
  float forward = 0;
  float turn = 0;
  bool ready = false;
  DriveState state = DriveState::Acquiring;
};

class Tracker {
public:
  // When a close ball enters the front concave holder, the upper IR PCB rim can shade it;
  // keep pushing straight across the field (2.5 s) unless knocked aside or stopped.
  static constexpr uint32_t CAPTURE_PUSH_MS = 2500;
  static constexpr uint32_t BRIEF_HOLD_MS = 220;
  static constexpr uint32_t TURN_HOLD_MS = 180;

  void reset() {
    count = 0;
    smoothX = 1.0f;
    smoothY = 0.0f;
    prevError = 0.0f;
    turning = false;
    aligning = false;
    chasingActive = false;
    direction = 0;
    stallBoost = 0.0f;
    stallStartError = 0.0f;
    stallFrames = 0;
    jumpFrames = 0;
    pushEligible = false;
    closeCaptured = false;
    pushForwardPwm = Motors::MAX_FORWARD_PWM;
    lastTurn = 0.0f;
    lastValidMs = 0;
  }

  int samples() const { return count; }

  DriveCommand update(const BallReading& ball, bool enabled, uint32_t now = millis()) {
    bool valid = ball.detected && isfinite(ball.degrees) &&
        ball.degrees >= 0.0f && ball.degrees < 360.0f &&
        isfinite(ball.strength) && ball.strength >= IR::MIN_DUTY &&
        ball.strength <= 1.0f;
    if (!enabled) {
      reset();
      return valid ? DriveCommand{signedAngle(ball.degrees)} : DriveCommand{};
    }

    uint32_t holdWindow = closeCaptured ? CAPTURE_PUSH_MS : BRIEF_HOLD_MS;

    // While pushing a close captured ball in the recessed holder, ignore weak (<0.28) rear/side
    // wall reflections (>55 deg); any nearby ball slipping out (>=0.28 or <=55 deg) overrides in 20 ms.
    if (valid && closeCaptured && static_cast<uint32_t>(now - lastValidMs) < holdWindow) {
      float rawMagnitude = fabsf(signedAngle(ball.degrees));
      if (rawMagnitude > 55.0f && ball.strength < 0.28f) {
        valid = false;
      }
    }

    if (!valid) {
      uint32_t elapsed = static_cast<uint32_t>(now - lastValidMs);
      if (count >= 2) {
        if (pushEligible && elapsed < holdWindow) {
          return {0.0f, static_cast<float>(pushForwardPwm), 0.0f, true, DriveState::BriefHold};
        }
        if (lastTurn != 0.0f && elapsed < TURN_HOLD_MS) {
          return {prevError, 0.0f, lastTurn, true, DriveState::BriefHold};
        }
      } else if (count == 1 && elapsed < 120) {
        // Do not wipe initial acquisition on a single dropped frame.
        return {prevError, 0.0f, 0.0f, false, DriveState::Acquiring};
      }
      reset();
      return {};
    }

    if (count > 0 && static_cast<uint32_t>(now - lastValidMs) >= holdWindow) {
      reset();
    }

    float rad = ball.degrees * (K_PI / 180.0f);
    float currentDegrees = wrap360(atan2f(smoothY, smoothX) * (180.0f / K_PI));
    float delta = signedAngle(ball.degrees - currentDegrees);

    if (count == 0) {
      smoothX = cosf(rad);
      smoothY = sinf(rad);
      prevError = signedAngle(ball.degrees);
      jumpFrames = 0;
      count = 1;
      lastValidMs = now;
      return {prevError, 0.0f, 0.0f, false, DriveState::Acquiring};
    }

    if (fabsf(delta) > 65.0f) {
      ++jumpFrames;
    } else {
      jumpFrames = 0;
    }

    if (jumpFrames >= 2) {
      // Two consecutive frames confirm a relocated target: snap immediately without ping-ponging on 1-frame post shadows.
      smoothX = cosf(rad);
      smoothY = sinf(rad);
      prevError = signedAngle(ball.degrees);
      stallBoost = 0.0f;
      stallFrames = 0;
      jumpFrames = 0;
      closeCaptured = false;
      chasingActive = false;
      count = 2;
    } else {
      smoothX = 0.45f * smoothX + 0.55f * cosf(rad);
      smoothY = 0.45f * smoothY + 0.55f * sinf(rad);
      float norm = hypotf(smoothX, smoothY);
      if (norm > 1e-6f) {
        smoothX /= norm;
        smoothY /= norm;
      } else {
        smoothX = cosf(rad);
        smoothY = sinf(rad);
      }
      if (count < 2) ++count;
    }

    lastValidMs = now;
    float filteredDegrees = wrap360(atan2f(smoothY, smoothX) * (180.0f / K_PI));
    float error = signedAngle(filteredDegrees);
    float rawError = signedAngle(ball.degrees);
    float absErr = fabsf(error);
    float closingSpeed = fmaxf(0.0f, fabsf(prevError) - absErr);

    // From rest, rotate the holder within 12 deg before sprinting; once chasing, stay in chase up to 36 deg.
    float alignThreshold = chasingActive ? ALIGN_REENTER_DEGREES : INITIAL_ALIGN_DEGREES;
    if (absErr >= alignThreshold) {
      if (!aligning) {
        aligning = true;
        chasingActive = false;
        stallFrames = 0;
        stallStartError = absErr;
      }
    } else if (aligning && absErr <= CHASE_ENTER_DEGREES) {
      aligning = false;
      chasingActive = true;
      stallBoost = 0.0f;
      stallFrames = 0;
    } else if (!aligning) {
      chasingActive = true;
    }

    bool crossedFront = error * prevError < 0.0f && absErr < CORRECT_DEGREES && fabsf(prevError) < 12.0f;
    if (absErr <= CENTERED_DEGREES ||
        (fabsf(rawError) <= CENTERED_DEGREES && absErr <= 6.0f) ||
        crossedFront) {
      turning = false;
      stallBoost = 0.0f;
      stallFrames = 0;
    } else if (absErr >= CORRECT_DEGREES) {
      turning = true;
    }

    float turn = 0.0f;
    if (turning) {
      // Lock turn direction near 180 deg so rear targets never chatter left/right.
      if (absErr < 150.0f || direction == 0) {
        int nextDirection = error >= 0.0f ? 1 : -1;
        if (nextDirection != direction) {
          stallBoost = 0.0f;
          stallFrames = 0;
          stallStartError = absErr;
        }
        direction = nextDirection;
      }

      if (aligning) {
        if (stallFrames == 0) stallStartError = absErr;
        if (++stallFrames >= 6) {
          if (stallStartError - absErr < 3.0f) {
            stallBoost = fminf(48.0f, stallBoost + 14.0f);
          }
          stallFrames = 0;
          stallStartError = absErr;
        }
        float baseTurn = MIN_INPLACE_TURN_PWM + 0.42f * (absErr - CENTERED_DEGREES) + stallBoost;
        float damping = absErr < 35.0f ? (3.2f * closingSpeed) : 0.0f;
        float mag = fminf(static_cast<float>(Motors::MAX_TURN_PWM), fmaxf(56.0f, baseTurn - damping));
        turn = direction * mag;
      } else {
        float baseTurn = 24.0f + 2.4f * (absErr - CENTERED_DEGREES);
        float damping = 2.5f * closingSpeed;
        float mag = fminf(95.0f, fmaxf(16.0f, baseTurn - damping));
        turn = direction * mag;
      }
    } else {
      direction = 0;
    }

    prevError = error;
    lastTurn = turn;

    if (aligning) {
      pushEligible = false;
      closeCaptured = false;
      return {error, 0.0f, turn, true, DriveState::Aligning};
    }

    float alignment = fmaxf(0.0f, fminf(1.0f, (ALIGN_REENTER_DEGREES - absErr) / 24.0f));
    float forward = 135.0f + (Motors::MAX_FORWARD_PWM - 135.0f) * alignment;
    if (absErr <= 12.0f) forward = static_cast<float>(Motors::MAX_FORWARD_PWM);

    pushEligible = true;
    pushForwardPwm = static_cast<int>(lroundf(fmaxf(220.0f, forward)));
    if (absErr <= 28.0f && ball.strength >= 0.30f) {
      closeCaptured = true;
      pushForwardPwm = Motors::MAX_FORWARD_PWM;
    }

    return {error, forward, turn, true, DriveState::Chasing};
  }

private:
  static constexpr float K_PI = 3.14159265358979323846f;
  static constexpr float CENTERED_DEGREES = 3.0f;
  static constexpr float CORRECT_DEGREES = 5.0f;
  static constexpr float INITIAL_ALIGN_DEGREES = 16.0f;
  static constexpr float ALIGN_REENTER_DEGREES = 36.0f;
  static constexpr float CHASE_ENTER_DEGREES = 12.0f;
  static constexpr float MIN_INPLACE_TURN_PWM = 62.0f;

  int count = 0;
  float smoothX = 1.0f;
  float smoothY = 0.0f;
  float prevError = 0.0f;
  bool turning = false;
  bool aligning = false;
  bool chasingActive = false;
  int direction = 0;
  float stallBoost = 0.0f;
  float stallStartError = 0.0f;
  int stallFrames = 0;
  int jumpFrames = 0;
  bool pushEligible = false;
  bool closeCaptured = false;
  int pushForwardPwm = Motors::MAX_FORWARD_PWM;
  float lastTurn = 0.0f;
  uint32_t lastValidMs = 0;

  static float wrap360(float degrees) {
    degrees = fmodf(degrees, 360.0f);
    if (degrees < 0.0f) degrees += 360.0f;
    if (degrees >= 360.0f) degrees -= 360.0f;
    return degrees;
  }

  static float signedAngle(float degrees) {
    degrees = wrap360(degrees);
    if (degrees > 180.0f) degrees -= 360.0f;
    return degrees;
  }
};
