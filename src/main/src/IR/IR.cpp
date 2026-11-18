#include "IR.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// This file measures the ball's angle; it does not drive the motors.
// read(): measure pulses -> remove background -> find direction -> apply holder zero.
// calibrateHolder(): save the raw angle that means "in front of the holder".
namespace {
constexpr float kPi = 3.14159265358979323846f;
// Keep the holder zero across power cycles. The marker/check help detect bad stored data.
constexpr uint32_t ZERO_MAGIC = 0x4952325A;
struct HolderCalibration {
  uint32_t magic;
  float degrees;
  uint32_t check;
};

uint32_t angleBits(float degrees) {
  // Use the float's stored bytes for the check, without rounding its angle.
  uint32_t bits;
  memcpy(&bits, &degrees, sizeof(bits));
  return bits;
}

bool validCalibration(const HolderCalibration& saved) {
  return saved.magic == ZERO_MAGIC && isfinite(saved.degrees) &&
      saved.degrees >= 0 && saved.degrees < 360 &&
      saved.check == (ZERO_MAGIC ^ angleBits(saved.degrees) ^ 0xA55A5AA5u);
}

float angularDifference(float degrees, float reference) {
  // Handle the 0/360 boundary: 1 - 359 means +2 degrees, not -358.
  float difference = degrees - reference;
  if (difference > 180) difference -= 360;
  if (difference < -180) difference += 360;
  return difference;
}
bool fitHolder(const float* angles, float& zero, int& valid,
               int& bestCount, float& spread, const char*& reason) {
  // Find a stable group in 50 readings instead of trusting the first reading.
  // Negative angles mean unclear samples; zero is a valid angle.
  constexpr int SAMPLE_COUNT = 50;
  constexpr int MIN_CONSISTENT = 40;
  constexpr float CLUSTER_DEGREES = 12;
  valid = 0;
  bestCount = 0;
  spread = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) if (angles[i] >= 0) ++valid;
  if (valid < MIN_CONSISTENT) {
    reason = "NOT_ENOUGH_CLEAR_READINGS";
    return false;
  }
  float bestSpread = 1e9f;
  float anchor = 0;
  // Try each clear reading as the centre; keep the largest group within +/-12 degrees.
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    if (angles[i] < 0) continue;
    int count = 0;
    float clusterSqSum = 0;
    for (int j = 0; j < SAMPLE_COUNT; ++j) {
      if (angles[j] < 0) continue;
      float difference = angularDifference(angles[j], angles[i]);
      if (fabsf(difference) <= CLUSTER_DEGREES) {
        ++count;
        clusterSqSum += difference * difference;
      }
    }
    if (count > bestCount || (count == bestCount && clusterSqSum < bestSpread)) {
      bestCount = count;
      bestSpread = clusterSqSum;
      anchor = angles[i];
    }
  }
  if (bestCount < MIN_CONSISTENT) {
    reason = "NO_STABLE_ANGLE_CLUSTER";
    return false;
  }
  float sum = 0;
  // Average offsets around this centre so 359 and 1 do not incorrectly average to 180.
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    float degrees = angles[i];
    if (degrees >= 0 && fabsf(angularDifference(degrees, anchor)) <= CLUSTER_DEGREES)
      sum += angularDifference(degrees, anchor);
  }
  zero = anchor + sum / bestCount;
  if (zero < 0) zero += 360;
  if (zero >= 360) zero -= 360;

  // A small overall spread can hide slow movement. Also compare the first/last 10 samples.
  float variance = 0, firstSum = 0, lastSum = 0;
  int firstCount = 0, lastCount = 0;
  for (int i = 0; i < SAMPLE_COUNT; ++i) {
    if (angles[i] < 0 ||
        fabsf(angularDifference(angles[i], anchor)) > CLUSTER_DEGREES) continue;
    float error = angularDifference(angles[i], zero);
    variance += error * error;
    if (i < 10) {
      firstSum += error;
      ++firstCount;
    } else if (i >= SAMPLE_COUNT - 10) {
      lastSum += error;
      ++lastCount;
    }
  }
  spread = sqrtf(variance / bestCount);
  // RMS measures typical error without letting left/right errors cancel.
  // Limit spread and drift to 6 degrees; each end needs at least 7 clear samples.
  if (spread > 6 || firstCount < 7 || lastCount < 7 ||
      fabsf(firstSum / firstCount - lastSum / lastCount) > 6) {
    reason = "MOVING_OR_JITTERING";
    return false;
  }
  return true;
}
}

// Receiver order sets direction: pin23=raw 0, pin27=90, pin41=180, pin19=270.
// The 16 receivers are 22.5 degrees apart; calibration shifts zero to the actual holder.
const uint8_t IR::pins[IR::COUNT] = {
    23, 24, 25, 26, 27, 38, 39, 40, 41, 14, 15, 18, 19, 20, 21, 22};

void IR::begin() {
  for (uint8_t pin : pins) pinMode(pin, INPUT_PULLUP);
  zeroDegrees = 0;
  hasCalibration = false;
  calibrationFault = false;
  // Load saved holder zero from the final 12 EEPROM bytes; fall back to S1=0 deg if blank/corrupt.
  HolderCalibration saved;
  EEPROM.get(EEPROM.length() - sizeof(saved), saved);
  if (validCalibration(saved)) {
    zeroDegrees = saved.degrees;
    hasCalibration = true;
  } else if (saved.magic != UINT32_MAX || saved.check != UINT32_MAX ||
             angleBits(saved.degrees) != UINT32_MAX) {
    calibrationFault = true;
    Serial.println("WARN: EEPROM zero corrupt; falling back to Zero:0.0 (S1). Send z to recalibrate.");
  }
}

BallReading IR::read() {
  // TSSP4038 is active-LOW: count LOW duration and transitions across 10 full 1.2 kHz ball cycles.
  uint32_t low[COUNT] = {};
  bool previous[COUNT];
  for (int i = 0; i < COUNT; ++i) {
    edges[i] = 0;
    previous[i] = digitalRead(pins[i]) == LOW;
  }
  uint32_t samples = 0;
  uint32_t start = micros();
  do {
    for (int i = 0; i < COUNT; ++i) {
      bool active = digitalRead(pins[i]) == LOW;
      low[i] += active;
      edges[i] += active != previous[i];
      previous[i] = active;
    }
    ++samples;
  } while (static_cast<uint32_t>(micros() - start) < SAMPLE_WINDOW_US);

  float duty[COUNT] = {};
  float strongestRaw = 0;
  int activeSensors = 0;
  bool isolatedStuckLow = false;
  for (int i = 0; i < COUNT; ++i) {
    rawDuty[i] = static_cast<float>(low[i]) / samples;
    strongestRaw = fmaxf(strongestRaw, rawDuty[i]);
    if (rawDuty[i] >= MIN_DUTY) ++activeSensors;
  }
  for (int i = 0; i < COUNT; ++i) {
    // Accept any pulsing receiver (edges >= 1, including partial off-gap bridging at close range)
    // OR continuous-LOW saturation (>=0.65 duty, 0 edges) whenever >=2 sensors on the ring see IR.
    // A single isolated 0-edge stuck-LOW pin with 15 dark pins is flagged as a hardware short.
    if ((rawDuty[i] >= MIN_DUTY && edges[i] >= 1) ||
        (rawDuty[i] >= SATURATED_DUTY && activeSensors >= 2)) {
      duty[i] = fminf(1.0f, rawDuty[i]);
    } else if (edges[i] == 0 && rawDuty[i] >= 0.80f && activeSensors == 1) {
      isolatedStuckLow = true;
    }
  }
  BallReading ball = bearing(duty);
  if (ball.sensor < 0) {
    ball.strength = strongestRaw;
    if (isolatedStuckLow) ball.reason = "SATURATED_OR_STUCK_LOW";
    else if (strongestRaw >= MIN_DUTY) ball.reason = "NO_VALID_PULSE_TRAIN";
  }
  ball.rawDegrees = ball.degrees;
  if (ball.detected) {
    ball.degrees = fmodf(ball.rawDegrees - zeroDegrees + 360.0f, 360.0f);
    if (ball.degrees < 0.0f || ball.degrees >= 360.0f) ball.degrees = 0.0f;
  }
  return ball;
}

bool IR::calibrateHolder() {
  // main.ino stops the motors before this call; IR only measures and saves the zero.
  // Recheck the latest 50 readings every 10 samples, up to 150 samples (about 3 seconds).
  constexpr int WINDOW = 50;
  float angles[WINDOW];
  float zero = 0, spread = 0;
  int valid = 0, bestCount = 0;
  const char* reason = "NOT_ENOUGH_CLEAR_READINGS";
  cancelled = false;
  bool stable = false;
  Serial.println("Calibration:SAMPLING. Hold ball on holder centreline for up to 3 seconds; s cancels.");
  for (int sample = 0; sample < 150; ++sample) {
    BallReading ball = read();
    if (sample >= WINDOW) {
      for (int i = 1; i < WINDOW; ++i) angles[i - 1] = angles[i];
    }
    // Use RAW angles, not angles already corrected by the previous zero.
    // Cluster consistency (40/50 within +/-12 deg, RMS <=6 deg) enforces stability even in reflective rooms.
    angles[sample < WINDOW ? sample : WINDOW - 1] =
        ball.detected && ball.coherence >= 0.30f ? ball.rawDegrees : -1;
    delay(10);
    for (int n = 0; n < 32 && Serial.available(); ++n) {
      int ch = Serial.read();
      if (ch == 's' || ch == 'S') {
        cancelled = true;
        Serial.println("Calibration:CANCELLED. Motors stopped; previous zero unchanged.");
        return false;
      }
    }
    if (sample >= WINDOW - 1 && (sample + 1) % 10 == 0 &&
        fitHolder(angles, zero, valid, bestCount, spread, reason)) {
      stable = true;
      break;
    }
  }
  // Failed sampling leaves the previous saved zero unchanged.
  if (!stable) {
    char message[176];
    snprintf(message, sizeof(message),
        "Calibration:ERROR %s Clear:%d/50 Consistent:%d/50 Spread:%.1f. Motors stopped; previous zero unchanged.",
        reason, valid, bestCount, spread);
    Serial.println(message);
    return false;
  }

  HolderCalibration saved = {ZERO_MAGIC, zero,
      ZERO_MAGIC ^ angleBits(zero) ^ 0xA55A5AA5u};
  int address = EEPROM.length() - sizeof(saved);
  // Invalidate the marker first. Write angle/check, then mark the record valid last.
  EEPROM.put(address, uint32_t{0});
  EEPROM.put(address + 4, saved.degrees);
  EEPROM.put(address + 8, saved.check);
  EEPROM.put(address, saved.magic);
  // Read back before using the new zero; do not report success if storage failed.
  HolderCalibration verified;
  EEPROM.get(address, verified);
  if (!validCalibration(verified) || verified.degrees != zero) {
    calibrationFault = true;
    Serial.println("ERROR: holder calibration storage failed; motors must remain paused.");
    return false;
  }
  zeroDegrees = zero;
  hasCalibration = true;
  calibrationFault = false;
  char message[112];
  snprintf(message, sizeof(message),
      "Calibration:SAVED Zero:%.1f Consistent:%d/50 Spread:%.1f.",
      zero, bestCount, spread);
  Serial.println(message);
  return true;
}

void IR::printDiagnostics() const {
  // Example: 23=180/24 means pin23 was active on 18% of reads, with 24 state changes.
  char line[384];
  int length = snprintf(line, sizeof(line), "RawIR duty_per_1000/edges:");
  for (int i = 0; i < COUNT; ++i) {
    int written = snprintf(line + length, sizeof(line) - length, " %u=%ld/%lu",
        pins[i], lroundf(rawDuty[i] * 1000), static_cast<unsigned long>(edges[i]));
    if (written < 0 || written >= static_cast<int>(sizeof(line)) - length) {
      Serial.println("ERROR: IR diagnostic formatting");
      return;
    }
    length += written;
  }
  line[length++] = '\n';
  // Do not block sensing just to wait for space in the USB print buffer.
  if (Serial && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
  }
}

BallReading IR::bearing(const float duty[COUNT]) {
  float signal[COUNT] = {};
  float sorted[COUNT] = {};
  int peak = -1;
  for (int i = 0; i < COUNT; ++i) {
    if (isfinite(duty[i]) && duty[i] >= MIN_DUTY && duty[i] <= 1.0f) {
      signal[i] = duty[i];
      if (peak < 0 || signal[i] > signal[peak]) peak = i;
    }
    int j = i;
    while (j > 0 && sorted[j - 1] > signal[i]) {
      sorted[j] = sorted[j - 1];
      --j;
    }
    sorted[j] = signal[i];
  }
  if (peak < 0) return {};

  float medianBg = (sorted[COUNT / 2 - 1] + sorted[COUNT / 2]) * 0.5f;

  // Extreme ring-wide saturation (>=9 sensors >0.92, e.g. ultra-close uncovered or steady-carrier ball):
  // the ball sits 180 deg opposite the shadowed/weaker rear sensors.
  if (medianBg > 0.92f) {
    float minSignal = sorted[0];
    float satContrast = signal[peak] - minSignal;
    if (satContrast < MIN_CONTRAST) {
      return {false, 0, signal[peak], peak, "UNIFORM_SIGNAL", satContrast, 0};
    }
    float sx = 0, sy = 0, sTotal = 0;
    for (int i = 0; i < COUNT; ++i) {
      // Cap per-sensor shadow weight so one post-occluded pin cannot skew a wide saturated arc.
      float w = fminf(0.35f, fmaxf(0.0f, signal[peak] - signal[i]));
      float rad = i * (2.0f * kPi / COUNT);
      sTotal += w;
      sx -= w * cosf(rad);
      sy -= w * sinf(rad);
    }
    float satCoherence = sTotal > 0.0f ? fminf(1.0f, hypotf(sx, sy) / sTotal) : 0.0f;
    if (sTotal <= 0.0f || satCoherence < 0.08f) {
      return {false, 0, signal[peak], peak, "UNIFORM_SIGNAL", satContrast, satCoherence};
    }
    float satDegrees = atan2f(sy, sx) * (180.0f / kPi);
    if (satDegrees < 0) satDegrees += 360.0f;
    if (satDegrees >= 360.0f) satDegrees = 0;
    return {true, satDegrees, signal[peak], peak, "OK", satContrast, fmaxf(0.55f, satCoherence)};
  }

  // Median background strips the 8-sensor Step-1 room-bounce pedestal (~0.30..0.35) cleanly
  // even when 2..4 sensors are shadowed by internal pillars/wires.
  float background = medianBg;
  float contrast = signal[peak] - background;
  if (contrast < MIN_CONTRAST) {
    return {false, 0, signal[peak], peak, "UNIFORM_SIGNAL", contrast, 0};
  }

  float excess[COUNT] = {};
  float ringTotal = 0, ringX = 0, ringY = 0;
  for (int i = 0; i < COUNT; ++i) {
    excess[i] = fmaxf(0.0f, signal[i] - background);
    float w = excess[i] * excess[i];
    float rad = i * (2.0f * kPi / COUNT);
    ringTotal += w;
    ringX += w * cosf(rad);
    ringY += w * sinf(rad);
  }
  if (ringTotal <= 0.0f) {
    return {false, 0, signal[peak], peak, "UNIFORM_SIGNAL", contrast, 0};
  }

  // Symmetric +/-5 cosine bell filter finds the true illuminated hemisphere without hopping across post dips.
  int bestCenter = peak;
  float bestScore = -1.0f;
  float lobeScore[COUNT] = {};
  for (int k = 0; k < COUNT; ++k) {
    float score = 0.0f;
    for (int offset = -5; offset <= 5; ++offset) {
      int idx = (k + offset + COUNT) % COUNT;
      score += excess[idx] * cosf(offset * (kPi / 12.0f));
    }
    lobeScore[k] = score;
    if (score > bestScore) {
      bestScore = score;
      bestCenter = k;
    }
  }

  // Reject ONLY virtually identical 180-deg twin sources; never reject real asymmetric room reflections.
  float oppScore = lobeScore[(bestCenter + COUNT / 2) % COUNT];
  float ringCoherence = hypotf(ringX, ringY) / ringTotal;
  if (oppScore >= 0.95f * bestScore && ringCoherence < 0.06f) {
    return {false, 0, signal[peak], peak, "AMBIGUOUS_DIRECTION", contrast, ringCoherence};
  }

  // Sum excess^2 across +/-5 sensors (+/-112.5 deg) around bestCenter so rear-wall bounce cannot cancel the front vector.
  float hx = 0, hy = 0, hTotal = 0;
  for (int offset = -5; offset <= 5; ++offset) {
    int idx = (bestCenter + offset + COUNT) % COUNT;
    float w = excess[idx] * excess[idx];
    float rad = idx * (2.0f * kPi / COUNT);
    hTotal += w;
    hx += w * cosf(rad);
    hy += w * sinf(rad);
  }

  float coherence = hTotal > 0.0f ? fminf(1.0f, hypotf(hx, hy) / hTotal) : 0.0f;
  if (hTotal <= 0.0f || coherence < MIN_COHERENCE) {
    return {false, 0, signal[peak], peak, "AMBIGUOUS_DIRECTION", contrast, coherence};
  }

  float degrees = atan2f(hy, hx) * (180.0f / kPi);
  if (degrees < 0) degrees += 360.0f;
  if (degrees >= 360.0f) degrees = 0;
  return {true, degrees, signal[peak], peak, "OK", contrast, coherence};
}
