#pragma once

#include <stdint.h>

struct BallReading {
  bool detected = false;
  float degrees = 0;
  float strength = 0;
  int sensor = -1;
  const char* reason = "NO_VALID_PULSES";
  float contrast = 0;
  float coherence = 0;
  float rawDegrees = 0;
};

class IR {
public:
  static constexpr int COUNT = 16;
  static constexpr uint32_t SAMPLE_WINDOW_US = 10000; // ~12 cycles @ 1.2 kHz, ~6 @ 600 Hz
  static constexpr float MIN_DUTY = 0.008f;
  static constexpr float SATURATED_DUTY = 0.65f;
  static constexpr float MIN_CONTRAST = 0.004f;
  static constexpr float MIN_COHERENCE = 0.12f;

  // Clockwise from front concave holder: front=23, right=27, rear=41, left=19.
  static const uint8_t pins[COUNT];
  void begin();
  BallReading read();
  bool calibrateHolder();
  bool calibrationCancelled() const { return cancelled; }
  bool calibrationHealthy() const { return !calibrationFault; }
  bool hasSavedCalibration() const { return hasCalibration && !calibrationFault; }
  // S1 (pin 23) sits physically at 0 deg above the holder; uncalibrated boots use 0.0 deg safely.
  bool readyForPursuit() const { return true; }
  float holderZero() const { return zeroDegrees; }
  const char* calibrationState() const {
    return calibrationFault ? "FALLBACK" : hasCalibration ? "SAVED" : "DEFAULT";
  }
  void printDiagnostics() const;
  static BallReading bearing(const float duty[COUNT]);

private:
  float rawDuty[COUNT] = {};
  uint32_t edges[COUNT] = {};
  float zeroDegrees = 0;
  bool hasCalibration = false;
  bool calibrationFault = false;
  bool cancelled = false;
};
