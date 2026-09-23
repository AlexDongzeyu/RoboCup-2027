#include "IRRing.h"
#include <Arduino.h>
#include <math.h>

IRRing::IRRing()
    : ballDetected(false), ballAngle(0), ballStrength(0), strongestSensor(-1) {}

void IRRing::begin() {
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    pinMode(irPins[i], INPUT_PULLUP);
    // Sensor ring is physically counter-clockwise.
    // The array irPins starts at North and naturally proceeds Clockwise:
    // irPins[0] = S1  (North) -> 0 deg
    // irPins[4] = S13 (East)  -> 90 deg
    // irPins[8] = S9  (South) -> 180 deg
    // irPins[12] = S5 (West)  -> 270 deg
    float a_cw = i * (2.0f * PI / NUM_IR_SENSORS);

    sensorAngles[i] = a_cw;

    irValues[i] = 0.0f;
  }
}

void IRRing::update() {
  // ---- Count-based sampling (2 ms window) ----
  int rawCounts[NUM_IR_SENSORS];
  for (int i = 0; i < NUM_IR_SENSORS; i++)
    rawCounts[i] = 0;

  unsigned long startSample = millis();
  int loopCount = 0;

  while (millis() - startSample < 2) {
    loopCount++;
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
      if (digitalRead(irPins[i]) == LOW)
        rawCounts[i]++;
    }
  }

  // Stuck-pin filter (90% threshold)
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    if (rawCounts[i] > loopCount * 0.90)
      rawCounts[i] = 0;
  }

  // Find peak sensor
  int maxCount = 0;
  int maxIndex = -1;
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    if (rawCounts[i] > maxCount) {
      maxCount = rawCounts[i];
      maxIndex = i;
    }
  }

  if (maxCount == 0) {
    ballDetected = false;
    ballAngle = 0;
    ballStrength = 0;
    strongestSensor = -1;
    for (int i = 0; i < NUM_IR_SENSORS; i++)
      irValues[i] = 0.0f;
    return;
  }

  // Normalise counts and sharpen (pow 2)
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    float w = (float)rawCounts[i] / (float)maxCount;
    irValues[i] = w * w;
  }

  // ±2 cluster vector average in Clockwise system
  float xForward = 0.0f;
  float yRight = 0.0f;
  for (int offset = -2; offset <= 2; offset++) {
    int idx = (maxIndex + offset + NUM_IR_SENSORS) % NUM_IR_SENSORS;
    float a = sensorAngles[idx]; // radians, CW
    // In CW system (0=Up, 90=Right):
    //   Forward axis = cos(a)
    //   Right axis   = sin(a)
    xForward += irValues[idx] * cosf(a);
    yRight += irValues[idx] * sinf(a);
  }

  ballDetected = true;
  // atan2(yRight, xForward) gives angle in CW system
  ballAngle = atan2f(yRight, xForward) * (180.0f / PI);
  if (ballAngle < 0)
    ballAngle += 360.0f;

  ballStrength = sqrtf(xForward * xForward + yRight * yRight) * (float)maxCount;
  strongestSensor = maxIndex;
}

bool IRRing::isBallDetected() const { return ballDetected; }
float IRRing::getBallAngle() const { return ballAngle; }
float IRRing::getBallStrength() const { return ballStrength; }
int IRRing::getStrongestSensor() const { return strongestSensor; }

int IRRing::getPinForSensor(int index) const {
  if (index >= 0 && index < NUM_IR_SENSORS)
    return irPins[index];
  return -1;
}

float IRRing::getSensorWeight(int index) const {
  if (index >= 0 && index < NUM_IR_SENSORS)
    return irValues[index];
  return 0.0f;
}
