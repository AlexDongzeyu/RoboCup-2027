#include <Arduino.h>
#include "IR.h"
#include <limits.h>
#include <math.h>

int pins[] = {23, 24, 25, 26, 27, 38, 39, 40, 41, 14, 15, 18, 19, 20, 21, 22};

void IR::initIR() {
  for (unsigned int i = 0; i < arrayLength(pins); i++) {
    pinMode(pins[i], INPUT);
  }
}

void IR::updateReadings() {
  double minVal = INT_MAX;
  double maxVal = INT_MIN;

  for (int i = 0; i < NUM_IR_PINS; i++) {
    pwReadings[i] = pulseIn(pins[i], HIGH, 800);
    if (pwReadings[i] != 0 && pwReadings[i] < minVal) minVal = pwReadings[i];
    if (pwReadings[i] > maxVal) maxVal = pwReadings[i];
  }

  if (minVal == INT_MAX || maxVal == INT_MIN) {
    for (int i = 0; i < NUM_IR_PINS; i++) {
      readings[i] = 0;
    }
    return;
  }

  double range = maxVal - minVal;
  if (range == 0) range = 1; 

  for (int i = 0; i < NUM_IR_PINS; i++) {
    if (pwReadings[i] == 0) {
      readings[i] = 0;
    } else {
      readings[i] = (float)(1 - ((pwReadings[i] - minVal) / range));
    }
  }
}

float* IR::getReadingsArr() {
  return readings;
}

double* IR::getPWsArr() {
  return pwReadings;
}

float IR::getBallAngle() {
  int n = NUM_IR_PINS;
  float sharpen = 2.5;
  float weightedX = 0.0;
  float weightedY = 0.0;
  float totalWeight = 0.0;

  for (int i = 0; i < n; i++) {
    float weight = readings[i];
    if (weight <= 0.0) continue;

    float angleRad = (-i * 2.0 * M_PI) / n;
    float adjustedWeight = pow(weight, sharpen);

    weightedX += adjustedWeight * cos(angleRad);
    weightedY += adjustedWeight * sin(angleRad);
    totalWeight += adjustedWeight;
  }

  if (totalWeight == 0.0) return -1;

  float angle = atan2(weightedY, weightedX) * 180.0 / M_PI;
  if (angle < 0) angle += 360.0;
  if (angle >= 360.0) angle -= 360.0;

  return angle;
}

// void IR::printReadingsArr() {
//   Serial.print("[ ");
//   for(int i = 0; i < NUM_IR_PINS; i++) {
//     Serial.print(readings[i]);
//     Serial.print(" ");
//   }
//   Serial.println("]");
// }

// void IR::printPWsArr() {
//   double* arr = getPWsArr();
//   Serial.print("[ ");
//   for(int i = 0; i < NUM_IR_PINS; i++) {
//     Serial.print(arr[i]);
//     Serial.print(" ");
//   }
//   Serial.println("]");
// }