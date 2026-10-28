#include <Arduino.h>
#include <Wire.h>
#include "colorsensor.h"

void ColorSensor::tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire1.beginTransmission(TCA_ADDR);
  Wire1.write(1 << channel);
  Wire1.endTransmission();
}

bool ColorSensor::i2cDetect(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return (Wire1.endTransmission() == 0);
}

void ColorSensor::init() {
  Wire1.begin();

  for (int i = 0; i < NUM_SENSORS; i++) {
    analogValues[i] = 0;
    greenValues[i] = 0;
  }

  if (!i2cDetect(TCA_ADDR)) {
    Serial.println("TCA9548A not found at 0x70");
    return;
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);
    if (!i2cDetect(TCS_ADDR)) {
      Serial.print("TCS3472 not found on CH");
      Serial.println(i);
    } else {
      if (!tcs.begin(TCS_ADDR, &Wire1)) {
        Serial.print("TCS3472 failed to start on CH");
        Serial.println(i);
      }
    }
  }

  updateReadings();

  unsigned long start = millis();
  while (analogValues[0] == 0 && millis() - start < 2000) {
    updateReadings();
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    greenValues[i] = analogValues[i];
  }
}

uint16_t* ColorSensor::getAnalogValues() {
  return analogValues;
}

void ColorSensor::updateReadings() {
  uint16_t r, g, b, c;
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);
    if (!i2cDetect(TCS_ADDR)) {
      analogValues[i] = 0;
      continue;
    }
    tcs.getRawData(&r, &g, &b, &c);
    analogValues[i] = g;
  }
}

void ColorSensor::printReadings() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(analogValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void ColorSensor::printGreenValues() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.print(greenValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}

float ColorSensor::getAvoidAngle() {
  if (analogValues[0] >= greenValues[0] + buffer) return 180;
  if (analogValues[1] >= greenValues[1] + buffer) return 135;
  if (analogValues[2] >= greenValues[2] + buffer) return 90;
  if (analogValues[3] >= greenValues[3] + buffer) return 45;
  if (analogValues[4] >= greenValues[4] + buffer) return 0;
  if (analogValues[5] >= greenValues[5] + buffer) return 315;
  if (analogValues[6] >= greenValues[6] + buffer) return 270;
  if (analogValues[7] >= greenValues[7] + buffer) return 225;
  return -1;
}