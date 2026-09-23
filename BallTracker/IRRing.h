#ifndef IRRING_H
#define IRRING_H

#include <Arduino.h>

class IRRing {
public:
  IRRing();

  // Configures pins and necessary variables
  void begin();

  // Must be called in loop() to refresh sensor readings
  void update();

  bool isBallDetected() const;
  float getBallAngle() const;
  float getBallStrength() const;
  int getStrongestSensor() const;
  int getPinForSensor(int index) const;

  // Returns the normalised+sharpened weight [0.0, 1.0] for a given sensor
  // index.  Used by the diagnostic mode in BallTracker.ino to print a live
  // bar-chart of all sensor weights so hardware problems can be identified.
  float getSensorWeight(int index) const;

  static const int NUM_IR_SENSORS = 16;

private:
  // TSSP40 IR Pins ordered counter-clockwise starting from the front (0°).
  const int irPins[NUM_IR_SENSORS] = {23, 24, 25, 26, 27, 38, 39, 40,
                                      41, 14, 15, 18, 19, 20, 21, 22};

  float sensorAngles[NUM_IR_SENSORS];
  float irValues[NUM_IR_SENSORS]; // normalised + sharpened weights [0.0, 1.0]

  bool ballDetected;
  float ballAngle;
  float ballStrength;
  int strongestSensor;
};

#endif
