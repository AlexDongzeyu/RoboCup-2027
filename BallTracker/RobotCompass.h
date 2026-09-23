#ifndef ROBOTCOMPASS_H
#define ROBOTCOMPASS_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>

class RobotCompass {
public:
  RobotCompass();

  // Initializes the compass module with I2C setup
  bool begin();

  // Retrieves the current Euler heading (0 - 360)
  float getHeading();

private:
  Adafruit_BNO055 bno;
};

#endif
