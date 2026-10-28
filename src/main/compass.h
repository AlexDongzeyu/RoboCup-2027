#ifndef COMPASS_H
#define COMPASS_H

#include "Adafruit_BNO055.h"
#include <Wire.h>

class Compass {
public:
  /**
   * Initialize pins to Adafruit_BNO055
   */
  void initialize();

  /**
   * Returns absolute (max 180 degrees) x-axis angle 
   */
  float readCompass();

private:
  Adafruit_BNO055 bno{55, 0x28, &Wire1};
};

#endif