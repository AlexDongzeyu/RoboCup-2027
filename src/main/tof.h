//Kyle Andersen 2025

#ifndef TOF_H
#define TOF_H

#include <Wire.h>
#include <VL53L0X.h>

class TOF {
private:
  VL53L0X sensor;

public:
  void initialize() {
    Wire.begin();
    sensor.setTimeout(500);
    

    if (!sensor.init()) {
      Serial.println("Failed to detect and initialize sensor!");
      while (1) {}
    }

    sensor.startContinuous();
  }

  int GetBallRange() {
    int x = sensor.readRangeContinuousMillimeters();
    if(x > 100)
    {
      return -1; //out of sight
    }
    else {
      return x;
    }
  }
};

#endif // TOF_H
