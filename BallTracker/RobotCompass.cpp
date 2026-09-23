#include "RobotCompass.h"

RobotCompass::RobotCompass() : bno(55, 0x28, &Wire1) {
  // Configures BNO055 object:
  // ID: 55
  // Addr: 0x28
  // I2C bus: &Wire1
}

bool RobotCompass::begin() {
  if (!bno.begin()) {
    return false;
  }
  bno.setExtCrystalUse(true);
  return true;
}

float RobotCompass::getHeading() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Offset by 180 degrees because the physical compass is mounted backwards
  float heading = event.orientation.x + 180.0;
  if (heading >= 360.0) {
    heading -= 360.0;
  }

  return heading;
}
