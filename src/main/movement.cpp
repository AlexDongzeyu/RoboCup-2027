#include <Arduino.h>
#include <cmath>  // for fabs(), isnan()
#include "movement.h"
#include <math.h>

// ROBOT 1 White
// const int dirFR = 1; //M1
// const int dirBR = 1; //M2
// const int dirBL = -1; //M3
// const int dirFL = -1; //M4

// Robot 2 No Tape
const int dirFR = 1; //M1
const int dirBR = 1; //M2
const int dirBL = -1; //M3
const int dirFL = -1; //M4

void Movement::initMovement() {
  compass.initialize();
}

bool Movement::isBetween(int lower, int upper, int x) {
  return lower < x && x < upper;
}

void Movement::debug() {
}

// void Movement::debug_sees_border() {
//   if (colorSensor.isDetected()) {
//     Serial.println("on the border");
//   } else {
//     Serial.println("no border detected");
//   }
// }

void Movement::brake() {
  motor_FR.brake();
  motor_BR.brake();
  motor_BL.brake();
  motor_FL.brake();
}

void Movement::rotate(int speed) {
  motor_FR.spin(dirFR * speed);
  motor_BR.spin(dirBR * speed); 
  motor_BL.spin(dirBL * speed);
  motor_FL.spin(dirFL * speed);
}

void Movement::rotate_motor(int speed, String motor) {
  if (motor == "FR") {
    motor_FR.spin(dirFR * speed);
  } else if (motor == "BR") {
    motor_BR.spin(dirBR * speed);
  } else if (motor == "BL") {
    motor_BL.spin(dirBL * speed);
  } else if (motor == "FL") {
    motor_FL.spin(dirFL * speed);
  }
}


// void Movement::basic_move(double theta, int maxSpeed) {

//   if (theta == -1) {
//     brake();
//     return;
//   }


//   // float reading = compass.readCompass();

//   double speeds[4] = {
//     maxSpeed * sin(((theta - 90 + 40) * M_PI) / 180) + spin_index,  // TR
//     maxSpeed * sin(((theta - 90 - 40) * M_PI) / 180) + spin_index,  // BR
//     maxSpeed * sin(((theta + 90 + 40) * M_PI) / 180) + spin_index,  // BL
//     maxSpeed * sin(((theta + 90 - 40) * M_PI) / 180) + spin_index   // TL
//   };

//   motor_FR.spin(speeds[0]);
//   motor_BR.spin(speeds[1]);
//   motor_BL.spin(speeds[2]);
//   motor_FL.spin(speeds[3]);


// }

void Movement::basic_move_with_compass(double theta, int maxSpeed) {


  if (theta == -1) {
    brake();
    return;
  }

  theta += 90.0;// This needs to be commented outw for bot #2



  // // 1. Read current heading (° in [0,360)) relative to north
  float heading = compass.readCompass();
  double error = heading;  // heading needs to be negative for bot #2, and positive for bot #1


  // // 3. Dead-zone and proportional gain
  const double deadzone = 5.0;  // ° within which we consider “on target”
  const double Kp = 0.5;        // tuning parameter: larger→faster correction

  // // 4. Compute rotational correction (zero inside dead-zone)
  double headingCorrection = 0.0;
  if (fabs(error) > deadzone) {
  //   // Negative sign so that positive error (clockwise drift) yields counter-clockwise bias
    headingCorrection = -Kp * error;
  }

  // // catching ball
  // if (!isBetween(0 - COMPASS_BUFF, 0 + COMPASS_BUFF, theta)) {
  //   if (theta <= 180) {
  //     theta = theta + 40;
  //   } else {
  //     theta = theta - 40;
  //   }
  // }

  // 5. Existing translational wheel speeds (sinusoidal mixing)
  double baseSpeeds[4] = {
    maxSpeed * sin(((theta - 90 + 40) * M_PI) / 180.0),  // FR
    maxSpeed * sin(((theta - 90 - 40) * M_PI) / 180.0),  // BR
    maxSpeed * sin(((theta + 90 + 40) * M_PI) / 180.0),  // BL
    maxSpeed * sin(((theta + 90 - 40) * M_PI) / 180.0)   // FL
  };

  // // 6. Mix translation, user spin_index, and heading‐correction bias
  double speeds[4];
  for (int i = 0; i < 4; ++i) {
    speeds[i] = baseSpeeds[i]
                + headingCorrection;  // auto‐correction toward north
  }

  // 7. Command the motors
  motor_FR.spin(dirFR * speeds[0]);
  motor_BR.spin(dirBR * speeds[1]);
  motor_BL.spin(dirBL * speeds[2]);
  motor_FL.spin(dirFL * speeds[3]);

  // Serial.print("TR: ");
  // Serial.println(speeds[0]);
  // Serial.print("BR: ");
  // Serial.println(speeds[1]);
  // Serial.print("TL: ");
  // Serial.println(speeds[2]);
  // Serial.print("BL ");
  // Serial.println(speeds[3]);
  // Serial.println();
}


void Movement::basic_move_with_compass_and_camera(double theta,
                                                  int maxSpeed,
                                                  float camAngle) {

  if (theta == -1) {
    brake();
    return;
  }

  theta += 180;  // This needs to be commented out for bot #2

  // 1. Read current heading [0,360) relative to north
  float heading = compass.readCompass();

  // 2. Decide which “error” to correct:
  //    - If camAngle is finite and within ±180°, we saw the net: use camAngle as error.
  //    - Otherwise fall back to compass error relative to north(0°).
  double error;
  bool netVisible = !isnan(camAngle) && fabs(camAngle) <= 180.0;
  if (netVisible) {
    // camAngle already expresses “degrees away from facing net” (–180…+180)
    error = camAngle;
  } else {
    // compute signed compass error to north
    error = heading;  // heading needs to be negative for bot #2, and positive for bot #1
  }

  // 3. Dead-zone and proportional gain
  const double deadzone = 5.0;  // ±5° tolerated as “on target”
  const double Kp = 0.5;        // tuning: bigger → faster swing back
  double headingCorrection = 0.0;
  if (fabs(error) > deadzone) {
    // Negative sign so positive error → CCW bias, and vice versa
    headingCorrection = Kp * error;
  }

  // catching ball
  if (!isBetween(0 - COMPASS_BUFF, 0 + COMPASS_BUFF, theta)) {
    if (theta <= 180) {
      theta = theta + 40;
    } else {
      theta = theta - 40;
    }
  }

  // 4. Compute your original translation wheel speeds
  double baseSpeeds[4] = {
    maxSpeed * sin(((theta - 90 + 40) * M_PI) / 180.0),  // FR
    maxSpeed * sin(((theta - 90 - 40) * M_PI) / 180.0),  // BR
    maxSpeed * sin(((theta + 90 + 40) * M_PI) / 180.0),  // BL
    maxSpeed * sin(((theta + 90 - 40) * M_PI) / 180.0)   // FL
  };

  // 5. Mix translation + manual spin + auto-heading-correction
  double speeds[4];
  for (int i = 0; i < 4; ++i) {
    speeds[i] = baseSpeeds[i]
                + headingCorrection;  // automatic swing back to net/north
  }

  // 6. Drive the motors
  motor_FR.spin(dirFR * speeds[0]);
  motor_BR.spin(dirBR * speeds[1]);
  motor_BL.spin(dirBL * speeds[2]);
  motor_FL.spin(dirFL * speeds[3]);
}

void Movement::move(double theta, int maxSpeed, bool avoid, float cameraRotationAngle) {
  if (theta == -1) {
    brake();
    return;
  }

  float reading = compass.readCompass();
  float intendedDirection = theta;

  float maxRotation = maxSpeed * 0.2;

  // PID
  static float lastError = 0;
  static float integral = 0;

  /* 
  adjust based on these factors:

  too slow to reach target: increase Kp
  overshoots: decrease Kp or increase Kd
  stops short of target: increase Ki
  slowly goes back and forth: reduce Ki or increase Kd
  reacts too slowly: reduce Kd
  */

  float Kp = 0.7;
  float Ki = 0.0;
  float Kd = 0.2;

  float error = intendedDirection - reading;

  // wrap error to [-180, 180]
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }

  integral += error;
  float derivative = error - lastError;

  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  if (correction > maxRotation) {
    correction = maxRotation;
  }
  if (correction < -maxRotation) {
    correction = -maxRotation;
  }

  spin_index = correction;

  // point north
  // int MINOR_ADJUSTMENT_WINDOW = 70;
  // int COMPASS_MAJOR_ADJUSTMENT_SPEED = 30;

  // if (!isBetween(intendedDirection - COMPASS_BUFF, intendedDirection + COMPASS_BUFF, reading)) {
  //   float speed = min + pow((double)(abs(reading) / 100), 1.5) * maxRotation;
  //   Serial.print("spin: ");
  //   Serial.println(speed);
  //   if (reading > 0) {
  //     speed = speed * -1;
  //   }
  //   spin_index = speed;
  // }

  // // catching ball
  // if (!isBetween(0 - COMPASS_BUFF, 0 + COMPASS_BUFF, theta) && !avoid) {
  //   if (theta <= 180) {
  //     theta = theta + 5;
  //   } else {
  //     theta = theta - 15;
  //   }
  // }

  double speeds[4] = {
    maxSpeed * sin(((theta - 90 + 40) * M_PI) / 180) + spin_index,  // TR
    maxSpeed * sin(((theta - 90 - 40) * M_PI) / 180) + spin_index,  // BR
    maxSpeed * sin(((theta + 90 + 40) * M_PI) / 180) + spin_index,  // BL
    maxSpeed * sin(((theta + 90 - 40) * M_PI) / 180) + spin_index   // TL
  };

  // scale speeds relaive to maxSpeed
  double max = 0;
  for (int i = 0; i < 4; i++) {
    if (abs(speeds[i]) > max) {
      max = abs(speeds[i]);
    }
  }

  if (max != 0 && max < maxSpeed) {
    double scale = (double)(maxSpeed / max);
    for (int i = 0; i < 4; i++) {
      speeds[i] *= scale;
    }
  }

  // point towards ball
  // if (!compass.isBetween(degrees - COMPASS_BUFF, degrees + COMPASS_BUFF, 0)) {
  //   float speed = min + (mult *((abs(degrees) / 180) * (maxSpeed - min)));
  //   if (degrees + COMPASS_BUFF < 0) {
  //     speed = speed * -1;
  //   }
  //   spin_index = speed;
  // }

  // Serial.print("Spin Index: ");
  // Serial.println(spin_index);
  // Serial.print(" | |theta - reading|: ");
  // Serial.print(x);
  // Serial.print(" | theta: ");
  // Serial.print(theta);
  // Serial.print(" | compass reading: ");
  // Serial.println(reading);

  // Serial.println(map(speeds[0] + spin_index, 0, 300, 0, maxSpeed));
  // Serial.println(spin_index);

  // Serial.println(speeds[0] + spin_index);
  motor_FR.spin(dirFR * speeds[0]);
  motor_BR.spin(dirBR * speeds[1]);
  motor_BL.spin(dirBL * speeds[2]);
  motor_FL.spin(dirFL * speeds[3]);

  Serial.print("TR: ");
  Serial.println(speeds[0]);
  Serial.print("BR: ");
  Serial.println(speeds[1]);
  Serial.print("TL: ");
  Serial.println(speeds[2]);
  Serial.print("BL ");
  Serial.println(speeds[3]);
  Serial.println();
  // motor_BR.spin(spin_index);
  // motor_BL.spin(spin_index);
  // motor_FL.spin(spin_index);
  // motor_FR.spin(spin_index);
}