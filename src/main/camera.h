#include <cmath>
#include <Pixy2SPI_SS.h>

//Kyle Andersen 2025

class Camera {
private:
  Pixy2SPI_SS pixy;
  int32_t centerX;
  int32_t middleThreshold;
  float fov;
  int32_t frameWidth;

  const float knownWidthOfObject = 530.0;
  const float referenceDistance = 1000.0;
  float focalLength;

public:
  Camera(float fieldOfView, int32_t threshold = 50)
    : middleThreshold(threshold), fov(fieldOfView) {}

  void initialize() {
    Serial.println("camera initialization begun");
    pixy.init();
    frameWidth = pixy.frameWidth;
    centerX = frameWidth / 2;
    
    focalLength = 500.0; 

    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
      int detectedWidth = pixy.ccc.blocks[0].m_width;
      if (detectedWidth > 0) {
        focalLength = (detectedWidth * referenceDistance) / knownWidthOfObject;
      }
    }
    Serial.println("camera initialization finished");
  }

  String detectDirection() {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
      int32_t midX = pixy.ccc.blocks[0].m_x;
      if (midX > centerX - middleThreshold && midX < centerX + middleThreshold)
        return "CENTER";
      return (midX < centerX) ? "LEFT" : "RIGHT";
    }
    return "NO BLOCKS";
  }

  float findDistance() {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
      int detectedWidth = pixy.ccc.blocks[0].m_width;
      if (detectedWidth > 0) {
        return (knownWidthOfObject * focalLength) / detectedWidth;
      }
    }
    return 999;
  }

  float calculateRotationAngle() {
    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks > 0) {
      int midX = pixy.ccc.blocks[0].m_x;
      float anglePerPixel = fov / frameWidth;
      return (midX - centerX) * anglePerPixel;
    }
    return 999.0;
  }

  void printStatus() {
    String direction = detectDirection();
    Serial.print("Direction: ");
    Serial.println(direction);

    if (pixy.ccc.numBlocks > 0) {
      float distance = findDistance();
      Serial.print("Distance from Object: ");
      Serial.println(distance > 0 ? String(distance, 2) + " mm" : "N/A");

      float rotationAngle = calculateRotationAngle();
      Serial.print("Rotation Angle: ");
      Serial.print(rotationAngle, 2);
      Serial.println(" degrees");
    } else {
      Serial.println("No object detected.");
    }
  }
};