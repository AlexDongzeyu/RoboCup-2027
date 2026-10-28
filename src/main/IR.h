#ifndef IR_H
#define IR_H

#define NUM_IR_PINS 16

class IR {
public:
  float* getReadingsArr();
  double* getPWsArr();
  void updateReadings();
  void initIR();
  float getBallAngle();

private:
  float readings[NUM_IR_PINS];
  double pwReadings[NUM_IR_PINS];
};

template <typename T, size_t N>
constexpr size_t arrayLength(const T (&)[N]) {
  return N;
}

#endif