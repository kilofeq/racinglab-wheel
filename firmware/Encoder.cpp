#include "Encoder.h"

Encoder::Encoder() {
}

Encoder::~Encoder() {
}

void Encoder::setConfig(WheelConfig wheelConfig) {
  cPR = wheelConfig.configCPR ;
  maxAngle = wheelConfig.configMaxAngle;
  maxValue = (float)maxAngle / 2 / 360 * cPR ;
  minValue = -maxValue;
  encoderMin = 0;
  encoderMax = wheelConfig.configCPR - 1;
  initVariables();
}

void Encoder::initVariables() {
}

void  Encoder::updatePosition(int encoderValue) {
  if (encoderValue == prevEncoderValue) {
    return;
  }
  int centerOffset = encoderValue - encoderCenterPosition;
  if (centerOffset < encoderMin) {
    encoderValue = encoderMax - -centerOffset;
  } else {
    encoderValue = encoderValue - encoderCenterPosition;
  }
  int encoderPositionChange = encoderValue - prevEncoderValue;
  // Handle new rotation
  if (encoderPositionChange > 9000) {
    turns = turns - 1;
  } else if (encoderPositionChange < -9000) {
    turns = turns + 1;
  }
  int negativeEncoderValue = map(encoderValue, 0, encoderMax, -encoderMax, 0);
  if (turns > 0) {
    currentPosition = cPR * turns + encoderValue;
  } else if (turns < -1) {
    int correctedTurns = turns + 1;
    currentPosition = cPR * correctedTurns + negativeEncoderValue;
  } else {
    if (turns == -1) {
      currentPosition = negativeEncoderValue;
    } else {
      currentPosition = encoderValue;
    }
  }
  prevEncoderValue = encoderValue;
  positionChange = currentPosition - lastPosition;
  uint32_t currentEncoderTime = (int32_t) millis();
  int16_t diffTime = (int16_t)(currentEncoderTime - lastEncoderTime) ;
  if (diffTime > 0) {
    currentVelocity = positionChange / diffTime;
    currentAcceleration = (abs(currentVelocity) - abs(lastVelocity)) / diffTime;
    lastEncoderTime = currentEncoderTime;
    lastVelocity = currentVelocity;
  }
  lastPosition = currentPosition;
}

void Encoder::setEncoderCenterPosition (int encoderValue) {
  encoderCenterPosition = encoderValue;
}

