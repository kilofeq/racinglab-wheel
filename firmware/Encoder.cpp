#include "Encoder.h"
#include <ModbusMaster.h>

Encoder::Encoder() {
}

Encoder::~Encoder() {
}

void Encoder::setConfig(WheelConfig wheelConfig) {
  cPR = wheelConfig.configCPR ;
  maxAngle = wheelConfig.configMaxAngle;
  inverted = wheelConfig.configInverted;
  resetPosition = wheelConfig.configResetEncoderPosition;
  maxValue = (float)maxAngle / 2 / 360 * cPR ;
  minValue = -maxValue;
  initVariables();
}

void Encoder::initVariables() {
  isInitialized = false;
  prevEncoderValue = 0;
  currentPosition = 0;
  lastPosition = 0;
  correctPosition = 0;
  z1stUp = false;
  maxAcceleration = 0;
  maxVelocity = 0;
  lastEncoderTime = (uint32_t) millis();
  lastVelocity = 0;
}

void Encoder::updatePosition(ModbusMaster modbus) {
  int result = modbus.readHoldingRegisters(391, 1);
  if (result == modbus.ku8MBSuccess) {
    int encoderValue = modbus.getResponseBuffer(0);
    if (!isInitialized) {
      prevEncoderValue = encoderValue;
      isInitialized = true;
    }
    if (encoderValue != prevEncoderValue) {
      int encoderPositionChange = encoderValue - prevEncoderValue;
      // Handle new rotation
      if (encoderPositionChange > 9000) {
        // Corrected position change
        positionChange = cPR - encoderPositionChange;
      } else if (encoderPositionChange < -9000) {
        // Corrected position change
        positionChange = cPR + encoderPositionChange;
      } else {
        positionChange = encoderPositionChange;
      }
      currentPosition = currentPosition + positionChange;
    }
  }
  uint32_t currentEncoderTime = (int32_t) millis();
  int16_t diffTime = (int16_t)(currentEncoderTime - lastEncoderTime);
  if (diffTime > 0) {
    currentVelocity = positionChange / diffTime;
    currentAcceleration = (abs(currentVelocity) - abs(lastVelocity)) / diffTime;
    lastEncoderTime = currentEncoderTime;
    lastVelocity = currentVelocity;
  }
  lastPosition = currentPosition;
}
