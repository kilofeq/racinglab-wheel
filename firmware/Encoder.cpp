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
    currentPosition = modbus.getResponseBuffer(0);
    positionChange = currentPosition - lastPosition;
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
}
