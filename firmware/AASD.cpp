#include "AASD.h"

AASD::AASD() {
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  modbus.preTransmission(AASD::preTransmission);
  modbus.postTransmission(AASD::postTransmission);
}

AASD::~AASD() {
}

void AASD::setConfig(WheelConfig wheelConfig) {
  maxAngle = wheelConfig.configMaxAngle;
  float turns = (float)maxAngle / (float)360;
  float maxPosition = (turns * float(ENCODER_CPR)) /2;
  int minValue = 0;
  int maxValue = ENCODER_CPR - 1;
  initVariables();
}

void AASD::initVariables() {
  currentPosition = 0;
  lastPosition = 0;
  correctPosition = 0;
  maxAcceleration = 0;
  maxVelocity = 0;
  lastEncoderTime = (uint32_t) millis();
  lastVelocity = 0;
}

void AASD::updatePosition() {
  uint8_t result = modbus.readHoldingRegisters(ENCODER_SETTING_POSITION, 1);
  if (result == modbus.ku8MBSuccess) {
    int encoderValue = modbus.getResponseBuffer(0);
    int centerOffset = encoderValue - centerPosition;
    if (centerOffset < minValue) {
      encoderValue = maxValue - -centerOffset;
    } else {
      encoderValue = centerOffset;
    }
    int positionChange = encoderValue - prevEncoderValue;
    if (encoderValue != prevEncoderValue) {
      // Handle new rotation
      if (positionChange > 9000) {
        turns = turns - 1;
      } else if (positionChange < -9000) {
        turns = turns + 1;
      }
      int negativeEncoderValue = map(encoderValue, 0, maxValue, -maxValue, 0);
      if (turns > 0) {
        currentPosition = ENCODER_CPR * turns + encoderValue;
      } else if (turns < -1) {
        int correctedTurns = turns + 1;
        currentPosition = ENCODER_CPR * correctedTurns + negativeEncoderValue;
      } else {
        if (turns == -1) {
          currentPosition = negativeEncoderValue;
        } else {
          currentPosition = encoderValue;
        }
      }
      prevEncoderValue = encoderValue;
    }
  }

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

void AASD::setTorque(uint8_t torque) {
  modbus.writeSingleRegister(TORQUE_SETTING_POSITION, torque);
}
