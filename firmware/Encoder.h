#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "WheelConfig.h"
#include <ModbusMaster.h>

class Encoder {
  public:
    Encoder(void);
    ~Encoder(void);
    uint32_t cPR;
    uint16_t maxAngle;
    int32_t maxValue;
    int32_t  minValue;

    bool isInitialized;
    bool inverted;
    bool z1stUp;
    uint32_t lastEncoderTime;

    int32_t  prevEncoderValue;
    int32_t  currentPosition;
    int32_t  lastPosition;
    int32_t  correctPosition;
    int32_t  currentVelocity;
    int32_t  lastVelocity;
    int32_t  maxVelocity;
    int32_t  currentAcceleration;
    int32_t  maxAcceleration;
    int32_t  positionChange;
    int32_t  maxPositionChange;
    void setConfig(WheelConfig wheelConfig);
    void initVariables(void);
    void updatePosition(ModbusMaster modbus);

  private:
    bool resetPosition;
    volatile int8_t oldState;
};

#endif
