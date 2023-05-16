#ifndef AASD_h
#define AASD_h

#include <Arduino.h>
#include "WheelConfig.h"

#define ENCODER_CPR 10000
#define SLAVE_ID 1
#define MAX485_DE 4
#define MAX485_RE_NEG 3
#define SLAVE_BAUDRATE 115200

class AASD {
  public:
    AASD();
    ~AASD(void);
    float turns;
    float maxPosition;
    int minValue;
    int maxValue;
    int centerPosition;
    int prevEncoderValue;
    uint16_t maxAngle;
    uint32_t lastEncoderTime;
    
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
    void updatePosition(uint16_t);
    void setConfig(WheelConfig);
    void initVariables();
};

#endif
