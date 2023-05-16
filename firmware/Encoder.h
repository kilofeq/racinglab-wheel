#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "WheelConfig.h"

#define interruptA 0
#define interruptB 1
#define interruptZ 2

#define encoderPinA 0
#define encoderPinB 1
#define encoderPinZ 2



class Encoder {
  public:
    Encoder(void);
    ~Encoder(void);
    float turns;
    float joystickMaxPosition;
    int encoderMin;
    int encoderMax;
    int prevEncoderValue;
    uint32_t encoderCenterPosition;
    uint32_t cPR;
    uint16_t maxAngle;
    int32_t maxValue;
    int32_t  minValue;
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
    void setConfig(WheelConfig wheelConfig);
    void initVariables(void);
    void updatePosition(int);
    void setEncoderCenterPosition(int);
  private:
};

#endif
