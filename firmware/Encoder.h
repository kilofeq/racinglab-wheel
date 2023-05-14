#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "WheelConfig.h"
#include <ModbusMaster.h>

#define SLAVE_ID 1
#define MAX485_DE 3
#define MAX485_RE_NEG 2
#define SLAVE_BAUDRATE 115200
#define ENCODER_CPR 10000
#define TORQUE_SETTING_POSITION 200
#define ENCODER_SETTING_POSITION 391

class Encoder {
  public:
    Encoder();
    ~Encoder(void);
    ModbusMaster modbus;
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
    void updatePosition(void);
    void setConfig(WheelConfig);
    void initVariables();

    static void preTransmission() {
      digitalWrite(MAX485_RE_NEG, 1);
      digitalWrite(MAX485_DE, 1);
    }
    static void postTransmission() {
      digitalWrite(MAX485_RE_NEG, 0);
      digitalWrite(MAX485_DE, 0);
    }
};

#endif
