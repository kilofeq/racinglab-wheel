#ifndef WHEELCONFIG_h
#define WHEELCONFIG_h

#include <Arduino.h>

/* Initial config defines */
#define InitialConfigNotDone 0

class WheelConfig {
  public:
    WheelConfig(void);
    ~WheelConfig(void);
    void SetDefault();
    uint8_t constantGainConfig;
    uint8_t rampGainConfig;
    uint8_t squareGainConfig;
    uint8_t sinGainConfig;
    uint8_t triangleGainConfig;
    uint8_t sawToothDownGainConfig;
    uint8_t sawToothUpGainConfig;
    uint8_t springGainConfig;
    uint8_t damperGainConfig;
    uint8_t inertiaGainConfig;
    uint8_t frictionGainConfig;
    uint8_t totalGainConfig;
 

};

#endif
