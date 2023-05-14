#include "WheelConfig.h"

WheelConfig::WheelConfig() {
  SetDefault();
}

WheelConfig::~WheelConfig() {
}

void WheelConfig::SetDefault() {
  configMaxAngle = (uint16_t) 1080;
  constantGainConfig = 100;
  rampGainConfig = 100;;
  squareGainConfig = 100;;
  sinGainConfig = 100;
  triangleGainConfig = 100;;
  sawToothDownGainConfig = 100;;
  sawToothUpGainConfig = 100;;
  springGainConfig = 100;
  damperGainConfig = 100;
  inertiaGainConfig = 100;;
  frictionGainConfig = 100;;
  totalGainConfig = 100;;
}
