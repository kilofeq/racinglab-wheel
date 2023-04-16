#include "Joystick.h"

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_MULTI_AXIS, 4, 0,
  true, true, false, //X,Y,Z
  false, false, false,//Rx,Ry,Rz
  false, false, false, false, false);

Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};

void setup(){
    Joystick.setXAxisRange(-512, 512);
    mygains.totalGain = 100;//0-100
    mygains.springGain = 100;//0-100
    Joystick.setGains(mygains);
    Joystick.begin();
}

void loop(){
  int value = analogRead(A2);
  myeffectparams[0].springPosition = value - 512; //-512-512
  Joystick.setXAxis(value - 512);
  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);
  int xAxisForce = forces[0];
  int xAxisForcePercentage = forces[0] / 2.55; // Value range -255-255
  delay(1);
}
