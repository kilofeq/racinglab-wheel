#include "FfbWheel.h"
#include "AASD.h"
#include "PID_v1.h"

Wheel_ Wheel;
#define BAUD_RATE 9600
#define CONTROL_PERIOD 1000

unsigned long nextUpdateTime = 0;
int32_t total_force = 0;
int32_t last_total_force = 0;

double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
double Kp = 0.1 , Ki = 30 , Kd =  0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wheel.aasd.setTorque(0);
  Wheel.begin();
  Input = Wheel.aasd.currentPosition;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
  Serial.begin(BAUD_RATE);
}
void loop() {
  unsigned long currentTime = micros();
  if (currentTime < nextUpdateTime) {
    return;
  }
  currentTime += CONTROL_PERIOD;
  Wheel.aasd.maxPositionChange = 1151;
  Wheel.aasd.maxVelocity  = 72;
  Wheel.aasd.maxAcceleration = 33;
  Wheel.aasd.updatePosition();
  if (Wheel.aasd.currentPosition > Wheel.aasd.maxValue) {
    Wheel.xAxis(32767);
  } else if (Wheel.aasd.currentPosition < Wheel.aasd.minValue) {
    Wheel.xAxis(-32767);
  } else {
    Wheel.xAxis(map(Wheel.aasd.currentPosition, Wheel.aasd.minValue , Wheel.aasd.maxValue, -32768, 32767));
  }

  Wheel.RecvFfbReport();
  Wheel.write();
  total_force = Wheel.ffbEngine.ForceCalculator(Wheel.aasd);    
  total_force = constrain(total_force, -300, 300);
  if (Wheel.aasd.currentPosition >= Wheel.aasd.maxValue) {
    total_force = 100;
  } else if (Wheel.aasd.currentPosition <= Wheel.aasd.minValue) {
    total_force = -100;
  }
  Wheel.aasd.setTorque(total_force);
}
