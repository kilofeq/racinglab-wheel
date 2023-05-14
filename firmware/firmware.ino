#include "FfbWheel.h"
#include "Encoder.h"
#include "PID_v1.h"

Wheel_ Wheel;
#define BAUD_RATE 115200

int32_t total_force = 0;
int32_t last_total_force = 0;

double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
double Kp = 0.1 , Ki = 30 , Kd =  0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
bool initialRun = true;

void setup() {
  // pwm.begin();
  
  //pwm.setPWM(0);
  Wheel.begin();
  Input = Wheel.encoder.currentPosition;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
  Serial.begin(BAUD_RATE);
}
void loop() {
    Wheel.encoder.maxPositionChange = 1151;
    Wheel.encoder.maxVelocity  = 72;
    Wheel.encoder.maxAcceleration = 33;
    Wheel.encoder.updatePosition();
    if (Wheel.encoder.currentPosition > Wheel.encoder.maxValue) {
      Wheel.xAxis(32767);
    } else if (Wheel.encoder.currentPosition < Wheel.encoder.minValue) {
      Wheel.xAxis(-32767);
    } else {
      Wheel.xAxis(map(Wheel.encoder.currentPosition, Wheel.encoder.minValue , Wheel.encoder.maxValue, -32768, 32767));
    }

    Wheel.RecvFfbReport();
    Wheel.write();
    total_force = Wheel.ffbEngine.ForceCalculator(Wheel.encoder);    
    total_force = constrain(total_force, -255, 255);
    //  Serial.println(Wheel.encoder.currentPosition);
    //  when reach max and min wheel range, max force to prevent wheel goes over.
    if (Wheel.encoder.currentPosition >= Wheel.encoder.maxValue) {
      total_force = 255;
    } else if (Wheel.encoder.currentPosition <= Wheel.encoder.minValue) {
      total_force = -255;
    }
//  set total gain = 0.2 need replace by wheelConfig.totalGain.
  // pwm.setPWM(total_force * 0.2);
}


void gotoPosition(int32_t targetPosition) {
  Setpoint = targetPosition;
  while (Wheel.encoder.currentPosition != targetPosition) {
    Setpoint = targetPosition;
    Wheel.encoder.updatePosition();
    Input = Wheel.encoder.currentPosition ;
    myPID.Compute();
    // pwm.setPWM(-Output);
    CalculateMaxSpeedAndMaxAcceleration();
  }
}

void CalculateMaxSpeedAndMaxAcceleration() {
  if (Wheel.encoder.maxVelocity < abs(Wheel.encoder.currentVelocity)) {
    Wheel.encoder.maxVelocity = abs(Wheel.encoder.currentVelocity);
  }
  if (Wheel.encoder.maxAcceleration < abs(Wheel.encoder.currentAcceleration)) {
    Wheel.encoder.maxAcceleration = abs(Wheel.encoder.currentAcceleration);
  }
  if (Wheel.encoder.maxPositionChange < abs(Wheel.encoder.positionChange)) {
    Wheel.encoder.maxPositionChange = abs(Wheel.encoder.positionChange);
  }
}
