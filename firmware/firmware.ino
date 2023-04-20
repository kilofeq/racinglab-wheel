#include "FfbWheel.h"
#include "Encoder.h"
#include "DigitalWriteFast.h"
#include "PID_v1.h"
#include <ModbusMaster.h>

Wheel_ Wheel;
#define BAUD_RATE 115200

#define SLAVE_ID 1
#define MAX485_DE 3
#define MAX485_RE_NEG 2
#define SLAVE_BAUDRATE 115200

int32_t total_force = 0;
int32_t last_total_force = 0;

double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
double Kp = 0.1 , Ki = 30 , Kd =  0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
bool initialRun = false;

ModbusMaster modbus;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

#include "TorqueModbus.h"
TorqueModbus torqueModbus;

void setup() {
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial1.begin(SLAVE_BAUDRATE, SERIAL_8O1);
  modbus.begin(SLAVE_ID, Serial1);

  // Callbacks allow us to configure the RS485 transceiver correctly
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);

  torqueModbus.setTorque(0, modbus);
  Wheel.begin();
  Input = Wheel.encoder.currentPosition;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
  Serial.begin(BAUD_RATE);
}
void loop() {
  if (initialRun == true ) {
//    position control is not correctly, wheel runs over disired postion serveral times before stop
    torqueModbus.setTorque(10, modbus);
    gotoPosition(Wheel.encoder.minValue);
    gotoPosition(Wheel.encoder.maxValue);
    gotoPosition( 0);
    initialRun = false;
    torqueModbus.setTorque(0, modbus);
  } else
  {
    // assign for re-test without initialRun
    //        Serial.print("currentVelocity: ");
    //        Serial.print(Wheel.encoder.maxVelocity);
    //        Serial.print(" maxAcceleration: ");
    //        Serial.println(Wheel.encoder.maxAcceleration);
    //        Serial.print("   maxPositionChange: ");
//    Serial.println(Wheel.encoder.currentPosition);
//    Wheel.encoder.maxPositionChange = 1151;
//    Wheel.encoder.maxVelocity  = 72;
//    Wheel.encoder.maxAcceleration = 33;
    Wheel.encoder.updatePosition(modbus);
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
  }
  Serial.println(Wheel.encoder.currentPosition);
  Serial.println(total_force);
//  set total gain = 0.2 need replace by wheelConfig.totalGain.
  torqueModbus.setTorque((total_force / 2.55) * -0.5, modbus);
}


void gotoPosition(int32_t targetPosition) {
  Setpoint = targetPosition;
  while (Wheel.encoder.currentPosition != targetPosition) {
    Setpoint = targetPosition;
    Wheel.encoder.updatePosition(modbus);
    Input = Wheel.encoder.currentPosition ;
    myPID.Compute();
    torqueModbus.setTorque(-Output, modbus);
    CalculateMaxSpeedAndMaxAcceleration();
    Serial.print("Encoder Position: ");
    Serial.print(Wheel.encoder.currentPosition);
    Serial.print("  ");
    Serial.print(Setpoint);
    Serial.print("  ");
    Serial.print("Torque: ");
    Serial.println(Output);
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
