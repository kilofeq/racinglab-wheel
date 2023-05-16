#include "FfbWheel.h"
#include "AASD.h"
#include "PID_v1.h"
#include "ModbusMaster.h"

Wheel_ Wheel;
ModbusMaster modbus;
#define CONTROL_PERIOD 2000

unsigned long nextUpdateTime = 0;
int32_t total_force = 0;
int32_t last_total_force = 0;

double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
double Kp = 0.1 , Ki = 30 , Kd =  0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wheel.begin();
  Input = Wheel.aasd.currentPosition;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
  Serial.begin(9600),
  Serial1.begin(SLAVE_BAUDRATE, SERIAL_8O1);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  modbus.begin(SLAVE_ID, Serial1);
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);
  Wheel.aasd.updatePosition(0);
}

void loop() {
  unsigned long currentTime = micros();
  if (currentTime < nextUpdateTime) {
    return;
  }
  currentTime = currentTime + CONTROL_PERIOD;
  Wheel.aasd.maxPositionChange = 1151;
  Wheel.aasd.maxVelocity  = 72;
  Wheel.aasd.maxAcceleration = 33;
  int8_t result = modbus.readHoldingRegisters(391, 1);
  Serial.println(result);
  if (result == modbus.ku8MBSuccess) {
    uint16_t encoderValue = modbus.getResponseBuffer(0);
    Serial.println(encoderValue);
    Wheel.aasd.updatePosition(encoderValue);
  }
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
  Serial.println(Wheel.aasd.currentPosition);
  Serial.println(Wheel.aasd.currentPosition);
  /* total_force = constrain(total_force, -300, 300);
  if (Wheel.aasd.currentPosition >= Wheel.aasd.maxValue) {
    total_force = 100;
  } else if (Wheel.aasd.currentPosition <= Wheel.aasd.minValue) {
    total_force = -100;
  } */
  modbus.writeSingleRegister(200, total_force);
}

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
