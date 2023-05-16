#include "FfbWheel.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "ModbusMaster.h"

ModbusMaster modbus;
Wheel_ Wheel;
#define BAUD_RATE 9600
#define SLAVE_ID 1
#define MAX485_DE 3
#define MAX485_RE_NEG 2
#define SLAVE_BAUDRATE 115200

int32_t total_force = 0;
int32_t last_total_force = 0;
long timeInterval = 250;
unsigned long previousTime = micros();

double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
double Kp = 0.1 , Ki = 30 , Kd =  0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
bool initialRun = true;

void setup() {
  Wheel.begin();
  Input = Wheel.encoder.currentPosition;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
  Serial.begin(BAUD_RATE);
  Serial1.begin(SLAVE_BAUDRATE, SERIAL_8O1);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  modbus.begin(SLAVE_ID, Serial1);
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);
  int8_t result = modbus.readHoldingRegisters(391, 1);
  if (result == modbus.ku8MBSuccess) {
    uint16_t encoderValue = modbus.getResponseBuffer(0);
    Wheel.encoder.setEncoderCenterPosition(encoderValue);
  }
}
void loop() {
  unsigned long currentTime = micros(); // or millis()
  if (currentTime - previousTime > timeInterval) {
    Wheel.encoder.maxPositionChange = 1151;
    Wheel.encoder.maxVelocity  = 72;
    Wheel.encoder.maxAcceleration = 33;
    int8_t result = modbus.readHoldingRegisters(391, 1);
    if (result == modbus.ku8MBSuccess) {
      uint16_t encoderValue = modbus.getResponseBuffer(0);
      Wheel.encoder.updatePosition(encoderValue);
    }
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
    total_force = map(total_force, -10000, 10000, 300, -300);
    //  when reach max and min wheel range, max force to prevent wheel goes over.
    Serial.println(Wheel.encoder.currentPosition);
    if (Wheel.encoder.currentPosition >= Wheel.encoder.maxValue) {
      total_force = -Wheel.wheelConfig.endstopMaxForceConfig;
    } else if (Wheel.encoder.currentPosition <= Wheel.encoder.minValue) {
      total_force = Wheel.wheelConfig.endstopMaxForceConfig;
    }
    modbus.writeSingleRegister(200, total_force * Wheel.wheelConfig.totalGainConfig / 100.0);
    previousTime = currentTime;
  }
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
