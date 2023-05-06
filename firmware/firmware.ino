#include "FfbWheel.h"
#include "PID_v1.h"
#include <ModbusMaster.h>

#define SLAVE_ID 1
#define MAX485_DE 3
#define MAX485_RE_NEG 2
#define SLAVE_BAUDRATE 115200
#define ENCODER_CPR 10000
#define MAX_TORQUE 300
#define ENDSTOP_TORQUE 100
#define WHEEL_RANGE 1080
#define CONTROL_PERIOD 2000

// AASD
ModbusMaster modbus;
float wheel_turns = (float)WHEEL_RANGE / (float)360;
float joystick_max_position = (wheel_turns * float(ENCODER_CPR)) /2;
int encoder_min = 0;
int encoder_max = ENCODER_CPR - 1;
int torque_setting_position = 200;
int encoder_setting_position = 391;
int turns = 0;
int prev_encoder_value;
int prev_torque = 0;
int prev_x_axis_position = 0;
int x_axis_position = 0;
int encoder_center_position = 0;

// FFB Encoder
int32_t currentVelocity = 0;
int32_t currentAcceleration = 0;
int32_t positionChange = 0;
int32_t maxValue = 32767;
int32_t maxVelocity = 72;
int32_t maxAcceleration = 33;
int32_t maxPositionChange = 1151;

Wheel_ Wheel;

int32_t total_force = 0;
int32_t last_total_force = 0;

double Setpoint, Input, Output;
//double Kp=2, Ki=5, Kd=1;
double Kp = 0.1 , Ki = 30 , Kd =  0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int nextUpdateRun = micros();

void setup() {
  Wheel.begin();
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(0.01);
  myPID.SetOutputLimits(-50, 50);
}

void loop() {
  if (micros() < nextUpdateRun) {
    return;
  }
  nextUpdateRun += CONTROL_PERIOD;
  int prev_x_axis = map(x_axis_position, -joystick_max_position, joystick_max_position, -maxValue - 1, maxValue);
  updatePosition();
  int x_axis = map(x_axis_position, -joystick_max_position, joystick_max_position, -maxValue - 1, maxValue);
  positionChange = x_axis - prev_x_axis;
  if (x_axis_position > joystick_max_position) {
    x_axis = maxValue;
  } else if (x_axis_position < joystick_max_position) {
    x_axis = -maxValue - 1;
  }
  Wheel.xAxis(x_axis);
  Wheel.RecvFfbReport();
  Wheel.write();
  total_force = Wheel.ffbEngine.ForceCalculator(
    x_axis,
    currentVelocity,
    currentAcceleration,
    positionChange,
    maxValue,
    maxVelocity,
    maxAcceleration,
    maxPositionChange
  );    
  if (x_axis >= maxValue) {
    total_force = ENDSTOP_TORQUE;
  } else if (x_axis <= maxValue) {
    total_force = -ENDSTOP_TORQUE;
  }
  setTorque(total_force);
}

void updatePosition(){
  // Encoder
  uint8_t result = modbus.readHoldingRegisters(encoder_setting_position, 1);
  if (result == modbus.ku8MBSuccess)
  {
    int encoder_value = modbus.getResponseBuffer(0);
    int center_offset = encoder_value - encoder_center_position;
    if (center_offset < encoder_min) {
      encoder_value = encoder_max - -center_offset;
    } else {
      encoder_value = encoder_value - encoder_center_position;
    }
    int encoder_position_change = encoder_value - prev_encoder_value;
    if (encoder_value != prev_encoder_value) {
      // Handle new rotation
      if (encoder_position_change > 9000) {
        turns = turns - 1;
      } else if (encoder_position_change < -9000) {
        turns = turns + 1;
      }
      int negative_encoder_value = map(encoder_value, 0, encoder_max, -encoder_max, 0);
      if (turns > 0) {
        x_axis_position = ENCODER_CPR * turns + encoder_value;
      } else if (turns < -1) {
        int corrected_turns = turns + 1;
        x_axis_position = ENCODER_CPR * corrected_turns + negative_encoder_value;
      } else {
        if (turns == -1) {
          x_axis_position = negative_encoder_value;
        } else {
          x_axis_position = encoder_value;
        }
      }
      prev_encoder_value = encoder_value;
    }
  }
}

void setTorque(int torque) {
  modbus.writeSingleRegister(torque_setting_position, torque);
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
