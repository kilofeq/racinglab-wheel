#include <ModbusMaster.h>
#include "Joystick.h"

#define SLAVE_ID 1
#define MAX485_DE 3
#define MAX485_RE_NEG 2
#define SLAVE_BAUDRATE 115200
#define ENCODER_CPR 10000
#define WHEEL_DEGREES 1080
#define END_STOP_TORQUE 100

float wheel_turns = (float)WHEEL_DEGREES / (float)360;
float joystick_max_position = (wheel_turns * float(ENCODER_CPR)) /2;
int encoder_min = 0;
int encoder_max = ENCODER_CPR - 1;

//X-axis & Y-axis REQUIRED
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
JOYSTICK_TYPE_JOYSTICK, 8, 0,
true, true, false, //X,Y, noZ
false, false, false,//Rx,Ry,Rz
false, false, false, false, false);

Gains mygains[2];
EffectParams myeffectparams[2];
int32_t forces[2] = {0};

ModbusMaster modbus;

int torque_setting_position = 200;
int encoder_setting_position = 391;
int turns = 0;
int prev_encoder_value;
int prev_torque = 0;
int x_axis_position = 0;
int encoder_center_position = 0;

void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(9600);
  Serial1.begin(SLAVE_BAUDRATE, SERIAL_8O1);
  modbus.begin(SLAVE_ID, Serial1);

  // Callbacks allow us to configure the RS485 transceiver correctly
  modbus.preTransmission(preTransmission);
  modbus.postTransmission(postTransmission);

  // FFB
  Joystick.setXAxisRange(-joystick_max_position, joystick_max_position);
  mygains[0].totalGain = 50;//0-100
  mygains[0].springGain = 100;//0-100
  Joystick.setGains(mygains);
  Joystick.begin();
  // Inital encoder value
  uint8_t result = modbus.readHoldingRegisters(encoder_setting_position, 1);
  if (result == modbus.ku8MBSuccess)
  {
    int encoder_value = modbus.getResponseBuffer(0);
    encoder_center_position = encoder_value; 
    prev_encoder_value = encoder_value;
  }
}

void loop()
{
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

  // FFB
  int torque = 0;
  myeffectparams[0].springMaxPosition = joystick_max_position;
  myeffectparams[0].springPosition = x_axis_position;
  Joystick.setXAxis(x_axis_position);
  Joystick.setEffectParams(myeffectparams);
  Joystick.getForce(forces);
  // Endstop
  if (x_axis_position > 0 && x_axis_position + 1 >= joystick_max_position) {
    torque = -END_STOP_TORQUE;
  } else if (x_axis_position < 0 && x_axis_position - 1 <= -joystick_max_position) {
    torque = END_STOP_TORQUE;
  } else {
    torque = forces[0];
  }
  if (prev_torque != torque) {
    modbus.writeSingleRegister(torque_setting_position, torque);
  }
  prev_torque = torque;
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
